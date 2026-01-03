"""
RAG Retrieval Pipeline Validation

This script validates the RAG retrieval pipeline by:
1. Connecting to Qdrant and loading existing vector collections
2. Accepting a test query and performing top-k similarity search
3. Validating results using returned text, metadata, and source URLs
"""
import os
import argparse
import logging
import time
from typing import List, Dict, Any
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
from dataclasses import dataclass
from datetime import datetime
from cohere.errors import TooManyRequestsError


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class RetrievalQuery:
    """Represents a text query submitted for similarity search against vector database"""
    query_text: str
    query_embedding: List[float]
    k_value: int
    created_at: datetime


@dataclass
class RetrievedChunk:
    """Represents a text segment returned from the vector database with relevance score and metadata"""
    chunk_id: str
    content: str
    similarity_score: float
    source_url: str
    metadata: Dict[str, Any]
    retrieved_at: datetime


@dataclass
class ValidationResult:
    """Represents the validation results for a retrieval operation"""
    query: str
    retrieved_chunks: List[RetrievedChunk]
    validation_passed: bool
    accuracy_score: float
    validation_details: Dict[str, Any]
    executed_at: datetime


class QdrantConnector:
    """Module to connect to Qdrant and load vector collections"""

    def __init__(self):
        qdrant_url = os.getenv('QDRANT_URL')
        qdrant_api_key = os.getenv('QDRANT_API_KEY')

        if not qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")
        if not qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")

        self.client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False  # Using HTTP for simplicity
        )

        self.collection_name = "document_embeddings"

    def connect_to_qdrant(self) -> QdrantClient:
        """Connect to Qdrant using environment variables"""
        try:
            # Test connection by getting collection info
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"Successfully connected to Qdrant collection: {self.collection_name}")
            logger.info(f"Collection has {collection_info.points_count} vectors")
            return self.client
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            raise

    def get_collection_info(self, collection_name: str) -> dict:
        """Get information about the vector collection"""
        try:
            collection_info = self.client.get_collection(collection_name)
            return {
                "points_count": collection_info.points_count,
                "vector_size": collection_info.config.params.vectors.size,
                "distance": collection_info.config.params.vectors.distance
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise


class QueryProcessor:
    """Module to convert text queries to embeddings using Cohere"""

    def __init__(self):
        api_key = os.getenv('COHERE_API_KEY')
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.client = cohere.Client(api_key)
        self.model = "embed-multilingual-v3.0"  # Using same model as ingestion

    def process_query(self, query_text: str) -> List[float]:
        """Convert text query to embedding vector using Cohere"""
        if not query_text.strip():
            raise ValueError("Query text cannot be empty")

        max_retries = 5
        base_delay = 1  # Start with 1 second delay

        for attempt in range(max_retries):
            try:
                response = self.client.embed(
                    texts=[query_text],
                    model=self.model,
                    input_type="search_query"  # Appropriate for search queries
                )

                embedding = response.embeddings[0]
                logger.info(f"Generated embedding for query: {query_text[:50]}...")
                return embedding

            except TooManyRequestsError as e:
                if attempt < max_retries - 1:
                    delay = base_delay * (2 ** attempt)  # Exponential backoff
                    logger.warning(f"Rate limited by Cohere API. Waiting {delay}s before retry {attempt + 1}/{max_retries}")
                    time.sleep(delay)
                else:
                    logger.error(f"Failed to generate query embedding after {max_retries} attempts: {e}")
                    raise
            except Exception as e:
                logger.error(f"Error generating query embedding: {e}")
                raise


class SimilaritySearch:
    """Module to perform top-k similarity search in Qdrant"""

    def __init__(self, qdrant_connector: QdrantConnector):
        self.qdrant_connector = qdrant_connector
        self.collection_name = qdrant_connector.collection_name

    def search_similar_chunks(self, query_embedding: List[float], k: int = 5) -> List[RetrievedChunk]:
        """Perform similarity search in Qdrant and return top-k chunks"""
        try:
            # Check available methods and use the appropriate one based on Qdrant client version
            if hasattr(self.qdrant_connector.client, 'query_points'):
                # Newer version: use query_points method
                results = self.qdrant_connector.client.query_points(
                    collection_name=self.collection_name,
                    query=query_embedding,
                    limit=k
                ).points  # Extract the points from the response
            elif hasattr(self.qdrant_connector.client, 'search'):
                # Older version: use search method
                results = self.qdrant_connector.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=k
                )
            elif hasattr(self.qdrant_connector.client, 'search_points'):
                # Another older version: use search_points method
                results = self.qdrant_connector.client.search_points(
                    collection_name=self.collection_name,
                    vector=query_embedding,
                    limit=k
                )
            else:
                raise AttributeError("Qdrant client does not have any known search method. Available methods: {}".format([m for m in dir(self.qdrant_connector.client) if not m.startswith('_')]))

            retrieved_chunks = []
            for result in results:
                # Handle different result formats based on Qdrant client version
                chunk_id = getattr(result, 'id', getattr(getattr(result, 'payload', {}), 'id', 'unknown'))
                content = getattr(getattr(result, 'payload', {}), 'get', lambda x, d="": d)("content", "") if hasattr(result, 'payload') and callable(getattr(result, 'payload', {}).get) else getattr(getattr(result, 'payload', {}), 'content', "")

                # Try to get similarity score (could be 'score' or 'distance' depending on version)
                similarity_score = getattr(result, 'score', getattr(result, 'similarity', 0))

                source_url = getattr(getattr(result, 'payload', {}), 'get', lambda x, d="": d)("url", "") if hasattr(result, 'payload') and callable(getattr(result, 'payload', {}).get) else getattr(getattr(result, 'payload', {}), 'url', "")

                # Handle metadata - try different access patterns
                if hasattr(result, 'payload') and hasattr(result.payload, 'get'):
                    # Payload is a dictionary-like object
                    result_payload = result.payload
                elif isinstance(getattr(result, 'payload', {}), dict):
                    # Payload is a dictionary
                    result_payload = result.payload
                else:
                    # Fallback to empty dict
                    result_payload = {}

                metadata = {
                    "title": result_payload.get("title", ""),
                    "section": result_payload.get("section", ""),
                    "position": result_payload.get("position", 0)
                }

                chunk = RetrievedChunk(
                    chunk_id=str(chunk_id),  # Ensure it's a string
                    content=content,
                    similarity_score=similarity_score,
                    source_url=source_url,
                    metadata=metadata,
                    retrieved_at=datetime.now()
                )
                retrieved_chunks.append(chunk)

            logger.info(f"Found {len(retrieved_chunks)} similar chunks for query")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error in similarity search: {e}")
            # Provide detailed error info to help debug
            available_methods = [m for m in dir(self.qdrant_connector.client) if not m.startswith('_')]
            logger.error(f"Available methods on Qdrant client: {available_methods}")
            raise


class ResultValidator:
    """Module to validate retrieved results against source content"""

    def validate_results(self, query: str, results: List[RetrievedChunk]) -> ValidationResult:
        """Validate retrieved results against original source documents"""
        if not results:
            logger.warning("No results to validate")
            return ValidationResult(
                query=query,
                retrieved_chunks=[],
                validation_passed=False,
                accuracy_score=0.0,
                validation_details={
                    "metadata_matches": False,
                    "url_validation_passed": False,
                    "content_accuracy": 0.0,
                    "message": "No results returned from search"
                },
                executed_at=datetime.now()
            )

        # Validate metadata and source URLs
        metadata_matches = all(chunk.metadata and chunk.source_url for chunk in results)
        url_validation_passed = all(chunk.source_url and chunk.source_url.startswith('http') for chunk in results)

        # Calculate content accuracy based on similarity scores
        avg_similarity = sum(chunk.similarity_score for chunk in results) / len(results) if results else 0
        content_accuracy = min(avg_similarity * 2, 1.0)  # Normalize to 0-1 range

        validation_passed = metadata_matches and url_validation_passed and content_accuracy > 0.1

        validation_details = {
            "metadata_matches": metadata_matches,
            "url_validation_passed": url_validation_passed,
            "content_accuracy": content_accuracy,
            "total_results": len(results),
            "avg_similarity_score": avg_similarity
        }

        logger.info(f"Validation completed: passed={validation_passed}, accuracy={content_accuracy:.2f}")
        return ValidationResult(
            query=query,
            retrieved_chunks=results,
            validation_passed=validation_passed,
            accuracy_score=content_accuracy,
            validation_details=validation_details,
            executed_at=datetime.now()
        )


def main():
    """Main function to orchestrate the full retrieval validation pipeline"""
    parser = argparse.ArgumentParser(description='RAG Retrieval Pipeline Validation')
    parser.add_argument('--query', type=str, required=True, help='Query text for retrieval validation')
    parser.add_argument('--k', type=int, default=5, help='Number of results to retrieve (default: 5)')

    args = parser.parse_args()

    logger.info("Starting RAG Retrieval Pipeline Validation")
    logger.info(f"Configuration: Query='{args.query[:50]}...', K={args.k}")

    try:
        # Step 1: Connect to Qdrant
        qdrant_connector = QdrantConnector()
        client = qdrant_connector.connect_to_qdrant()

        # Get collection info
        collection_info = qdrant_connector.get_collection_info(qdrant_connector.collection_name)
        logger.info(f"Collection info: {collection_info}")

        # Step 2: Process query
        query_processor = QueryProcessor()
        query_embedding = query_processor.process_query(args.query)

        # Create retrieval query object
        retrieval_query = RetrievalQuery(
            query_text=args.query,
            query_embedding=query_embedding,
            k_value=args.k,
            created_at=datetime.now()
        )

        # Step 3: Perform similarity search
        similarity_search = SimilaritySearch(qdrant_connector)
        retrieved_chunks = similarity_search.search_similar_chunks(query_embedding, args.k)

        # Step 4: Validate results
        validator = ResultValidator()
        validation_result = validator.validate_results(args.query, retrieved_chunks)

        # Output validation results
        logger.info(f"Validation completed: {validation_result.validation_passed}")
        logger.info(f"Accuracy score: {validation_result.accuracy_score:.2f}")
        logger.info(f"Retrieved {len(validation_result.retrieved_chunks)} chunks")

        # Print details of top result
        if validation_result.retrieved_chunks:
            top_result = validation_result.retrieved_chunks[0]
            logger.info(f"Top result - URL: {top_result.source_url}")
            logger.info(f"Similarity score: {top_result.similarity_score:.3f}")
            logger.info(f"Content preview: {top_result.content[:100]}...")

        # Print validation details
        logger.info(f"Validation details: {validation_result.validation_details}")

    except Exception as e:
        logger.error(f"Pipeline failed with error: {e}")
        raise


def test_validation():
    """Function to test the validation pipeline with a simple query"""
    try:
        # Use a simple test query
        test_query = "What is the main topic of this documentation?"

        qdrant_connector = QdrantConnector()
        client = qdrant_connector.connect_to_qdrant()

        query_processor = QueryProcessor()
        query_embedding = query_processor.process_query(test_query)

        similarity_search = SimilaritySearch(qdrant_connector)
        retrieved_chunks = similarity_search.search_similar_chunks(query_embedding, k=3)

        validator = ResultValidator()
        validation_result = validator.validate_results(test_query, retrieved_chunks)

        logger.info(f"Test validation completed: {validation_result.validation_passed}")
        logger.info(f"Test accuracy: {validation_result.accuracy_score:.2f}")
        logger.info(f"Test retrieved {len(validation_result.retrieved_chunks)} chunks")

        return validation_result.validation_passed
    except Exception as e:
        logger.error(f"Test validation failed: {e}")
        return False


if __name__ == "__main__":
    main()