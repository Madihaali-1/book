"""
OpenAI Agents SDK Function Tool for RAG Retrieval

This module implements a function tool for the OpenAI Agents SDK that integrates
with the existing Qdrant-based retrieval pipeline to provide RAG (Retrieval-Augmented Generation) capabilities.
"""
import os
import logging
from typing import List, Dict, Any
from datetime import datetime
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from agents import function_tool
import openai
from openai import OpenAI

# Load environment variables from .env file
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass  # dotenv not available, will rely on system environment

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the classes from retrieve.py to reuse the functionality
class QdrantConnector:
    """Module to connect to Qdrant and load vector collections (from retrieve.py)"""

    def __init__(self):
        qdrant_url = os.getenv('QDRANT_URL', 'http://localhost:6333')  # Default to local Qdrant
        qdrant_api_key = os.getenv('QDRANT_API_KEY')  # API key is optional for local Qdrant

        if not qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")
        # Note: qdrant_api_key is optional for local instances

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
    """Module to convert text queries to embeddings using Cohere with fallback to OpenRouter (from retrieve.py)"""

    def __init__(self):
        # Initialize Cohere client
        cohere_api_key = os.getenv('COHERE_API_KEY')
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        self.cohere_client = cohere.Client(cohere_api_key)
        self.cohere_model = "embed-multilingual-v3.0"

        # Initialize OpenRouter client as fallback for embeddings
        openrouter_api_key = os.getenv('OPENROUTER_API_KEY')
        if not openrouter_api_key:
            raise ValueError("OPENROUTER_API_KEY environment variable is required")

        # Use OpenAI client but point to OpenRouter's API
        self.openrouter_client = OpenAI(
            api_key=openrouter_api_key,
            base_url="https://openrouter.ai/api/v1"
        )
        # Using a compatible embedding model from OpenRouter
        self.openrouter_model = "text-embedding-3-small"

    def process_query(self, query_text: str) -> List[float]:
        """Convert text query to embedding vector using Cohere with OpenRouter fallback"""
        if not query_text.strip():
            raise ValueError("Query text cannot be empty")

        from cohere.errors import TooManyRequestsError
        import time
        import openai

        # First, try Cohere with enhanced retry logic
        max_retries = 5
        base_delay = 1  # Start with 1 second delay
        last_error = None

        for attempt in range(max_retries):
            try:
                response = self.cohere_client.embed(
                    texts=[query_text],
                    model=self.cohere_model,
                    input_type="search_query"  # Appropriate for search queries
                )

                embedding = response.embeddings[0]
                logger.info(f"Successfully generated embedding with Cohere for query: {query_text[:50]}...")
                return embedding

            except TooManyRequestsError as e:
                last_error = e
                if attempt < max_retries - 1:
                    delay = base_delay * (2 ** attempt)  # Exponential backoff
                    logger.warning(f"Rate limited by Cohere API. Waiting {delay}s before retry {attempt + 1}/{max_retries}")
                    time.sleep(delay)
                else:
                    logger.error(f"Cohere API rate limited after {max_retries} attempts, falling back to OpenRouter: {e}")
                    break  # Break to try OpenRouter fallback
            except Exception as e:
                last_error = e
                logger.error(f"Cohere embedding failed (attempt {attempt + 1}/{max_retries}): {e}")
                if attempt < max_retries - 1:
                    delay = base_delay * (2 ** attempt)  # Exponential backoff
                    time.sleep(delay)
                else:
                    logger.error(f"Cohere embedding failed after {max_retries} attempts, falling back to OpenRouter")
                    break  # Break to try OpenRouter fallback

        # If Cohere fails, try OpenRouter as fallback
        logger.info("Attempting OpenRouter embedding as fallback...")
        openrouter_retries = 3
        openrouter_delay = 1

        for attempt in range(openrouter_retries):
            try:
                response = self.openrouter_client.embeddings.create(
                    input=[query_text],
                    model=self.openrouter_model
                )

                # Extract embedding
                embedding = response.data[0].embedding
                logger.info(f"Successfully generated embedding with OpenRouter for query: {query_text[:50]}...")
                return embedding

            except openai.RateLimitError as e:
                if attempt < openrouter_retries - 1:
                    delay = openrouter_delay * (2 ** attempt)  # Exponential backoff
                    logger.warning(f"Rate limited by OpenRouter API. Waiting {delay}s before retry {attempt + 1}/{openrouter_retries}")
                    time.sleep(delay)
                else:
                    logger.error(f"OpenRouter API also rate limited after {openrouter_retries} attempts: {e}")
                    raise Exception(f"Both Cohere and OpenRouter failed. Cohere error: {str(last_error)}, OpenRouter error: {str(e)}")
            except Exception as e:
                if attempt < openrouter_retries - 1:
                    delay = openrouter_delay * (2 ** attempt)  # Exponential backoff
                    logger.warning(f"OpenRouter embedding failed (attempt {attempt + 1}/{openrouter_retries}): {e}, retrying...")
                    time.sleep(delay)
                else:
                    logger.error(f"OpenRouter embedding failed after {openrouter_retries} attempts: {e}")
                    raise Exception(f"Both Cohere and OpenRouter failed. Cohere error: {str(last_error)}, OpenRouter error: {str(e)}")


class RetrievedChunk:
    """Represents a text segment returned from the vector database with relevance score and metadata (from retrieve.py)"""
    def __init__(self, chunk_id: str, content: str, similarity_score: float, source_url: str, metadata: Dict[str, Any], retrieved_at: datetime):
        self.chunk_id = chunk_id
        self.content = content
        self.similarity_score = similarity_score
        self.source_url = source_url
        self.metadata = metadata
        self.retrieved_at = retrieved_at


class SimilaritySearch:
    """Module to perform top-k similarity search in Qdrant (from retrieve.py)"""

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


@function_tool
def retrieve_content(query: str, k: int = 5) -> Dict[str, Any]:
    """
    Retrieve relevant content from book documentation based on user query.

    Args:
        query: The search query to find relevant content
        k: Number of results to retrieve (default: 5)

    Returns:
        Dictionary containing retrieved chunks in JSON format
    """
    # Initialize the Qdrant connector and query processor
    qdrant_connector = QdrantConnector()
    query_processor = QueryProcessor()

    # Process the query to get embeddings
    query_embedding = query_processor.process_query(query)

    # Perform similarity search
    similarity_search = SimilaritySearch(qdrant_connector)
    retrieved_chunks = similarity_search.search_similar_chunks(query_embedding, k)

    # Format the results for OpenAI
    formatted_results = []
    for chunk in retrieved_chunks:
        formatted_results.append({
            "chunk_id": chunk.chunk_id,
            "content": chunk.content,
            "similarity_score": chunk.similarity_score,
            "source_url": chunk.source_url,
            "metadata": chunk.metadata
        })

    # Handle case where no relevant content is found
    if len(formatted_results) == 0:
        logger.info(f"No relevant content found for query: {query}")
        return {
            "retrieved_chunks": [],
            "total_results": 0,
            "message": "No relevant content found in the documentation for this query."
        }

    return {
        "retrieved_chunks": formatted_results,
        "total_results": len(formatted_results)
    }