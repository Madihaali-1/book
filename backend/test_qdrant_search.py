#!/usr/bin/env python3
"""
Simple test script to verify Qdrant search functionality without hitting Cohere API
"""

import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_qdrant_connection_and_search():
    """Test Qdrant connection and search functionality directly"""

    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')

    if not qdrant_url:
        raise ValueError("QDRANT_URL environment variable is required")
    if not qdrant_api_key:
        raise ValueError("QDRANT_API_KEY environment variable is required")

    # Connect to Qdrant
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False
    )

    collection_name = "document_embeddings"

    # Test connection by getting collection info
    try:
        collection_info = client.get_collection(collection_name)
        logger.info(f"Successfully connected to Qdrant collection: {collection_name}")
        logger.info(f"Collection has {collection_info.points_count} vectors")
        logger.info(f"Vector size: {collection_info.config.params.vectors.size}")
        logger.info(f"Distance: {collection_info.config.params.vectors.distance}")
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        return False

    # Check available methods and use the appropriate one based on Qdrant client version
    if hasattr(client, 'query_points'):
        logger.info("Using 'query_points' method (newer Qdrant version)")
        search_method = 'query_points'
    elif hasattr(client, 'search'):
        logger.info("Using 'search' method (older Qdrant version)")
        search_method = 'search'
    elif hasattr(client, 'search_points'):
        logger.info("Using 'search_points' method (another older version)")
        search_method = 'search_points'
    else:
        logger.error(f"No known search method found. Available methods: {[m for m in dir(client) if not m.startswith('_')]}")
        return False

    # Try to perform a simple search with a dummy vector (all zeros)
    # This won't return meaningful results but will test if the method works
    try:
        dummy_vector = [0.0] * 1024  # 1024-dim vector to match Cohere embeddings

        if search_method == 'query_points':
            search_result = client.query_points(
                collection_name=collection_name,
                query=dummy_vector,
                limit=3
            )
            results = search_result.points  # Extract the points from the response
        elif search_method == 'search':
            results = client.search(
                collection_name=collection_name,
                query_vector=dummy_vector,
                limit=3
            )
        else:  # search_points
            results = client.search_points(
                collection_name=collection_name,
                vector=dummy_vector,
                limit=3
            )

        logger.info(f"Search method '{search_method}' executed successfully")
        logger.info(f"Returned {len(results)} results (with dummy vector, so 0 is expected)")

        # List available methods on the result objects to understand the structure
        if results:
            result = results[0]
            logger.info(f"Result object type: {type(result)}")
            logger.info(f"Result object attributes: {[attr for attr in dir(result) if not attr.startswith('_')]}")

        return True

    except Exception as e:
        logger.error(f"Error in search operation: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    logger.info("Testing Qdrant connection and search functionality...")
    success = test_qdrant_connection_and_search()
    if success:
        logger.info("✅ Qdrant search functionality test passed!")
    else:
        logger.error("❌ Qdrant search functionality test failed!")