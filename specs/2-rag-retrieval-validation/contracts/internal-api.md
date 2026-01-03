# Internal API Contracts: RAG Retrieval Pipeline Validation

## Qdrant Connector Module

### connect_to_qdrant() -> QdrantClient
**Purpose**: Connect to Qdrant using environment variables

**Input**: None (uses environment variables)

**Output**: QdrantClient instance

**Errors**:
- `ConnectionError`: If unable to connect to Qdrant
- `AuthenticationError`: If credentials are invalid

**Preconditions**:
- QDRANT_URL and QDRANT_API_KEY environment variables must be set

**Postconditions**:
- Returns a valid QdrantClient connection

### get_collection_info(client: QdrantClient, collection_name: str) -> dict
**Purpose**: Get information about the vector collection

**Input**:
- `client`: QdrantClient instance
- `collection_name`: Name of the collection to query

**Output**:
- Dictionary with collection information including point count, vector dimensions, etc.

**Errors**:
- `CollectionNotFound`: If the collection doesn't exist
- `ConnectionError`: If unable to communicate with Qdrant

**Preconditions**:
- Valid QdrantClient connection must exist
- Collection name must be valid

**Postconditions**:
- Returns collection metadata

## Query Processor Module

### process_query(query_text: str) -> List[float]
**Purpose**: Convert text query to embedding vector using Cohere

**Input**:
- `query_text`: The text query to convert to embedding

**Output**:
- List of floats representing the query embedding vector

**Errors**:
- `APIError`: If the Cohere API is unavailable
- `InvalidInputError`: If query text is invalid

**Preconditions**:
- Cohere API key must be configured
- Query text must not be empty

**Postconditions**:
- Returns embedding vector for the query text

## Similarity Search Module

### search_similar_chunks(query_embedding: List[float], k: int = 5) -> List[RetrievedChunk]
**Purpose**: Perform similarity search in Qdrant and return top-k chunks

**Input**:
- `query_embedding`: Vector representation of the query
- `k`: Number of top results to retrieve (default: 5)

**Output**:
- List of RetrievedChunk objects with similarity scores and metadata

**Errors**:
- `SearchError`: If the search operation fails
- `ConnectionError`: If unable to connect to Qdrant

**Preconditions**:
- Query embedding must be properly formatted
- Qdrant collection must contain stored embeddings

**Postconditions**:
- Returns top-k most similar chunks based on cosine similarity

## Validation Module

### validate_results(query: str, results: List[RetrievedChunk]) -> ValidationResult
**Purpose**: Validate retrieved results against original source documents

**Input**:
- `query`: The original query text
- `results`: List of retrieved chunks to validate

**Output**:
- ValidationResult object with validation status and accuracy metrics

**Errors**:
- `ValidationError`: If validation cannot be performed
- `DataIntegrityError`: If retrieved content doesn't match source

**Preconditions**:
- Results must contain valid RetrievedChunk objects
- Query text must not be empty

**Postconditions**:
- Returns validation results with accuracy metrics
- Validates source URL and metadata integrity