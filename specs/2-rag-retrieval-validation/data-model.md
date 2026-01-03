# Data Model: RAG Retrieval Pipeline Validation

## Retrieval Query Entity

**Description**: Represents a text query submitted for similarity search against vector database

**Fields**:
- `query_text` (string, required): The original text query
- `query_embedding` (array of floats, required): Vector representation of the query
- `k_value` (integer, required): Number of top results to retrieve
- `created_at` (datetime, required): Timestamp of query execution

**Validation Rules**:
- `query_text` must not be empty
- `query_embedding` must be an array of floats with consistent dimensionality (1024 for Cohere multilingual model)
- `k_value` must be a positive integer (typically 1-20)
- `created_at` must be a valid timestamp

## Retrieved Chunk Entity

**Description**: Represents a text segment returned from the vector database with relevance score and metadata

**Fields**:
- `chunk_id` (string, required): Unique identifier for the retrieved chunk
- `content` (string, required): The actual text content of the chunk
- `similarity_score` (float, required): Semantic similarity score to the query (0-1 range)
- `source_url` (string, required): URL of the original document
- `metadata` (object, required): Additional information like title, section, etc.
  - `title` (string): Title of the source page
  - `section` (string): Section or heading of the chunk
  - `position` (integer): Position of the chunk within the original document
- `retrieved_at` (datetime, required): Timestamp of retrieval

**Validation Rules**:
- `chunk_id` must be a valid identifier
- `content` must not be empty
- `similarity_score` must be between 0 and 1
- `source_url` must be a valid URL format
- `retrieved_at` must be a valid timestamp

## Validation Result Entity

**Description**: Represents the validation results for a retrieval operation

**Fields**:
- `query` (string, required): The original query text
- `retrieved_chunks` (list of RetrievedChunk, required): List of retrieved chunks
- `validation_passed` (boolean, required): Whether validation was successful
- `accuracy_score` (float, optional): Accuracy metric for retrieved content (0-1 range)
- `validation_details` (object, optional): Detailed validation results
  - `metadata_matches` (boolean): Whether metadata matches source documents
  - `url_validation_passed` (boolean): Whether source URLs are valid
  - `content_accuracy` (float): Accuracy of content retrieval
- `executed_at` (datetime, required): Timestamp of validation execution

**Validation Rules**:
- `query` must not be empty
- `retrieved_chunks` must be a non-empty list
- `validation_passed` must be a boolean value
- `accuracy_score` must be between 0 and 1 if provided
- `executed_at` must be a valid timestamp