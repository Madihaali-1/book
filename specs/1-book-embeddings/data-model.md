# Data Model: URL Ingestion & Embedding Pipeline

## Document Chunk Entity

**Description**: Represents a segment of text extracted from a Docusaurus page with its vector embedding

**Fields**:
- `chunk_id` (string, required): Unique identifier for the chunk (UUID format)
- `url` (string, required): Source URL of the original document
- `content` (string, required): The actual text content of the chunk
- `embedding` (array of floats, required): Vector representation of the content
- `metadata` (object, optional): Additional information like page title, section, etc.
  - `title` (string): Title of the source page
  - `section` (string): Section or heading of the chunk
  - `position` (integer): Position of the chunk within the original document
- `created_at` (datetime, required): Timestamp of when the chunk was created

**Validation Rules**:
- `chunk_id` must be a valid UUID
- `url` must be a valid URL format
- `content` must not be empty
- `embedding` must be an array of floats with consistent dimensionality
- `created_at` must be a valid timestamp

## Crawled Page Entity

**Description**: Represents a page fetched from a Docusaurus website before cleaning and chunking

**Fields**:
- `page_id` (string, required): Unique identifier for the page (UUID format)
- `url` (string, required): The URL of the page
- `title` (string, required): Title of the page
- `content` (string, required): Raw HTML content before cleaning
- `processed_content` (string, required): Cleaned text content
- `status` (string, required): Status of crawling (success, error, processing)
- `fetched_at` (datetime, required): Timestamp of when the page was fetched
- `word_count` (integer, optional): Number of words in the processed content
- `error_message` (string, optional): Error details if status is error

**Validation Rules**:
- `page_id` must be a valid UUID
- `url` must be a valid URL format
- `title` must not be empty
- `status` must be one of: "success", "error", "processing"
- `fetched_at` must be a valid timestamp

## Embedding Configuration Entity

**Description**: Configuration parameters for the embedding process

**Fields**:
- `model_name` (string, required): Name of the embedding model to use
- `chunk_size` (integer, required): Maximum size of text chunks (in tokens)
- `chunk_overlap` (integer, required): Overlap between chunks (in tokens)
- `batch_size` (integer, required): Number of chunks to process in each API call
- `max_retries` (integer, required): Maximum number of retry attempts for API calls
- `delay_between_requests` (float, optional): Delay between API requests in seconds

**Validation Rules**:
- `chunk_size` must be between 100 and 2048
- `chunk_overlap` must be less than `chunk_size`
- `batch_size` must be between 1 and 100
- `max_retries` must be between 1 and 10