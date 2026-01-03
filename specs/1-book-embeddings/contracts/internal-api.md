# Internal API Contracts: URL Ingestion & Embedding Pipeline

## URL Fetcher Module

### fetch_docusaurus_pages(base_url: str) -> List[CrawledPage]
**Purpose**: Fetch all public pages from a Docusaurus website

**Input**:
- `base_url`: The base URL of the Docusaurus site to crawl

**Output**:
- List of `CrawledPage` objects containing the fetched content

**Errors**:
- `InvalidURLError`: If the base URL is malformed
- `ConnectionError`: If unable to connect to the site
- `CrawlingError`: If the crawling process fails

**Preconditions**:
- The provided URL must be a valid Docusaurus site
- The site must be publicly accessible

**Postconditions**:
- All public pages from the site are fetched and returned
- Each page contains both raw HTML and processed content

### extract_content(html_content: str) -> str
**Purpose**: Extract relevant text content from HTML, removing navigation elements

**Input**:
- `html_content`: Raw HTML content from a page

**Output**:
- Clean text content with navigation and other non-content elements removed

**Errors**:
- `ParseError`: If the HTML cannot be parsed

**Preconditions**:
- The input must be valid HTML content

**Postconditions**:
- Returns clean text content suitable for chunking
- Navigation, headers, and footers are removed

## Text Processing Module

### chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]
**Purpose**: Split text into overlapping chunks of specified size

**Input**:
- `text`: The text to be chunked
- `chunk_size`: Maximum size of each chunk (default: 512)
- `overlap`: Overlap between chunks (default: 50)

**Output**:
- List of text chunks

**Errors**:
- `ValueError`: If chunk_size or overlap parameters are invalid

**Preconditions**:
- Input text must not be empty
- chunk_size must be greater than overlap

**Postconditions**:
- Returns list of text chunks of appropriate size
- Chunks maintain semantic boundaries where possible

## Embedding Module

### generate_embeddings(texts: List[str]) -> List[List[float]]
**Purpose**: Generate embeddings for a list of text chunks

**Input**:
- `texts`: List of text chunks to embed

**Output**:
- List of embedding vectors (each vector as a list of floats)

**Errors**:
- `APIError`: If the embedding API is unavailable
- `RateLimitError`: If API rate limits are exceeded
- `InvalidInputError`: If input texts are invalid

**Preconditions**:
- Cohere API key must be configured
- Input texts must be within API limits

**Postconditions**:
- Returns embedding vectors for all input texts
- Each embedding vector has consistent dimensionality

## Storage Module

### store_embeddings(chunks: List[DocumentChunk]) -> bool
**Purpose**: Store document chunks with embeddings in Qdrant

**Input**:
- `chunks`: List of DocumentChunk objects to store

**Output**:
- Boolean indicating success or failure

**Errors**:
- `StorageError`: If the storage operation fails
- `ConnectionError`: If unable to connect to Qdrant

**Preconditions**:
- Qdrant connection must be configured
- Embeddings must be properly formatted

**Postconditions**:
- All chunks are stored in Qdrant with proper indexing
- Returns true if all chunks were successfully stored

### test_similarity_search(query: str) -> List[DocumentChunk]
**Purpose**: Test similarity search functionality

**Input**:
- `query`: Query text for similarity search

**Output**:
- List of DocumentChunk objects that match the query

**Errors**:
- `SearchError`: If the search operation fails
- `ConnectionError`: If unable to connect to Qdrant

**Preconditions**:
- Qdrant must contain stored embeddings
- Query text must not be empty

**Postconditions**:
- Returns relevant document chunks based on semantic similarity
- Results are ranked by similarity score