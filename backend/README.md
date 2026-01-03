# Book Embeddings Pipeline

This backend service implements a complete pipeline for crawling Docusaurus documentation sites, generating embeddings, and storing them in a vector database for RAG applications.

## Features

- Crawls Docusaurus documentation sites
- Extracts and cleans text content
- Chunks text into appropriate sizes for embeddings
- Generates vector embeddings using Cohere models
- Stores embeddings in Qdrant Cloud with metadata
- Provides similarity search functionality

## Prerequisites

- Python 3.11+
- uv package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Install uv package manager:
   ```bash
   pip install uv
   ```

2. Install dependencies:
   ```bash
   uv sync
   ```

3. Create a `.env` file with your configuration:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   DOCUSAURUS_URL=https://your-docusaurus-site.com
   CHUNK_SIZE=512
   CHUNK_OVERLAP=50
   BATCH_SIZE=10
   ```

## Usage

### Ingestion Pipeline

Run the complete ingestion pipeline:

```bash
python main.py
```

#### Command Line Options

The ingestion pipeline supports various command line options:

```bash
# Basic usage with environment variables
python main.py

# Specify URL directly (overrides environment variable)
python main.py --url https://example-docusaurus-site.com

# Customize chunk size and overlap
python main.py --chunk-size 1024 --chunk-overlap 100

# Perform a dry run without storing embeddings
python main.py --dry-run

# Full options
python main.py --url <URL> --chunk-size <SIZE> --chunk-overlap <OVERLAP> --batch-size <SIZE> --dry-run
```

### Retrieval Validation

To validate the retrieval pipeline with a query:

```bash
# Basic usage with a query
python retrieve.py --query "your search query here"

# With custom number of results (k-value)
python retrieve.py --query "your query" --k 10

# Examples
python retrieve.py --query "What are the main features?"
python retrieve.py --query "How do I configure the system?" --k 3
```

#### Retrieval Validation Options

The retrieval validation script supports various options:

```bash
# Query with default k-value (5 results)
python retrieve.py --query "What is this documentation about?"

# Query with custom k-value
python retrieve.py --query "API usage examples" --k 7

# Full options
python retrieve.py --query "your query text" --k <number_of_results>
```

### Environment Variables

The system requires the following environment variables:

- `COHERE_API_KEY`: Your Cohere API key (required)
- `QDRANT_URL`: Your Qdrant Cloud cluster URL (required)
- `QDRANT_API_KEY`: Your Qdrant API key (required)
- `DOCUSAURUS_URL`: The Docusaurus site to crawl (required for ingestion only)
- `CHUNK_SIZE`: Size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50)
- `BATCH_SIZE`: Number of chunks per API call (default: 10)

## Troubleshooting

### Common Issues

1. **API Rate Limits**: If you encounter rate limit errors, the pipeline has built-in retry mechanisms with exponential backoff. You can also reduce the batch size to lower the API request frequency.

2. **Connection Issues**: Ensure your Qdrant URL and API key are correct, and that your Cohere API key is valid.

3. **Crawling Problems**: If the crawler can't access certain pages, check if they're behind authentication or have robots.txt restrictions.

4. **Memory Issues**: For large documentation sites, consider processing in smaller batches or increasing system memory.

### Environment Variables

Make sure all required environment variables are properly set before running the pipeline. Missing or incorrect values will cause the pipeline to fail.

## Architecture

The system consists of several modules:

- **Ingestion Pipeline (main.py)**:
  - URL Fetcher: Crawls Docusaurus sites and extracts content
  - Text Cleaner: Removes navigation and extracts relevant text
  - Chunker: Splits content into appropriately sized chunks
  - Embedder: Generates vector embeddings using Cohere
  - Storage Handler: Stores embeddings in Qdrant with metadata

- **Retrieval Validation (retrieve.py)**:
  - Qdrant Connector: Connects to Qdrant and loads vector collections
  - Query Processor: Converts text queries to embeddings using Cohere
  - Similarity Search: Performs top-k similarity search in Qdrant
  - Result Validator: Validates retrieved results against source content

## Configuration

The pipeline can be configured via environment variables:

- `DOCUSAURUS_URL`: The Docusaurus site to crawl (required)
- `COHERE_API_KEY`: Your Cohere API key (required)
- `QDRANT_URL`: Your Qdrant Cloud cluster URL (required)
- `QDRANT_API_KEY`: Your Qdrant API key (required)
- `CHUNK_SIZE`: Size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50)
- `BATCH_SIZE`: Number of chunks per API call (default: 10)