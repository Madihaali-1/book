# Quickstart Guide: URL Ingestion & Embedding Pipeline

## Prerequisites

1. **Python 3.11+**: Ensure you have Python 3.11 or higher installed
2. **uv package manager**: Install uv for dependency management
3. **Cohere API key**: Get an API key from [Cohere Dashboard](https://dashboard.cohere.com/)
4. **Qdrant Cloud account**: Create a free account at [Qdrant Cloud](https://cloud.qdrant.io/)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to Backend Directory
```bash
cd backend/
```

### 3. Install Dependencies with uv
```bash
# Install uv if you don't have it
pip install uv

# Install project dependencies
uv sync
```

### 4. Create Environment File
Create a `.env` file in the backend directory with the following content:

```
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
DOCUSAURUS_URL=https://your-docusaurus-site.com
```

## Running the Ingestion Pipeline

### 1. Basic Execution
```bash
python main.py
```

### 2. Configuration Options
The pipeline can be configured with the following environment variables:

- `COHERE_API_KEY`: Your Cohere API key (required)
- `QDRANT_URL`: Your Qdrant Cloud cluster URL (required)
- `QDRANT_API_KEY`: Your Qdrant API key (required)
- `DOCUSAURUS_URL`: The Docusaurus site to crawl (required)
- `CHUNK_SIZE`: Size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50)
- `BATCH_SIZE`: Number of chunks to process per API call (default: 10)

## Architecture Overview

The pipeline consists of the following components:

1. **URL Fetcher**: Crawls the Docusaurus site and extracts all public pages
2. **Text Cleaner**: Parses HTML and extracts relevant content while removing navigation
3. **Chunker**: Splits long documents into appropriately sized chunks
4. **Embedder**: Generates vector embeddings using Cohere models
5. **Storage Handler**: Stores embeddings in Qdrant with metadata
6. **Main Pipeline**: Orchestrates the entire process end-to-end

## Testing the Pipeline

After running the pipeline, you can verify that embeddings were stored correctly by running:

```bash
python -c "from main import test_pipeline; test_pipeline()"
```

This will perform a test similarity search to verify the stored embeddings work correctly.

## Troubleshooting

### Common Issues

1. **API Rate Limits**: If you encounter rate limit errors, the pipeline has built-in retry mechanisms with exponential backoff.

2. **Connection Issues**: Ensure your Qdrant URL and API key are correct, and that your Cohere API key is valid.

3. **Crawling Problems**: If the crawler can't access certain pages, check if they're behind authentication or have robots.txt restrictions.

### Environment Variables

Make sure all required environment variables are properly set before running the pipeline. Missing or incorrect values will cause the pipeline to fail.

## Next Steps

Once the pipeline runs successfully:
1. Verify the embeddings are stored in your Qdrant collection
2. Test the similarity search functionality
3. Adjust chunk size and overlap parameters based on your content
4. Monitor API usage for Cohere and Qdrant to stay within free tier limits