# Quickstart Guide: RAG Retrieval Pipeline Validation

## Prerequisites

1. **Python 3.11+**: Ensure you have Python 3.11 or higher installed
2. **Existing Vectors**: Vectors must already be stored in Qdrant from previous ingestion (Spec-1)
3. **Cohere API key**: Get an API key from [Cohere Dashboard](https://dashboard.cohere.com/)
4. **Qdrant Cloud account**: Access to the same Qdrant Cloud cluster used in Spec-1

## Setup

### 1. Navigate to Backend Directory
```bash
cd backend/
```

### 2. Install Dependencies
```bash
# Install project dependencies (if not already installed)
pip install -r requirements.txt
```

### 3. Create Environment File
Ensure your `.env` file contains the following:

```
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Running the Retrieval Validation

### 1. Basic Execution
```bash
python retrieve.py
```

### 2. Configuration Options
The validation can be configured with the following environment variables:

- `COHERE_API_KEY`: Your Cohere API key (required)
- `QDRANT_URL`: Your Qdrant Cloud cluster URL (required)
- `QDRANT_API_KEY`: Your Qdrant API key (required)
- `K_VALUE`: Number of results to retrieve (default: 5)

### 3. Command Line Options
The script also accepts command line arguments:

```bash
# Run with a specific query
python retrieve.py --query "your query text here"

# Run with custom k-value
python retrieve.py --k 10

# Run with both query and k-value
python retrieve.py --query "your query" --k 5
```

## Architecture Overview

The validation pipeline consists of the following components:

1. **Qdrant Connector**: Connects to Qdrant and verifies stored vectors exist
2. **Query Processor**: Converts text queries to embeddings using Cohere
3. **Similarity Search**: Performs top-k search in Qdrant for relevant chunks
4. **Result Validator**: Validates retrieved content against source metadata
5. **Validation Pipeline**: Orchestrates the entire validation process

## Testing the Validation

The script will execute a test query and validate the results:

```bash
python retrieve.py --query "test query for validation"
```

This will:
1. Connect to Qdrant and verify the collection exists
2. Convert the query to an embedding using Cohere
3. Perform similarity search to find relevant chunks
4. Validate the retrieved content against source URLs and metadata
5. Report validation results with accuracy metrics

## Troubleshooting

### Common Issues

1. **Connection Issues**: Ensure your Qdrant URL and API key are correct, and that your Cohere API key is valid.

2. **No Results**: If no vectors are found, verify that you have successfully run the ingestion pipeline from Spec-1 and vectors are stored in Qdrant.

3. **Dimension Mismatch**: Ensure the Cohere model used for queries matches the one used for stored embeddings.

### Environment Variables

Make sure all required environment variables are properly set before running the validation. Missing or incorrect values will cause the validation to fail.

## Next Steps

Once the validation runs successfully:
1. Review the validation results and accuracy metrics
2. Test with various queries to ensure retrieval quality
3. Adjust k-value based on your requirements
4. Monitor API usage for Cohere and Qdrant to stay within limits