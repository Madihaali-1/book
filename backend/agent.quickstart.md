# Quickstart Guide: OpenAI Agent with RAG

## Overview

This guide will help you get started quickly with the OpenAI Agent that provides Retrieval-Augmented Generation (RAG) capabilities for book content.

## Prerequisites

1. **API Keys**:
   - OpenAI API Key (https://platform.openai.com/api-keys)
   - Qdrant Cloud URL and API Key
   - Cohere API Key (for embeddings)

2. **Environment Setup**:
   ```bash
   # Create/update .env file in project root
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   COHERE_API_KEY=your_cohere_api_key_here
   ```

3. **Install Dependencies** (already included in backend/requirements.txt):
   ```bash
   pip install -r backend/requirements.txt
   ```

## Basic Usage

### 1. Initialize the Agent

```python
from backend.agent import RAGAgent

# The agent will automatically load environment variables
agent = RAGAgent()
```

### 2. Ask Questions

```python
# Simple query
response = agent.query("What is the main topic of this documentation?")
print(response.answer)
print(f"Sources: {response.sources}")
print(f"Confidence: {response.confidence}")
```

### 3. Multi-turn Conversations

```python
# Start a conversation thread
thread_id = agent.start_conversation()

# First question
response1 = agent.query("What is ROS 2?", thread_id=thread_id)

# Follow-up question (context will be maintained)
response2 = agent.query("How does it differ from ROS 1?", thread_id=thread_id)
```

## Configuration Options

The agent can be configured with the following parameters:

- `model`: OpenAI model to use (default: "gpt-3.5-turbo")
- `retrieval_k`: Number of chunks to retrieve (default: 5)
- `temperature`: Response randomness (default: 0.3)

Example:
```python
agent = RAGAgent(
    model="gpt-4-turbo",
    retrieval_k=7,
    temperature=0.1
)
```

## Expected Response Format

The agent returns a named tuple with:
- `answer`: The agent's response to the query
- `sources`: List of URLs for retrieved content used in the response
- `confidence`: Confidence score based on similarity of retrieved content (0.0-1.0)

## Troubleshooting

### Common Issues:

1. **API Key Errors**: Verify all environment variables are set correctly
2. **Rate Limits**: The system includes retry logic for API rate limits
3. **No Results**: If no relevant content is found, the agent will indicate this

### Testing the Agent:

```python
# Test basic functionality
from backend.agent import RAGAgent

try:
    agent = RAGAgent()
    thread_id = agent.start_conversation()
    response = agent.query("What is the main topic of this documentation?")
    print("Success! Response:", response.answer[:100] + "...")
except Exception as e:
    print(f"Error: {e}")
```

## Next Steps

1. Integrate the agent into your application
2. Customize the system prompt for your specific use case
3. Experiment with different OpenAI models for different performance/quality trade-offs
4. Monitor response times and confidence scores to optimize performance