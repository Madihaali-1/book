# OpenAI Agent with RAG Capabilities

This module implements an OpenAI Agent that integrates with the existing Qdrant-based retrieval pipeline to provide RAG (Retrieval-Augmented Generation) capabilities for book content. The agent uses the OpenAI Assistants API and leverages the existing retrieval logic from the backend/retrieve.py module.

## Features

- **Retrieval-Augmented Generation**: Answers questions using information retrieved from stored book content
- **OpenAI Integration**: Uses OpenAI Assistants API with custom retrieval tools
- **Qdrant Integration**: Queries Qdrant vector database for relevant content
- **Conversation Context**: Maintains context across multi-turn conversations
- **Source Attribution**: Provides sources for all information used in responses
- **Confidence Scoring**: Returns confidence scores based on similarity scores

## Prerequisites

Before using the agent, ensure you have the following environment variables configured:

```bash
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
```

## Installation

The agent is part of the backend module and requires the dependencies in `backend/requirements.txt`.

## Usage

### Basic Usage

```python
from backend.agent import RAGAgent

# Initialize the agent
agent = RAGAgent()

# Start a conversation
thread_id = agent.start_conversation()

# Ask a question
response = agent.query("What is the main topic of this documentation?", thread_id=thread_id)

print(f"Answer: {response.answer}")
print(f"Sources: {response.sources}")
print(f"Confidence: {response.confidence}")
```

### Configuration

The agent can be configured with the following parameters:

```python
agent = RAGAgent(
    model="gpt-3.5-turbo",      # OpenAI model to use
    retrieval_k=5,              # Number of chunks to retrieve
    temperature=0.3             # Response randomness
)
```

### Multi-turn Conversations

```python
# Continue conversation using the same thread
followup_response = agent.query("Can you elaborate on that?", thread_id=thread_id)
```

## Architecture

The agent follows this workflow:

1. User submits a query
2. Agent calls the `retrieve_content` tool to search Qdrant for relevant content
3. Retrieved content is passed to the OpenAI assistant
4. Assistant generates a response based on the retrieved content
5. Response is formatted with sources and confidence score

## Error Handling

- If no relevant content is found, the agent indicates this appropriately
- API rate limits are handled with exponential backoff
- Connection errors to Qdrant or Cohere are caught and reported

## Validation

The agent ensures all responses are grounded in retrieved content only, preventing hallucinations.