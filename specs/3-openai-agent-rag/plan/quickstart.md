# Quickstart: OpenAI Agent with RAG

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

3. **Install Dependencies**:
   ```bash
   pip install openai
   ```

## Basic Usage

1. **Initialize the Agent**:
   ```python
   from backend.agent import RAGAgent

   # The agent will automatically load environment variables
   agent = RAGAgent()
   ```

2. **Ask Questions**:
   ```python
   # Simple query
   response = agent.query("What is the main topic of this documentation?")
   print(response.answer)
   print(f"Sources: {response.sources}")
   print(f"Confidence: {response.confidence}")
   ```

3. **Multi-turn Conversations**:
   ```python
   # Start a conversation thread
   thread_id = agent.start_conversation()

   # First question
   response1 = agent.query("What is ROS 2?", thread_id=thread_id)

   # Follow-up question (context will be maintained)
   response2 = agent.query("How does it differ from ROS 1?", thread_id=thread_id)
   ```

## Configuration

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

## Error Handling

The agent handles common errors gracefully:

- **API Limit Errors**: Automatic retry with exponential backoff
- **Qdrant Connection Errors**: Graceful degradation with error message
- **No Results Found**: Informs user when no relevant content is found