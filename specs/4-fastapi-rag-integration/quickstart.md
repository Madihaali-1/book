# Quickstart: FastAPI RAG Integration

## Prerequisites

- Python 3.8 or higher
- pip package manager
- Access to API keys (OpenRouter, Cohere, Qdrant)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install fastapi uvicorn python-dotenv pydantic
```

### 4. Configure Environment Variables
Create a `.env` file in the backend directory with your API keys:

```env
OPENROUTER_API_KEY=your_openrouter_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
```

### 5. Verify Backend Components
Ensure the following files exist in the backend directory:
- `agent.py` - Contains the RAGAgent implementation
- `agent_function_tool.py` - Contains the retrieval function tool
- `api.py` - FastAPI server implementation (will be created)

## Running the API Server

### 1. Start the Server
```bash
cd backend
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

### 2. Verify Server is Running
Open your browser and navigate to:
- API Documentation: http://localhost:8000/docs
- Health Check: http://localhost:8000/health

## Using the API

### Query Endpoint
Send a POST request to `/query` with a JSON payload:

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is this book about?",
    "session_id": "optional_session_id"
  }'
```

### Expected Response
```json
{
  "answer": "This book is about Physical AI and Humanoid Robotics...",
  "sources": [],
  "confidence": 0.8,
  "session_id": "session_20260103_103000",
  "timestamp": "2026-01-03T10:30:00.123456"
}
```

## Frontend Integration

### JavaScript Example
```javascript
async function queryRAGAgent(query, sessionId = null) {
  const response = await fetch('http://localhost:8000/query', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      query: query,
      session_id: sessionId
    })
  });

  if (!response.ok) {
    throw new Error(`API request failed: ${response.status}`);
  }

  return await response.json();
}

// Example usage
const result = await queryRAGAgent("What is this book about?");
console.log(result.answer);
```

## Troubleshooting

### Common Issues

1. **API Keys Not Loaded**
   - Ensure `.env` file is in the correct directory
   - Verify environment variables are set correctly
   - Check that python-dotenv is installed

2. **RAG Agent Not Initializing**
   - Verify all required API keys are provided
   - Check that agent.py and agent_function_tool.py are accessible
   - Confirm Qdrant connection details are correct

3. **CORS Issues in Frontend**
   - Add CORS middleware to FastAPI if needed:
   ```python
   from fastapi.middleware.cors import CORSMiddleware

   app.add_middleware(
       CORSMiddleware,
       allow_origins=["*"],  # Configure appropriately for production
       allow_credentials=True,
       allow_methods=["*"],
       allow_headers=["*"],
   )
   ```

## Next Steps

1. Integrate the API into your Docusaurus frontend
2. Implement frontend chatbot UI components
3. Add error handling and loading states in the frontend
4. Test the complete end-to-end flow