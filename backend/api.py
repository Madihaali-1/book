"""
FastAPI server for RAG Agent Integration

This module provides a REST API for the RAG agent, allowing frontend applications
to send queries and receive intelligent responses based on book content.
"""
import os
import logging
from typing import Optional, Dict, Any
from datetime import datetime
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from agent import RAGAgent
import asyncio

# Load environment variables
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for interacting with the RAG Agent for book content queries",
    version="1.0.0"
)

# Add CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

from pydantic import BaseModel, field_validator

class QueryRequest(BaseModel):
    """Request model for query endpoint"""
    query: str
    session_id: Optional[str] = None

    @field_validator('query')
    @classmethod
    def validate_query_length(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('Query cannot be empty')
        if len(v) > 1000:  # Limit query length to prevent abuse
            raise ValueError('Query is too long (maximum 1000 characters)')
        return v

class QueryResponse(BaseModel):
    """Response model for query endpoint"""
    answer: str
    sources: list = []
    confidence: float = 0.8
    session_id: str
    timestamp: str

class ErrorResponse(BaseModel):
    """Error response model"""
    error: str
    detail: Optional[str] = None

# Global agent instance
rag_agent: Optional[RAGAgent] = None

@app.on_event("startup")
async def startup_event():
    """Initialize the RAG agent when the application starts"""
    global rag_agent
    logger.info("Initializing RAG Agent...")
    try:
        rag_agent = RAGAgent()
        logger.info("RAG Agent initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAG Agent: {e}")
        raise

@app.get("/")
async def root():
    """Health check endpoint"""
    return {"status": "healthy", "message": "RAG Agent API is running"}

@app.post("/query",
          response_model=QueryResponse,
          responses={200: {"description": "Query processed successfully"},
                    400: {"model": ErrorResponse, "description": "Invalid request"}})
async def query_endpoint(request: QueryRequest):
    """
    Process a user query and return a response from the RAG agent.

    - **query**: The user's question or query about book content
    - **session_id**: Optional session ID to maintain conversation context
    """
    if not rag_agent:
        raise HTTPException(status_code=500, detail="RAG Agent not initialized")

    try:
        # If no session_id provided, generate one
        session_id = request.session_id
        if not session_id:
            session_id = f"api_session_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"

        start_time = datetime.now()
        logger.info(f"Processing query: '{request.query[:50]}...' for session {session_id}")

        # Process the query with the RAG agent
        result = await rag_agent.query(request.query, session_id=session_id)

        # Create response - result is a string from the agent
        response = QueryResponse(
            answer=result,
            sources=[],  # Sources would need to be extracted from tool calls in a full implementation
            confidence=0.8,  # Default confidence since we can't extract from current agent response
            session_id=session_id,
            timestamp=datetime.now().isoformat()
        )

        end_time = datetime.now()
        processing_time = (end_time - start_time).total_seconds()
        logger.info(f"Query processed successfully for session {session_id} in {processing_time:.2f}s")

        # Print the query and response in the terminal for easy debugging
        print(f"\n--- RAG AGENT QUERY ---")
        print(f"Query: {request.query}")
        print(f"Session ID: {session_id}")
        print(f"Answer: {result}")
        print(f"Processing Time: {processing_time:.2f}s")
        print(f"----------------------\n")

        return response

    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.get("/health")
async def health_check():
    """Detailed health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "rag_agent_initialized": rag_agent is not None
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)