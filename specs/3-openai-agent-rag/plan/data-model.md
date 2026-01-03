# Data Model: OpenAI Agent with RAG

## AgentRequest
- **query**: str
  - Description: User's question/query to the agent
  - Validation: Non-empty string, maximum 1000 characters
- **context**: List[RetrievedChunk]
  - Description: Retrieved content from Qdrant used to answer the query
  - Relationship: One-to-many with RetrievedChunk
- **conversation_history**: List[Message]
  - Description: Previous conversation turns for follow-up query context
  - Validation: Maximum 10 previous messages to maintain performance

## AgentResponse
- **answer**: str
  - Description: Agent's response based on retrieved content
  - Validation: Non-empty string, grounded in retrieved content only
- **sources**: List[str]
  - Description: URLs of retrieved chunks used to generate the answer
  - Validation: Each URL must be valid and correspond to retrieved content
- **confidence**: float
  - Description: Confidence score based on similarity scores of retrieved content
  - Range: 0.0 to 1.0
  - Validation: Must be calculated from similarity scores of retrieved chunks

## RetrievedChunk (from existing backend/retrieve.py)
- **chunk_id**: str
  - Description: Unique identifier for the text chunk
- **content**: str
  - Description: The actual text content of the chunk
- **similarity_score**: float
  - Description: Similarity score from vector search
  - Range: 0.0 to 1.0
- **source_url**: str
  - Description: URL where the original content can be found
- **metadata**: Dict[str, Any]
  - Description: Additional metadata about the chunk (title, section, etc.)

## Message (conversation history item)
- **role**: str
  - Description: Role of the message participant ('user' or 'assistant')
  - Validation: Must be either 'user' or 'assistant'
- **content**: str
  - Description: The text content of the message
  - Validation: Non-empty string
- **timestamp**: datetime
  - Description: When the message was created

## ToolParameters
- **query**: str
  - Description: Query string to pass to the retrieval tool
  - Validation: Non-empty string
- **k**: int
  - Description: Number of results to retrieve (default: 5)
  - Range: 1 to 10
  - Default: 5