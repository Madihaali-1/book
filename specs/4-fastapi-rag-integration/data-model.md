# Data Model: FastAPI RAG Integration

## Request Models

### QueryRequest
Represents a user query request containing the query text and optional session information

**Fields**:
- `query: str` - The user's question or query about book content (required, non-empty)
- `session_id: Optional[str]` - Session identifier to maintain conversation context (optional)

**Validation**:
- query must not be empty
- query length limited to reasonable size (e.g., 1000 characters) to prevent abuse
- session_id, if provided, must be a valid string format

## Response Models

### QueryResponse
Contains the RAG agent's response with answer, sources, confidence score, and metadata

**Fields**:
- `answer: str` - The agent's response to the user's query
- `sources: list` - List of sources used to generate the response (currently empty until enhanced)
- `confidence: float` - Confidence score for the response (0.0-1.0, default 0.8)
- `session_id: str` - Session identifier used for this query
- `timestamp: str` - ISO 8601 formatted timestamp of the response

### ErrorResponse
Represents error responses with details about what went wrong

**Fields**:
- `error: str` - Brief description of the error
- `detail: Optional[str]` - Additional details about the error (optional)

## Internal Data Structures

### Session
Represents a conversation context that maintains state between related queries

**Fields**:
- `session_id: str` - Unique identifier for the session
- `created_at: datetime` - When the session was created
- `last_activity: datetime` - When the session was last used
- `conversation_history: list` - List of query-response pairs in the session

## State Transitions

### Query Processing Flow
1. **Request Received**: QueryRequest validated by FastAPI
2. **Session Lookup/Create**: Session established (new or existing)
3. **Agent Query**: RAG agent processes the query with context
4. **Response Formation**: QueryResponse created with agent results
5. **Session Update**: Conversation history updated in session store
6. **Response Sent**: QueryResponse returned to client

### Error Handling Flow
1. **Validation Error**: Invalid QueryRequest triggers ErrorResponse with 400 status
2. **Processing Error**: Agent failure triggers ErrorResponse with 500 status
3. **Response Sent**: ErrorResponse returned to client with appropriate status code