# Implementation Plan: OpenAI Agent with RAG Capabilities

**Feature**: OpenAI Agent with RAG Capabilities
**Branch**: 3-openai-agent-rag
**Created**: 2026-01-02
**Status**: Draft

## Technical Context

This feature implements an OpenAI Agent that integrates with the existing Qdrant-based retrieval pipeline to provide RAG (Retrieval-Augmented Generation) capabilities for book content. The agent will use the OpenAI Agents SDK and leverage the existing retrieval logic from the backend/retrieve.py module.

**Key Technologies**:
- Python 3.11+
- OpenAI Agents SDK
- Qdrant vector database
- Existing retrieval pipeline (backend/retrieve.py)
- Cohere for embeddings (existing dependency)

**Integration Points**:
- backend/retrieve.py: Reuse SimilaritySearch and related classes
- Qdrant Cloud: Existing connection parameters
- Environment variables: QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, OPENAI_API_KEY

**Unknowns**: None (all resolved in research.md)

## Constitution Check

This implementation aligns with the project constitution:

✅ **Spec-First Workflow**: Implementation follows approved specification
✅ **Technical Accuracy and Source Integrity**: Agent will use only retrieved content
✅ **Developer-Focused Documentation**: Clear setup and usage instructions
✅ **Reproducible Setup and Deployment**: Python-based with requirements file
✅ **RAG System Integrity**: Agent will only respond using retrieved content
✅ **End-to-End Reproducibility**: Complete agent implementation with setup guide

## Gates

**GATE 1: Technical Feasibility** - Verify OpenAI Agents SDK compatibility with existing retrieval pipeline
**GATE 2: Security Compliance** - Ensure API keys are properly handled and not exposed
**GATE 3: Performance Requirements** - Verify response times meet success criteria (5 seconds)
**GATE 4: Quality Assurance** - Agent responses must be grounded in retrieved content only

## Phase 0: Outline & Research

### Research Tasks

1. **OpenAI Agents SDK Integration**: Research how to create custom tools that integrate with existing retrieval pipeline
2. **API Key Management**: Determine proper handling of OPENAI_API_KEY alongside existing keys
3. **Conversation Context Management**: Research how OpenAI Agents handle follow-up queries
4. **Model Selection**: Identify appropriate OpenAI models for RAG applications

### Dependencies

- OpenAI Python library
- Existing qdrant-client and cohere dependencies
- python-dotenv for environment management

## Phase 1: Design & Architecture

### Data Model

**AgentRequest**
- query: str - User's question/query
- context: List[RetrievedChunk] - Retrieved content from Qdrant
- conversation_history: List[Message] - Previous conversation turns

**AgentResponse**
- answer: str - Agent's response based on retrieved content
- sources: List[str] - URLs of retrieved chunks
- confidence: float - Confidence score based on similarity scores

### Tool Interface Contract

**OpenAI Agent Tool Function**
- Function Name: "retrieve_content"
- Parameters: {"query": str, "k": int}
- Returns: JSON object containing list of retrieved chunks with content, source, and similarity scores
- Purpose: Allow the OpenAI agent to retrieve relevant content from Qdrant when answering questions

### Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   User Query    │───▶│  OpenAI Agent   │───▶│  Agent Response │
└─────────────────┘    │                 │    └─────────────────┘
                       │ ┌─────────────┐ │
                       │ │Retrieval    │ │
                       │ │Tool         │ │
                       │ └─────────────┘ │
                       └─────────────────┘
                              │
                              ▼
                       ┌──────────────────┐
                       │  Qdrant Vector  │
                       │   Database      │
                       └─────────────────┘
```

## Phase 2: Implementation Plan

### Task 1: Setup and Dependencies
- Add OpenAI to requirements.txt
- Create agent.py in backend directory (backend/agent.py)
- Set up proper environment variable handling

### Task 2: Retrieval Tool Creation
- Create a wrapper around existing retrieval logic
- Implement OpenAI-compatible tool interface
- Ensure proper error handling

### Task 3: Agent Configuration
- Initialize OpenAI agent with retrieval tool
- Configure system prompt to use only retrieved content
- Implement follow-up query handling

### Task 4: Testing and Validation
- Test basic retrieval functionality
- Validate response grounding in retrieved content
- Verify follow-up query handling

## Phase 3: Integration & Deployment

### Integration Testing
- End-to-end testing with sample queries
- Performance validation (response time < 5 seconds)
- Accuracy validation against human expert responses

### Documentation
- Quickstart guide for using the agent
- Configuration instructions
- Troubleshooting guide

## Risks & Mitigation

**Risk 1**: OpenAI API rate limits affecting performance
*Mitigation*: Implement proper retry logic and caching

**Risk 2**: Qdrant unavailability causing agent failures
*Mitigation*: Implement graceful degradation and error handling

**Risk 3**: Agent generating responses not grounded in retrieved content
*Mitigation*: Implement strict system prompts and content validation

## Success Criteria Validation

- [ ] Agent answers questions with 80% accuracy compared to human experts
- [ ] Retrieval completes within 5 seconds for 95% of queries
- [ ] Follow-up queries handled in 90% of multi-turn conversations
- [ ] Setup time under 5 minutes as specified