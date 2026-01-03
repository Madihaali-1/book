# Implementation Plan: URL Ingestion & Embedding Pipeline

**Feature**: Book URL Embeddings for RAG
**Branch**: 1-book-embeddings
**Created**: 2026-01-02
**Status**: Draft
**Input**: Spec-1: URL Ingestion & Embedding Pipeline

- In 'backend/ folder, initialize project with 'uv', and add a single 'main.py'
- In 'main.py', implement URL fetching, text cleaning, and chunking
- Generate embeddings using Cohere models
- Store embeddings and metadata in Qdrant Cloud
- Add a 'main() function to run the full ingestion pipeline end-to-end

## Technical Context

### Architecture Overview
- **Backend Framework**: Python with modular script structure
- **Project Management**: uv for dependency management
- **Web Scraping**: BeautifulSoup/lxml for HTML parsing and cleaning
- **Text Processing**: Custom chunking algorithm for document segmentation
- **Embeddings**: Cohere API for vector generation
- **Vector Storage**: Qdrant Cloud for storing and indexing embeddings
- **Configuration**: Environment variables for API keys and service endpoints

### Components
- **URL Fetcher**: Module to crawl Docusaurus websites and extract page content
- **Text Cleaner**: Module to extract relevant text and remove navigation elements
- **Chunker**: Module to segment text into appropriate sizes for embeddings
- **Embedder**: Module to generate vector embeddings using Cohere
- **Storage Handler**: Module to store embeddings in Qdrant with metadata
- **Main Pipeline**: Orchestrator that runs the full ingestion process end-to-end

### Technology Stack
- **Language**: Python 3.11+
- **Package Manager**: uv
- **Web Scraping**: requests, BeautifulSoup4
- **Text Processing**: nltk or custom implementation
- **Embeddings**: cohere Python package
- **Vector DB**: qdrant-client
- **Configuration**: python-dotenv for environment management

## Constitution Check

### Spec-First Workflow
✅ All development follows Spec-Kit Plus methodology: Implementation will follow this plan with clear acceptance criteria documented before implementation begins.

### Technical Accuracy and Source Integrity
✅ All content will be grounded in official documentation for Cohere API, Qdrant, and Docusaurus structure.

### Developer-Focused Documentation
✅ Code will include clear documentation, examples, and setup instructions.

### Reproducible Setup and Deployment
✅ All dependencies will be explicitly declared and the environment will be reproducible from documentation alone.

### RAG System Integrity
✅ Embeddings will be generated only from actual document content, ensuring grounding in source material.

### End-to-End Reproducibility
✅ Complete setup instructions will be provided with versioned dependencies.

## Gates

### Gate 1: Architecture Alignment
- [x] Architecture aligns with functional requirements (FR-001 through FR-010)
- [x] Technology choices support success criteria (SC-001 through SC-006)
- [x] No architectural conflicts with existing system

### Gate 2: Technical Feasibility
- [x] All required technologies have stable APIs and good documentation
- [x] Architecture supports performance requirements
- [x] Error handling and monitoring capabilities included

### Gate 3: Security & Compliance
- [x] API keys managed through environment variables (FR-007)
- [x] No hardcoded secrets in source code
- [x] Proper error handling to prevent information disclosure

## Phase 0: Research & Unknowns Resolution

### Research Findings

#### 1. Docusaurus Site Structure Analysis
- **Decision**: Use requests + BeautifulSoup for crawling Docusaurus sites
- **Rationale**: Docusaurus sites follow predictable patterns with static HTML content that can be parsed effectively
- **Alternatives considered**: Selenium for dynamic content vs. requests for static content; requests chosen for simplicity and performance

#### 2. Text Chunking Strategy
- **Decision**: Implement recursive text splitting with overlap
- **Rationale**: Preserves context while ensuring chunks fit within embedding model limits (typically 512 tokens)
- **Alternatives considered**: Fixed-length splitting vs. semantic-aware chunking; recursive approach chosen for better context preservation

#### 3. Cohere Embedding Model Selection
- **Decision**: Use Cohere's embed-multilingual-v3.0 model
- **Rationale**: Good performance for documentation content, supports multiple languages, and has high throughput
- **Alternatives considered**: Various Cohere models and OpenAI embeddings; Cohere multilingual chosen for documentation use case

#### 4. Qdrant Collection Schema Design
- **Decision**: Store embeddings with URL, chunk content, and metadata in structured format
- **Rationale**: Enables efficient retrieval and maintains connection to source documents
- **Alternatives considered**: Different metadata schemas; minimal viable schema chosen for initial implementation

#### 5. Error Handling Strategy
- **Decision**: Implement comprehensive error handling with retry mechanisms
- **Rationale**: External APIs (Cohere, web requests) can be unreliable; robust error handling is essential
- **Alternatives considered**: Basic error handling vs. comprehensive retry/backoff; comprehensive approach chosen for reliability

## Phase 1: Design & Contracts

### Data Model

#### Document Chunk Entity
- **chunk_id**: Unique identifier for the chunk (UUID string)
- **url**: Source URL of the original document (string)
- **content**: The actual text content of the chunk (string)
- **embedding**: Vector representation of the content (array of floats)
- **metadata**: Additional information like page title, section, etc. (JSON object)
- **created_at**: Timestamp of when the chunk was created (datetime)

#### Crawled Page Entity
- **page_id**: Unique identifier for the page (UUID string)
- **url**: The URL of the page (string)
- **title**: Title of the page (string)
- **content**: Raw HTML content before cleaning (string)
- **processed_content**: Cleaned text content (string)
- **status**: Status of crawling (string: success, error, etc.)
- **fetched_at**: Timestamp of when the page was fetched (datetime)

### API Contracts

Since this is a backend script rather than a web service, the contracts are internal function interfaces:

#### URL Fetcher Interface
```python
def fetch_docusaurus_pages(base_url: str) -> List[CrawledPage]:
    """Fetch all public pages from a Docusaurus website"""
    pass

def extract_content(html_content: str) -> str:
    """Extract relevant text content from HTML, removing navigation elements"""
    pass
```

#### Chunker Interface
```python
def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """Split text into overlapping chunks of specified size"""
    pass
```

#### Embedder Interface
```python
def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """Generate embeddings for a list of text chunks"""
    pass
```

#### Storage Interface
```python
def store_embeddings(chunks: List[DocumentChunk]) -> bool:
    """Store document chunks with embeddings in Qdrant"""
    pass

def test_similarity_search(query: str) -> List[DocumentChunk]:
    """Test similarity search functionality"""
    pass
```

### Quickstart Guide

#### Prerequisites
1. Python 3.11+
2. uv package manager
3. Cohere API key
4. Qdrant Cloud cluster URL and API key

#### Setup
1. Clone the repository
2. Navigate to the backend directory
3. Install dependencies with `uv sync`
4. Create `.env` file with required API keys
5. Run the ingestion pipeline with `python main.py`

#### Environment Variables
```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
DOCUSAURUS_URL=the_docusaurus_site_to_crawl
```

## Phase 2: Implementation Plan

### Task Breakdown

1. **Setup Project Structure**
   - Create backend/ directory
   - Initialize with uv
   - Add dependencies (requests, beautifulsoup4, cohere, qdrant-client, python-dotenv)
   - Create main.py file

2. **Implement URL Fetcher**
   - Create function to crawl Docusaurus site
   - Extract all public page URLs
   - Fetch and store HTML content

3. **Implement Text Cleaner**
   - Parse HTML and extract relevant content
   - Remove navigation, headers, footers
   - Preserve document structure and meaning

4. **Implement Chunker**
   - Create recursive text splitting function
   - Ensure appropriate chunk sizes with overlap
   - Preserve context between chunks

5. **Implement Embedder**
   - Integrate with Cohere API
   - Generate embeddings for text chunks
   - Handle API rate limits and errors

6. **Implement Storage Handler**
   - Connect to Qdrant Cloud
   - Create collection schema
   - Store embeddings with metadata

7. **Create Main Pipeline**
   - Orchestrate the full ingestion process
   - Handle configuration and error management
   - Add test functionality

8. **Testing and Validation**
   - Verify all components work together
   - Test with sample Docusaurus site
   - Validate stored embeddings can be retrieved

## Re-evaluation of Constitution Check (Post-Design)

### Spec-First Workflow
✅ Confirmed: Implementation follows the specification with clear task breakdown.

### Technical Accuracy and Source Integrity
✅ Confirmed: Design uses official APIs and follows best practices.

### Developer-Focused Documentation
✅ Confirmed: Plan includes setup instructions and clear function interfaces.

### Reproducible Setup and Deployment
✅ Confirmed: Dependencies and environment setup clearly defined.

### RAG System Integrity
✅ Confirmed: Embeddings will be properly grounded in source content.

### End-to-End Reproducibility
✅ Confirmed: Complete workflow from crawling to storage is defined.

## Risk Analysis

### High Priority Risks
- API rate limits from Cohere affecting processing speed
- Large documentation sites exceeding Qdrant Cloud Free Tier limits
- Network issues during web crawling

### Mitigation Strategies
- Implement exponential backoff for API calls
- Add progress tracking and checkpointing
- Include retry mechanisms for network requests