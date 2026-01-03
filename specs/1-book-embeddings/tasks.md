# Tasks: Book URL Embeddings for RAG

**Feature**: Book URL Embeddings for RAG
**Branch**: 1-book-embeddings
**Created**: 2026-01-02
**Status**: Draft

## Implementation Strategy

This implementation follows a phased approach delivering an MVP with core functionality first, then enhancing with additional features. The MVP will focus on User Story 1 (Docusaurus Documentation Ingestion) which provides the core value proposition of crawling, processing, and storing embeddings.

## Dependencies

User Story 2 (Text Chunking and Embedding) and User Story 3 (Vector Storage and Indexing) depend on User Story 1 (Docusaurus Documentation Ingestion) for foundational components like URL fetching and text cleaning.

## Parallel Execution Examples

- [P] Tasks T002-T005 (dependencies) can run in parallel with T006 (main.py skeleton)
- [P] Tasks T010-T015 (component implementations) can run in parallel after foundational setup
- [P] Tasks T020-T025 (testing components) can run in parallel after implementations

## Phase 1: Setup

**Goal**: Initialize project structure and dependencies

- [X] T001 Create backend directory structure
- [X] T002 Add dependencies to pyproject.toml (requests, beautifulsoup4, cohere, qdrant-client, python-dotenv, dataclasses)
- [X] T003 Create .env file template with required environment variables
- [X] T004 Create README.md with setup instructions
- [X] T005 Create requirements.txt as backup dependency file
- [X] T006 Create main.py skeleton with imports and basic structure

## Phase 2: Foundational Components

**Goal**: Implement foundational data structures and configuration management

- [X] T007 Implement DocumentChunk dataclass in backend/main.py
- [X] T008 Implement CrawledPage dataclass in backend/main.py
- [X] T009 Implement configuration loading with environment variables in backend/main.py

## Phase 3: [US1] Docusaurus Documentation Ingestion

**Goal**: Implement core functionality to crawl Docusaurus sites and extract content

**Independent Test Criteria**:
- Given a valid Docusaurus website URL, when I run the ingestion script, then all public pages are crawled and their text content is extracted and cleaned

- [X] T010 [P] [US1] Implement URLFetcher class with fetch_docusaurus_pages method in backend/main.py
- [X] T011 [P] [US1] Implement extract_content method for HTML parsing and cleaning in backend/main.py
- [X] T012 [P] [US1] Implement _extract_title helper method in backend/main.py
- [X] T013 [P] [US1] Add error handling for URL fetching in backend/main.py
- [X] T014 [US1] Test URL fetching functionality with sample Docusaurus site
- [X] T015 [US1] Verify text extraction removes navigation elements correctly

## Phase 4: [US2] Text Chunking and Embedding

**Goal**: Implement text segmentation and embedding generation

**Independent Test Criteria**:
- Given a long text document, when the chunking process runs, then it's divided into segments of appropriate size (e.g., 512 tokens) without breaking context

- [X] T016 [P] [US2] Implement TextChunker class with chunk_text method in backend/main.py
- [X] T017 [P] [US2] Implement _split_into_sentences helper method in backend/main.py
- [X] T018 [P] [US2] Implement _get_overlap_sentences helper method in backend/main.py
- [X] T019 [P] [US2] Implement Embedder class with generate_embeddings method in backend/main.py
- [X] T020 [US2] Test text chunking with various document sizes
- [X] T021 [US2] Verify embeddings are generated successfully with Cohere API

## Phase 5: [US3] Vector Storage and Indexing

**Goal**: Implement storage and retrieval of embeddings in Qdrant

**Independent Test Criteria**:
- Given generated embeddings, when the storage process runs, then they are indexed in Qdrant with proper metadata and accessible via the API

- [X] T022 [P] [US3] Implement StorageHandler class with store_embeddings method in backend/main.py
- [X] T023 [P] [US3] Implement _ensure_collection_exists method in backend/main.py
- [X] T024 [P] [US3] Implement test_similarity_search method in backend/main.py
- [X] T025 [US3] Test storing embeddings in Qdrant Cloud
- [X] T026 [US3] Verify similarity search returns relevant results

## Phase 6: Integration and Main Pipeline

**Goal**: Orchestrate the full ingestion pipeline end-to-end

- [X] T027 Implement main() function to orchestrate the full pipeline in backend/main.py
- [X] T028 Add command-line argument parsing for configuration options in backend/main.py
- [X] T029 Implement error handling and logging throughout the pipeline in backend/main.py
- [X] T030 Add retry mechanisms for API calls and network requests in backend/main.py
- [X] T031 Create test_pipeline function for validation in backend/main.py

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Add finishing touches and ensure quality

- [X] T032 Add comprehensive logging throughout the application
- [X] T033 Implement progress tracking for long-running operations
- [X] T034 Add validation for environment variables and configuration
- [X] T035 Write comprehensive docstrings for all functions and classes
- [ ] T036 Test end-to-end pipeline with a real Docusaurus site
- [X] T037 Update README with usage examples and troubleshooting
- [X] T038 Add error handling for edge cases identified in testing