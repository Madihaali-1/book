"""
URL Ingestion & Embedding Pipeline

This script implements a complete pipeline for:
1. Crawling Docusaurus websites
2. Extracting and cleaning text content
3. Chunking text into appropriate sizes
4. Generating embeddings using Cohere
5. Storing embeddings in Qdrant Cloud
"""
import os
import asyncio
import requests
from bs4 import BeautifulSoup
from typing import List, Dict, Any
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import time
import logging
from dataclasses import dataclass
import uuid
from datetime import datetime

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class DocumentChunk:
    """Represents a chunk of text with its embedding and metadata"""
    chunk_id: str
    url: str
    content: str
    embedding: List[float]
    metadata: Dict[str, Any]
    created_at: datetime


@dataclass
class CrawledPage:
    """Represents a page fetched from a Docusaurus website"""
    page_id: str
    url: str
    title: str
    content: str
    processed_content: str
    status: str
    fetched_at: datetime
    word_count: int = 0
    error_message: str = ""


class URLFetcher:
    """Module to crawl Docusaurus websites and extract page content"""

    def __init__(self):
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; DocusaurusCrawler/1.0)'
        })

    def fetch_docusaurus_pages(self, base_url: str) -> List[CrawledPage]:
        """Fetch all public pages from a Docusaurus website"""
        logger.info(f"Starting to crawl: {base_url}")

        # Check if the URL is a sitemap
        is_sitemap = 'sitemap' in base_url.lower()

        if is_sitemap:
            # Handle sitemap.xml file
            try:
                response = self.session.get(base_url, timeout=30)
                response.raise_for_status()

                # Parse XML sitemap
                from bs4 import BeautifulSoup
                soup = BeautifulSoup(response.content, 'xml')  # Use XML parser for sitemaps

                # Extract URLs from sitemap
                links = []
                for url_elem in soup.find_all('url'):
                    loc_elem = url_elem.find('loc')
                    if loc_elem:
                        url = loc_elem.text.strip()
                        # Only include URLs from the same domain
                        from urllib.parse import urlparse
                        base_domain = urlparse(base_url).netloc
                        link_domain = urlparse(url).netloc
                        if not link_domain or link_domain == base_domain:
                            links.append(url)

                logger.info(f"Found {len(links)} URLs in sitemap")

            except Exception as e:
                logger.error(f"Failed to parse sitemap: {e}")
                return []
        else:
            # Get the main page to extract links (original behavior)
            try:
                response = self.session.get(base_url, timeout=30)
                response.raise_for_status()
            except requests.RequestException as e:
                logger.error(f"Failed to fetch base URL: {e}")
                return []

            # Parse the main page to find other pages
            soup = BeautifulSoup(response.content, 'html.parser')

            # Find all internal links
            links = set()
            for link in soup.find_all('a', href=True):
                href = link['href']
                if href.startswith('/') or href.startswith(base_url):
                    if href.startswith('/'):
                        full_url = base_url.rstrip('/') + href
                    else:
                        full_url = href
                    # Only add valid URLs that are part of the same domain
                    from urllib.parse import urlparse
                    base_domain = urlparse(base_url).netloc
                    link_domain = urlparse(full_url).netloc
                    if not link_domain or link_domain == base_domain:  # Same domain or relative URL
                        links.add(full_url)

            # Limit to 50 links to avoid overloading (adjust as needed)
            links = list(links)[:50]

        crawled_pages = []
        for url in links:
            try:
                logger.info(f"Fetching: {url}")
                page_response = self.session.get(url, timeout=30)
                page_response.raise_for_status()

                page = CrawledPage(
                    page_id=str(uuid.uuid4()),
                    url=url,
                    title=self._extract_title(page_response.content),
                    content=page_response.text,
                    processed_content=self.extract_content(page_response.content),
                    status="success",
                    fetched_at=datetime.now(),
                    word_count=len(self.extract_content(page_response.content).split())
                )
                crawled_pages.append(page)

            except requests.RequestException as e:
                logger.error(f"Failed to fetch {url}: {e}")
                page = CrawledPage(
                    page_id=str(uuid.uuid4()),
                    url=url,
                    title="",
                    content="",
                    processed_content="",
                    status="error",
                    fetched_at=datetime.now(),
                    error_message=str(e)
                )
                crawled_pages.append(page)

        logger.info(f"Successfully crawled {len(crawled_pages)} pages")
        return crawled_pages

    def _extract_title(self, html_content: str) -> str:
        """Extract title from HTML content"""
        soup = BeautifulSoup(html_content, 'html.parser')
        title_tag = soup.find('title')
        if title_tag:
            return title_tag.get_text().strip()
        return ""

    def extract_content(self, html_content: str) -> str:
        """Extract relevant text content from HTML, removing navigation elements"""
        soup = BeautifulSoup(html_content, 'html.parser')

        # Remove common navigation elements
        for tag in soup(['nav', 'header', 'footer', 'aside']):
            tag.decompose()

        # Remove script and style elements
        for tag in soup(['script', 'style']):
            tag.decompose()

        # Look for main content areas typical in Docusaurus sites
        content_selectors = [
            'main',
            '.markdown',
            '.container',
            '.main-wrapper',
            '.theme-doc-markdown'
        ]

        content = ""
        for selector in content_selectors:
            elements = soup.select(selector)
            if elements:
                for element in elements:
                    content += element.get_text(separator=' ', strip=True) + ' '
                break

        # If no specific content area found, get all text
        if not content.strip():
            content = soup.get_text(separator=' ', strip=True)

        # Clean up excessive whitespace
        content = ' '.join(content.split())

        return content


class TextChunker:
    """Module to segment text into appropriate sizes for embeddings"""

    def __init__(self, chunk_size: int = 512, overlap: int = 50):
        self.chunk_size = chunk_size
        self.overlap = overlap

    def chunk_text(self, text: str) -> List[str]:
        """Split text into overlapping chunks of specified size"""
        if not text:
            return []

        # Split text into sentences to preserve semantic boundaries
        sentences = self._split_into_sentences(text)

        chunks = []
        current_chunk = ""
        current_length = 0

        for sentence in sentences:
            sentence_length = len(sentence.split())

            # If adding this sentence would exceed chunk size
            if current_length + sentence_length > self.chunk_size:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())

                # Start new chunk with potential overlap
                if self.overlap > 0:
                    # Get the last few sentences from current chunk to create overlap
                    overlap_sentences = self._get_overlap_sentences(current_chunk, self.overlap)
                    current_chunk = overlap_sentences + " " + sentence
                else:
                    current_chunk = sentence
                current_length = len(current_chunk.split())
            else:
                current_chunk += " " + sentence
                current_length += sentence_length

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """Split text into sentences"""
        import re
        # Split on sentence boundaries (. ! ?) followed by whitespace and capital letter
        sentences = re.split(r'(?<=[.!?])\s+', text)
        return [s.strip() for s in sentences if s.strip()]

    def _get_overlap_sentences(self, chunk: str, overlap_size: int) -> str:
        """Get the last few sentences from a chunk to create overlap"""
        sentences = self._split_into_sentences(chunk)
        words = chunk.split()

        # Get the last overlap_size words
        if len(words) <= overlap_size:
            return chunk

        overlap_words = words[-overlap_size:]
        return ' '.join(overlap_words)


class Embedder:
    """Module to generate vector embeddings using Cohere"""

    def __init__(self):
        api_key = os.getenv('COHERE_API_KEY')
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.client = cohere.Client(api_key)
        self.model = "embed-multilingual-v3.0"  # Using multilingual model for documentation

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of text chunks"""
        if not texts:
            return []

        # Cohere has limits on batch sizes, so we'll process in chunks
        batch_size = 96  # Leave some room under the 96 limit
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            logger.info(f"Processing batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

            try:
                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type="search_document"  # Appropriate for document search
                )

                batch_embeddings = [embedding for embedding in response.embeddings]
                all_embeddings.extend(batch_embeddings)

                # Add a small delay to respect rate limits
                time.sleep(0.1)

            except Exception as e:
                logger.error(f"Error generating embeddings for batch: {e}")
                # Return zeros for failed embeddings to maintain alignment
                all_embeddings.extend([[0.0] * 1024 for _ in range(len(batch))])

        return all_embeddings


class StorageHandler:
    """Module to store embeddings in Qdrant with metadata"""

    def __init__(self):
        qdrant_url = os.getenv('QDRANT_URL')
        qdrant_api_key = os.getenv('QDRANT_API_KEY')

        if not qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")
        if not qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")

        self.client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False  # Using HTTP for simplicity
        )

        self.collection_name = "document_embeddings"
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Ensure the collection exists with proper configuration"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except:
            # Create collection if it doesn't exist
            logger.info(f"Creating collection '{self.collection_name}'")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1024,  # Cohere multilingual-v3.0 embeddings are 1024-dimensional
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Collection '{self.collection_name}' created successfully")

    def store_embeddings(self, chunks: List[DocumentChunk]) -> bool:
        """Store document chunks with embeddings in Qdrant"""
        if not chunks:
            logger.info("No chunks to store")
            return True

        logger.info(f"Storing {len(chunks)} chunks in Qdrant")

        try:
            points = []
            for chunk in chunks:
                points.append(
                    models.PointStruct(
                        id=chunk.chunk_id,
                        vector=chunk.embedding,
                        payload={
                            "url": chunk.url,
                            "content": chunk.content,
                            "title": chunk.metadata.get("title", ""),
                            "section": chunk.metadata.get("section", ""),
                            "position": chunk.metadata.get("position", 0),
                            "created_at": chunk.created_at.isoformat()
                        }
                    )
                )

            # Upload points in batches
            batch_size = 100
            for i in range(0, len(points), batch_size):
                batch = points[i:i + batch_size]
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=batch
                )
                logger.info(f"Uploaded batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1}")

            logger.info(f"Successfully stored {len(chunks)} chunks in Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error storing embeddings in Qdrant: {e}")
            return False

    def test_similarity_search(self, query: str) -> List[DocumentChunk]:
        """Test similarity search functionality"""
        try:
            embedder = Embedder()
            query_embedding = embedder.generate_embeddings([query])[0]

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=5
            )

            found_chunks = []
            for result in results:
                chunk = DocumentChunk(
                    chunk_id=result.id,
                    url=result.payload.get("url", ""),
                    content=result.payload.get("content", ""),
                    embedding=result.vector,
                    metadata={
                        "title": result.payload.get("title", ""),
                        "section": result.payload.get("section", ""),
                        "position": result.payload.get("position", 0),
                        "score": result.score
                    },
                    created_at=datetime.fromisoformat(result.payload.get("created_at", datetime.now().isoformat()))
                )
                found_chunks.append(chunk)

            logger.info(f"Found {len(found_chunks)} similar chunks for query: {query[:50]}...")
            return found_chunks

        except Exception as e:
            logger.error(f"Error in similarity search: {e}")
            return []


def main():
    """Main function to run the full ingestion pipeline end-to-end"""
    import argparse

    parser = argparse.ArgumentParser(description='Docusaurus URL Ingestion & Embedding Pipeline')
    parser.add_argument('--url', type=str, help='Docusaurus site URL to crawl (overrides DOCUSAURUS_URL env var)')
    parser.add_argument('--chunk-size', type=int, default=512, help='Size of text chunks (default: 512)')
    parser.add_argument('--chunk-overlap', type=int, default=50, help='Overlap between chunks (default: 50)')
    parser.add_argument('--batch-size', type=int, default=10, help='Number of chunks per API call (default: 10)')
    parser.add_argument('--dry-run', action='store_true', help='Run without storing embeddings')

    args = parser.parse_args()

    logger.info("Starting URL Ingestion & Embedding Pipeline")

    # Configuration from command line arguments (with environment variable fallback)
    docusaurus_url = args.url or os.getenv('DOCUSAURUS_URL')
    if not docusaurus_url:
        raise ValueError("DOCUSAURUS_URL environment variable or --url argument is required")

    chunk_size = args.chunk_size
    chunk_overlap = args.chunk_overlap
    batch_size = args.batch_size

    logger.info(f"Configuration: URL={docusaurus_url}, Chunk Size={chunk_size}, Overlap={chunk_overlap}")

    try:
        # Step 1: Fetch pages from Docusaurus site
        fetcher = URLFetcher()
        pages = fetcher.fetch_docusaurus_pages(docusaurus_url)

        # Filter out failed pages
        successful_pages = [p for p in pages if p.status == "success"]
        logger.info(f"Successfully fetched {len(successful_pages)} out of {len(pages)} pages")

        # Step 2: Process each page - clean and chunk
        chunker = TextChunker(chunk_size=chunk_size, overlap=chunk_overlap)
        all_chunks = []

        for idx, page in enumerate(successful_pages):
            logger.info(f"Processing page {idx + 1}/{len(successful_pages)}: {page.url}")

            # Chunk the content
            text_chunks = chunker.chunk_text(page.processed_content)

            # Create document chunks with embeddings
            for i, chunk_text in enumerate(text_chunks):
                chunk = DocumentChunk(
                    chunk_id=str(uuid.uuid4()),
                    url=page.url,
                    content=chunk_text,
                    embedding=[],  # Will be filled in the next step
                    metadata={
                        "title": page.title,
                        "section": f"chunk_{i}",
                        "position": i
                    },
                    created_at=datetime.now()
                )
                all_chunks.append(chunk)

            # Progress tracking
            if (idx + 1) % 10 == 0:  # Log progress every 10 pages
                logger.info(f"Progress: Processed {idx + 1}/{len(successful_pages)} pages, {len(all_chunks)} chunks created so far")

        logger.info(f"Created {len(all_chunks)} text chunks from {len(successful_pages)} pages")

        # Step 3: Generate embeddings for all chunks
        embedder = Embedder()
        chunk_texts = [chunk.content for chunk in all_chunks]
        logger.info(f"Generating embeddings for {len(chunk_texts)} text chunks...")

        # Progress tracking for embeddings
        embeddings = embedder.generate_embeddings(chunk_texts)

        # Update chunks with their embeddings
        for chunk, embedding in zip(all_chunks, embeddings):
            chunk.embedding = embedding

        logger.info("Embedding generation completed")

        # Step 4: Store embeddings in Qdrant
        if args.dry_run:
            logger.info(f"Dry run mode: Would store {len(all_chunks)} chunks in Qdrant")
            success = True  # Consider dry run as successful
        else:
            storage = StorageHandler()
            success = storage.store_embeddings(all_chunks)

            if success:
                logger.info("Pipeline completed successfully!")

                # Test the search functionality
                if all_chunks:
                    sample_query = all_chunks[0].content[:100] + "..."
                    logger.info(f"Testing search with sample: {sample_query}")
                    results = storage.test_similarity_search(all_chunks[0].content[:100])
                    logger.info(f"Search test returned {len(results)} results")
            else:
                logger.error("Pipeline failed during storage step")

    except requests.exceptions.ConnectionError as e:
        logger.error(f"Connection error during crawling: {e}")
        raise
    except requests.exceptions.Timeout as e:
        logger.error(f"Timeout error during crawling: {e}")
        raise
    except requests.exceptions.RequestException as e:
        logger.error(f"Request error during crawling: {e}")
        raise
    except ValueError as e:
        logger.error(f"Value error (possibly missing environment variables): {e}")
        raise
    except Exception as e:
        logger.error(f"Pipeline failed with unexpected error: {e}")
        raise


def test_pipeline():
    """Function to test the pipeline with a simple query"""
    try:
        storage = StorageHandler()
        results = storage.test_similarity_search("test query for validation")
        logger.info(f"Test search returned {len(results)} results")
        return len(results) > 0
    except Exception as e:
        logger.error(f"Test pipeline failed: {e}")
        return False


if __name__ == "__main__":
    main()