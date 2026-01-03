"""
OpenAI Agent with RAG Capabilities
"""

# =========================
# ENV MUST BE FIRST
# =========================
import os

# Load environment variables from .env file
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass  # dotenv not available, will rely on system environment

os.environ["OPENAI_TRACING"] = "false"
os.environ["OPENAI_AGENTS_DISABLE_TRACING"] = "1"
os.environ["OPENAI_API_KEY"] = "sk-dummy"  # tracing ke liye required, real use nahi hogi

# =========================
# IMPORTS
# =========================
import logging
from typing import Optional
from datetime import datetime

from agents import Agent, Runner, SQLiteSession
from agents import OpenAIChatCompletionsModel
from agent_function_tool import retrieve_content
from openai import AsyncOpenAI


# =========================
# OpenRouter Configuration
# =========================

ROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")

if not ROUTER_API_KEY:
    raise ValueError("OPENROUTER_API_KEY environment variable is required")

client = AsyncOpenAI(
    api_key=ROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1",
    default_headers={
        "HTTP-Referer": "http://localhost",
        "X-Title": "Book RAG Agent"
    }
)

model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="mistralai/devstral-2512:free"
)

# =========================
# Logging
# =========================

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# =========================
# RAG Agent
# =========================

class RAGAgent:

    def __init__(self):
        self.agent = Agent(
            name="Book Content RAG Assistant",
            instructions=(
                "Always use retrieve_content before answering. "
                "Answer strictly from retrieved content only."
            ),
            model=model,
            tools=[retrieve_content],
        )

        logger.info("RAG Agent initialized")

    async def query(self, question: str, session_id: Optional[str] = None):

        if not session_id:
            session_id = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        session = SQLiteSession(session_id=session_id, db_path="conversations.db")

        result = await Runner.run(
            self.agent,
            question,
            session=session
        )

        return result.final_output


# =========================
# RUNNER
# =========================

async def main():
    agent = RAGAgent()
    answer = await agent.query("What is book about?")
    print("\nANSWER:\n", answer)


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
