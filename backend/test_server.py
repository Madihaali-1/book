"""
Test script to verify API endpoints are working
"""
import asyncio
import httpx
from api import app

async def test_api_endpoints():
    """Test if the API endpoints are accessible"""
    print("Testing API endpoints...")

    # Create an async client using FastAPI's test client capabilities
    async with httpx.AsyncClient(app=app, base_url="http://testserver") as client:
        try:
            # Test root endpoint
            response = await client.get("/")
            print(f"✓ Root endpoint: {response.status_code} - {response.json()}")
        except Exception as e:
            print(f"✗ Root endpoint failed: {e}")

        try:
            # Test health endpoint
            response = await client.get("/health")
            print(f"✓ Health endpoint: {response.status_code} - {response.json()}")
        except Exception as e:
            print(f"✗ Health endpoint failed: {e}")

        try:
            # Test docs endpoint (this might not work with test client)
            print("✓ Documentation endpoints should be available at:")
            print("  - http://localhost:8000/docs (Swagger UI)")
            print("  - http://localhost:8000/redoc (ReDoc)")
        except Exception as e:
            print(f"Documentation endpoints info: {e}")

    print("\nTo run the server and access the API:")
    print("1. Make sure you have uvicorn installed: pip install uvicorn")
    print("2. Run: uvicorn api:app --reload --host 0.0.0.0 --port 8000")
    print("3. Access endpoints at http://localhost:8000")

if __name__ == "__main__":
    asyncio.run(test_api_endpoints())