import os
import psycopg2
import psycopg2.extras
from dotenv import load_dotenv

load_dotenv()

# --- Neon DB Connection ---
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")

async def get_db_connection():
    """Establishes a connection to the Neon Postgres database."""
    if not NEON_DATABASE_URL:
        raise ValueError("NEON_DATABASE_URL not found in environment variables.")
    try:
        conn = psycopg2.connect(NEON_DATABASE_URL)
        return conn
    except Exception as e:
        print(f"Error connecting to Neon database: {e}")
        return None
