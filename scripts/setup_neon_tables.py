import os
import psycopg2
from dotenv import load_dotenv

load_dotenv()

NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")

def create_tables():
    """Create the chat_history table in the Neon Postgres database."""
    if not NEON_DATABASE_URL:
        print("NEON_DATABASE_URL not found. Please set it in your .env file.")
        return

    conn = None
    try:
        conn = psycopg2.connect(NEON_DATABASE_URL)
        cur = conn.cursor()

        cur.execute("""
            CREATE TABLE IF NOT EXISTS chat_history (
                id SERIAL PRIMARY KEY,
                question TEXT NOT NULL,
                answer TEXT NOT NULL,
                sources TEXT,
                created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
            );
        """)

        conn.commit()
        print("Table 'chat_history' created successfully or already exists.")
        cur.close()
    except Exception as e:
        print(f"Error creating table: {e}")
    finally:
        if conn:
            conn.close()

if __name__ == "__main__":
    create_tables()
