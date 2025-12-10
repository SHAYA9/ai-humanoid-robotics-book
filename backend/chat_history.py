from . import neon_client

async def get_history():
    """Retrieves the chat history from the Neon database."""
    conn = await neon_client.get_db_connection()
    if not conn:
        return []

    try:
        with conn.cursor(cursor_factory=psycopg2.extras.DictCursor) as cur:
            cur.execute("SELECT question, answer, sources, created_at FROM chat_history ORDER BY created_at DESC LIMIT 50")
            history = [dict(row) for row in cur.fetchall()]
        return history
    except Exception as e:
        print(f"Error fetching chat history: {e}")
        return []
    finally:
        if conn:
            conn.close()
