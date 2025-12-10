import os
import mysql.connector
from mysql.connector import Error
from dotenv import load_dotenv

load_dotenv()

# --- MySQL Connection Details from Environment Variables ---
DB_HOST = os.getenv("MYSQL_HOST")
DB_DATABASE = os.getenv("MYSQL_DATABASE")
DB_USER = os.getenv("MYSQL_USER")
DB_PASSWORD = os.getenv("MYSQL_PASSWORD")

def get_db_connection():
    """Establishes a connection to the MySQL database."""
    try:
        conn = mysql.connector.connect(
            host=DB_HOST,
            database=DB_DATABASE,
            user=DB_USER,
            password=DB_PASSWORD
        )
        if conn.is_connected():
            return conn
    except Error as e:
        print(f"Error connecting to MySQL database: {e}")
        return None

def create_chat_history_table():
    """Creates the chat_history table if it doesn't exist."""
    conn = get_db_connection()
    if conn:
        try:
            cursor = conn.cursor()
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS chat_history (
                    id INT AUTO_INCREMENT PRIMARY KEY,
                    question TEXT NOT NULL,
                    answer TEXT NOT NULL,
                    sources TEXT,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """)
            print("Successfully created or verified chat_history table.")
        except Error as e:
            print(f"Error creating table: {e}")
        finally:
            conn.close()

def save_chat_message(question: str, answer: str, sources: str):
    """Saves a chat message to the database."""
    conn = get_db_connection()
    if conn:
        try:
            cursor = conn.cursor()
            query = "INSERT INTO chat_history (question, answer, sources) VALUES (%s, %s, %s)"
            cursor.execute(query, (question, answer, sources))
            conn.commit()
        except Error as e:
            print(f"Error saving chat message: {e}")
        finally:
            conn.close()

# Initialize the table on startup
if __name__ == "__main__":
    create_chat_history_table()
