# pythonanywhere/wsgi.py

import sys
import os

# --- Add your project directory to the sys.path ---
# This is the full path to your project folder on PythonAnywhere
# e.g., '/home/your_username/ai-humanoid-robotics-book'
path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if path not in sys.path:
    sys.path.insert(0, path)

# --- Change to the backend directory ---
# This is important for relative imports and for python-dotenv to find the .env file
backend_path = os.path.join(path, 'backend')
os.chdir(backend_path)

# --- Import the FastAPI app ---
# The WSGI server will look for an 'application' callable
# backend.main refers to the backend/main.py file, and 'app' is the FastAPI instance
from backend.main import app as application
