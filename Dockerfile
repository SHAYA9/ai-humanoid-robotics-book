# Use Python 3.11 slim image
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Copy backend requirements
COPY backend/requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy backend code
COPY backend/ .

# Expose port (Railway will set this via $PORT)
EXPOSE 8080

# Start command - use shell form to allow $PORT variable
CMD uvicorn main:app --host 0.0.0.0 --port ${PORT:-8080}