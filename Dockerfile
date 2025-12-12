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

# Make start script executable
RUN chmod +x start.sh

# Expose port (Railway will set this via $PORT)
EXPOSE 8080

# Start command
CMD ["bash", "start.sh"]