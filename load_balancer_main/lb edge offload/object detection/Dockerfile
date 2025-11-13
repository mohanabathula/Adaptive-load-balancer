# Use ultralytics base image
FROM ultralytics/ultralytics:latest

# Set the working directory in the container
WORKDIR /app

# Update and install OS dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ffmpeg \
    curl \
    vim \
    wget

# Install additional Python requirements
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy your model + server code
COPY yolov5s.pt .
COPY server.py .

# --- MPS Setup ---
ENV CUDA_MPS_PIPE_DIRECTORY=/tmp/nvidia-mps
ENV CUDA_MPS_LOG_DIRECTORY=/var/log/nvidia-mps
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

# Create required directories for MPS
RUN mkdir -p /tmp/nvidia-mps /var/log/nvidia-mps

# Copy entrypoint script that launches MPS and your server
COPY start.sh /start.sh
RUN chmod +x /start.sh

# Expose the server port
EXPOSE 5010

# Launch entrypoint with MPS
ENTRYPOINT ["/start.sh"]
