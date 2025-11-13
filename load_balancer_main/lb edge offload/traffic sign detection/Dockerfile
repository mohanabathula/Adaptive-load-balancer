# # SAMPLE DOCKER FILE
# # yuvrajmakkena/hdt:gpu-ultralytics-yolov5
# # Base image
# # FROM ultralytics/yolov5:latest
# FROM nvcr.io/nvidia/tensorrt:21.06-py3

# # Prevent interactive prompts during package installations
# ENV DEBIAN_FRONTEND=noninteractive

# # Install system dependencies
# RUN apt-get update && \
#     apt-get install -y tzdata && \
#     ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#     dpkg-reconfigure -f noninteractive tzdata && \
#     apt-get install -y python3-pip ffmpeg && \
#     rm -rf /var/lib/apt/lists/*

# # Set the working directory in the container
# WORKDIR /app

# # Copy the requirements file
# COPY requirements.txt .

# # Install the required packages
# RUN pip install --upgrade pip
# RUN pip install --no-cache-dir -r requirements.txt
# RUN pip install --no-binary :all: flask torch torchvision opencv-python-headless numpy pycuda
# # Copy the server code and yolo model into the container 
# #COPY best.pt .
# COPY traffic_signs_a40n.engine .
# COPY yolov5_trt.py .
# COPY server.py .
# COPY libmyplugins.so .
# # Start the server
# CMD ["python3", "server.py"]

# # Base image
# FROM nvcr.io/nvidia/tensorrt:21.06-py3

# # Prevent interactive prompts
# ENV DEBIAN_FRONTEND=noninteractive

# # Install system dependencies
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#     tzdata python3-pip ffmpeg && \
#     ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#     dpkg-reconfigure -f noninteractive tzdata && \
#     apt-get clean && rm -rf /var/lib/apt/lists/*

# # Set the working directory
# WORKDIR /app

# # Copy and install Python dependencies
# COPY requirements.txt .
# RUN pip install --upgrade pip && \
#     pip install --no-cache-dir -r requirements.txt flask torch torchvision opencv-python-headless numpy pycuda

# # Copy application files
# COPY traffic_signs_a40n.engine .
# COPY yolov5_trt.py .
# COPY server.py .
# COPY libmyplugins.so .

# # Set the default command to run the application
# CMD ["python3", "server.py"]

# Base image
FROM nvcr.io/nvidia/tensorrt:23.06-py3

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    tzdata python3-pip ffmpeg && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Copy and install Python dependencies
COPY requirements.txt .
RUN pip install --upgrade pip
RUN pip install --no-cache-dir -r requirements.txt

# Copy application files
COPY traffic_signs_a40n.engine .
COPY yolov5_trt.py .
COPY server.py .
COPY libmyplugins.so .
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

# Expose the port on which the server will listen
EXPOSE 5011

# Set the default command to run the application
ENTRYPOINT ["/start.sh"]
