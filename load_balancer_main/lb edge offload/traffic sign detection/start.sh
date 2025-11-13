#!/bin/bash

# Start the MPS control daemon
nvidia-cuda-mps-control -d
echo "✅ NVIDIA MPS started..."

# Now launch your object detection server
exec python3 /app/server.py
