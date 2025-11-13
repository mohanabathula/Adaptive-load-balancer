# # Use an official Python runtime as a parent image
# FROM python:3.10-slim

# # Set the working directory inside the container
# WORKDIR /app

# # Install pip and system dependencies for OpenCV
# RUN apt-get update && apt-get install -y \
#     libgl1-mesa-glx \
#     libglib2.0-0 \
#     && rm -rf /var/lib/apt/lists/*

# # Copy the requirements file into the container
# COPY requirements.txt .

# # Upgrade pip
# RUN pip install --upgrade pip

# # Install Python dependencies
# RUN pip install --no-cache-dir -r requirements.txt

# # Copy the rest of the application code into the container
# COPY . .

# # Set the environment variable to avoid buffering issues
# ENV PYTHONUNBUFFERED=1

# # Expose the port on which the Flask app will run
# EXPOSE 5000

# # Command to run the Flask app
# CMD ["flask", "run", "--host=0.0.0.0"]





# Use the full Python 3.10 image
FROM python:3.10

# Set the working directory in the container
WORKDIR /app

RUN apt update
# Install necessary libraries for OpenCV GUI support
RUN apt-get install -y libglib2.0-0 libsm6 libxext6 libxrender-dev libgl1-mesa-glx
# RUN apt-get install -y libsm6 libxext6 libxrender-dev libgl1-mesa-glx
# RUN apt-get install -y libsm6 libxext6 libxrender-dev
# Copy requirements and install dependencies
COPY requirements.txt .
RUN pip install --upgrade pip
RUN pip install --no-cache-dir -r requirements.txt
# RUN pip3 install Flask
COPY . .

EXPOSE 5005

CMD [ "python3", "server.py"]


# CMD ["python3", "lane_detect.py"]




