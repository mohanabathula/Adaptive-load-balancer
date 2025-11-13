# Use the full Python 3.10 image
FROM python:3.10

# Set the working directory in the container
WORKDIR /app

# Install necessary libraries for OpenCV GUI support
RUN apt update && apt install python3-pip -y && apt install ffmpeg -y

COPY requirements.txt .
RUN pip install --upgrade pip
RUN pip install --no-cache-dir -r requirements.txt
# RUN pip3 install Flask
COPY . .

EXPOSE 5001

CMD [ "python3", "server.py"]
