import cv2
import requests
import zlib
import time

# server_url = "http://192.168.20.19:8080/video_feed"
server_url= "http://192.168.20.20:30053/traffic-sign"



cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Encode frame as JPEG bytes
    success, img_encoded = cv2.imencode('.jpg', frame)
    if not success:
        print("Could not encode image")
        continue

    # Compress bytes with zlib
    compressed_bytes = zlib.compress(img_encoded.tobytes())

    # Prepare headers with frame dimensions
    headers = {
        'Content-Type': 'application/octet-stream',
        'Frame-Width': str(frame.shape[1]),
        'Frame-Height': str(frame.shape[0])
    }

    try:
        response = requests.post(server_url, data=compressed_bytes, headers=headers, timeout=5)
        if response.status_code == 200:
            print("Server response:", response.json())
        else:
            print(f"Server returned status code {response.status_code}")
    except requests.exceptions.RequestException as e:
        print("Error sending frame:", e)

    # Show frame locally (optional)
    cv2.imshow('Camera', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Optional delay to limit FPS
    time.sleep(0.1)

cap.release()
cv2.destroyAllWindows()
