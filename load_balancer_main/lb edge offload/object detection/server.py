import cv2
import numpy as np
from flask import Flask, Response, request, jsonify
import threading
from ultralytics import YOLO
import time
import datetime

app = Flask(__name__)
current_frame = None
frame_count_client = 0
frame_count_server = 0
model = YOLO("yolov5s.pt").cuda()

# Specify the desired dimensions for the resized frames
# RESIZED_WIDTH = 1200
# RESIZED_HEIGHT = 800

#Resizing the height and width for depth image
RESIZED_WIDTH = 640
RESIZED_HEIGHT = 480

@app.route('/object_detection', methods=['POST', 'GET'])
def video_feed():
    global current_frame, frame_count_client, frame_count_server
    if request.method == 'POST':
        # Handle incoming video frame
        # try:
            frame_data = request.data
            # print("Frame data from dummy is ", frame_data)
            frame = np.frombuffer(frame_data, np.uint8)
            img = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            frame_width = int(request.headers.get('Frame-Width', RESIZED_WIDTH))
            frame_height = int(request.headers.get('Frame-Height', RESIZED_HEIGHT))

            client_timestamp = float(request.headers.get('Client-Timestamp', time.time()))
            server_to_client_delay = time.time() - client_timestamp
            print('Delay time (client to server):', server_to_client_delay)

            img = cv2.resize(img, (RESIZED_WIDTH, RESIZED_HEIGHT))

            # Perform YOLOv5 detection
            start_time = time.time()
            results = model.predict(img)
            end_time = time.time()
            processing_delay = end_time - start_time
            print('Processing delay:', processing_delay)

            current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            print('Frame received by server at:', current_time)

            detection_results = []

            # Draw results on the frame
            for result in results:
                for obj in result.boxes:
                    x1, y1, x2, y2 = map(int, obj.xyxy[0])
                    label = result.names[int(obj.cls[0])]
                    confidence = obj.conf[0]

                    # cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # cv2.putText(img, f'{label}: {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    # Store detection information
                    detection_results.append({
                        'bbox': [x1, y1, x2, y2],
                        'class': label,
                        'confidence': float(confidence)
                    })

            # Encode and store the processed frame
            ret, buffer = cv2.imencode('.jpg', img)
            if not ret:
                return "Error encoding frame.", 500
            current_frame = buffer.tobytes()
            frame_count_client += 1
            frame_count_server += 1
            print("Received frame from client. Total frames received:", frame_count_client)
            print("Total frames processed:", frame_count_server)

            server_timestamp = time.time()
            server_to_browser_delay = server_timestamp - client_timestamp
            print('Delay time (server to browser URL):', server_to_browser_delay)

            current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            print('Frame sent to browser at:', current_time)

            # return "Video frame received."
            return jsonify({
                    "message": "Video frame received.",
                    "detections": detection_results})

        # except Exception as e:
        #     print("Error processing frame:", e)
        #     return "Error processing frame.", 500
    else:
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def generate():
    global current_frame
    while True:
        if current_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n')
        else:
            # Sleep briefly to avoid high CPU usage when no frame is available
            time.sleep(0.1)

def process_frame():
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    threading.Thread(target=process_frame).start()
    app.run(host='0.0.0.0', port=5010)
