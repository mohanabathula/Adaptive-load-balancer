from flask import Flask, jsonify, request
import threading
import requests
import os
import numpy as np
import cv2
import zlib

app = Flask(__name__)

@app.route('/processimage', methods=['POST'])
def process_image():
    if not request.data:
        return jsonify({"status": "failure", "message": "No image found in request"}), 400

    try:
        # Extract headers
        frame_width = request.headers.get('Frame-Width')
        frame_height = request.headers.get('Frame-Height')
        client_timestamp = request.headers.get('Client-Timestamp')
        frame_identifier = request.headers.get('Frame-Identifier')
        detection_flags = request.headers.get("detection-flag", "")

        print(f"Client Timestamp: {client_timestamp}")
        print(f"Frame Identifier: {frame_identifier}")
        print(f"Detection flag: {detection_flags}")

        image_data = request.data
        responses = {}
        lock = threading.Lock()

        def safe_update_response(key, value):
            with lock:
                responses[key] = value

        def request_obd():
            try:
                obd_response = forward_to_obd(image_data, frame_width, frame_height, client_timestamp, frame_identifier)
                #print(f"OBD response: {obd_response}")
                safe_update_response('OBD_model', obd_response)
            except Exception as e:
                safe_update_response('OBD_model', f"Error: {str(e)}")

        def request_lane_detect():
            try:
                lane_response = forward_to_lane_detect(image_data, frame_width, frame_height, client_timestamp, frame_identifier)
                # print(f"Lane detection response: {lane_response}")
                safe_update_response('Lane_detection_model', lane_response)
            except Exception as e:
                safe_update_response('Lane_detection_model', f"Error: {str(e)}")

        def request_traffic_detect():
            try:
                traffic_response = forward_to_traffic_detect(image_data, frame_width, frame_height, client_timestamp, frame_identifier)
                print(f"Traffic detection response: {traffic_response}")
                safe_update_response('Traffic_detection_model', traffic_response)
            except Exception as e:
                safe_update_response('Traffic_detection_model', f"Error: {str(e)}")

        # Parse detection flags into list, strip whitespace and lower-case for safety
        flags = [flag.strip().lower() for flag in detection_flags.split(",")] if detection_flags else []

        threads = []
        if 'collision_avoidance' in flags or 'collision_detection' in flags or 'pedestrian_avoidance' in flags or 'pedestrian_detection' in flags:
            threads.append(threading.Thread(target=request_obd))
        if 'lane_detection' in flags:
            threads.append(threading.Thread(target=request_lane_detect))
        if 'traffic_sign_detection' in flags or 'traffic_light_detection' in flags:
            threads.append(threading.Thread(target=request_traffic_detect))

        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        return jsonify(responses)

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


def forward_to_obd(image_data, frame_width, frame_height, client_timestamp, frame_identifier):
    obd_service_url = os.getenv("OBD_SERVICE_URL", "http://192.168.20.16:30057/object_detection")
    headers = {
        'Content-Type': 'application/octet-stream',
        'Frame-Width': frame_width,
        'Frame-Height': frame_height,
        'Client-Timestamp': client_timestamp,
        'Frame-Identifier': frame_identifier
    }
    response = requests.post(obd_service_url, data=image_data, headers=headers)
    response.raise_for_status()
    return response.json()


def forward_to_lane_detect(image_data, frame_width, frame_height, client_timestamp, frame_identifier):
    lane_service_url = os.getenv("LANE_SERVICE_URL", "http://192.168.20.16:30055/lane_detect")
    
    # Convert bytes to numpy array
    np_arr = np.frombuffer(image_data, np.uint8)
    # Decode numpy array to OpenCV image
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    # Encode image as JPEG bytes
    _, img_encoded = cv2.imencode('.jpg', image)
    
    files = {
        'frame': ('image.jpg', img_encoded.tobytes(), 'image/jpeg')
    }
    response = requests.post(lane_service_url, files=files)
    response.raise_for_status()
    return response.json()


def forward_to_traffic_detect(image_data, frame_width, frame_height, client_timestamp, frame_identifier):
    traffic_service_url = os.getenv("TRAFFIC_SERVICE_URL", "http://192.168.20.16:30053/traffic-sign")
    
    # Convert bytes to numpy array
    np_arr = np.frombuffer(image_data, np.uint8)
    # Decode numpy array to OpenCV image
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    # Encode image as JPEG bytes
    _, img_encoded = cv2.imencode('.jpg', image)
    # Compress bytes with zlib
    compressed_bytes = zlib.compress(img_encoded.tobytes())
    
    headers = {
        'Content-Type': 'application/octet-stream',
        'Frame-Width': frame_width,
        'Frame-Height': frame_height,
    }
    
    
    response = requests.post(traffic_service_url, data=compressed_bytes, headers=headers)
    response.raise_for_status()
    return response.json()


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
