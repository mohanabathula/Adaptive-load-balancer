import torch
import cv2
import numpy as np
if not hasattr(np, 'bool'):
    np.bool = np.bool_
from flask import Flask, request, jsonify
from yolov5_trt import YoLov5TRT #, profiler
import zlib
import time
from datetime import datetime

app = Flask(__name__)
frame_count_client = 0
frame_count_server = 0
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)
classes = ['go', 'right', 'park', 'red', 'green', 'crosswalk']
model = YoLov5TRT('./traffic_signs_a40n.engine', './libmyplugins.so', classes)

# Specify the desired dimensions for the resized frames
RESIZED_WIDTH = 1200
RESIZED_HEIGHT = 800
    
@app.route('/traffic-sign', methods=['POST'])
def video():
    
    st = time.time()
    global frame_count_client, frame_count_server

    frame_data = request.data

    reqtime = time.time()
    frame_data = zlib.decompress(frame_data)
    decomptime = time.time()
    frame = np.frombuffer(frame_data, np.uint8)
    npconv = time.time()
    img = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    bytestoimgmat = time.time()
    frame_width = int(request.headers['Frame-Width'])
    frame_height = int(request.headers['Frame-Height'])
    #client_timestamp = float(request.headers['Client-Timestamp'])
    #server_to_client_delay = time.time() - client_timestamp

    img = cv2.resize(img, (RESIZED_WIDTH, RESIZED_HEIGHT))
    start_time = time.time()
    # Inference
    boxes, scores, class_ids = model.infer(img)
    #model.log_profiler_summary()
    print(boxes,scores,class_ids)
    scores = [float(i) for i in scores]
    boxes = [i.tolist() for i in boxes]
    temp = []
    for i in boxes:
        p = [float(round(j,3)) for j in i]
        temp.append(p)
    boxes = temp

    end_time = time.time()
    processing_delay = end_time - start_time
    # Appending labels, confidence to detections
    st_det = time.time()
    et = time.time()
    total_process = et - st
    rst = start_time - bytestoimgmat
    appending_det_values = et - st_det
    reqt = reqtime - st
    npconvt = npconv - decomptime
    decompt = decomptime - reqtime
    bytestoimgt = bytestoimgmat - npconv
    response_data = {
        # 'Inference-Delay': processing_delay,
        # 'Before-inference-time': start_time,
        # 'After-inference-time': end_time,
        # 'total-process': total_process,
        # 'total-process-start-time': st,
        # 'total-process-end-time': et,
        # 'resizing-time': rst,
        # 'Appending-Det-Values': appending_det_values,
        'classids' : class_ids,
        'scores': scores,
        'boxes': temp,
        # 'reqtime': reqt,
        # 'decompress-time': decompt,
        # 'np-conversion': npconvt,
        # 'bytes-to-img': bytestoimgt
    }
    return jsonify(success=True, data="Detected labels and confidences sent to client.", **response_data)

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5011)
    except Exception as e:
        model.destroy()
