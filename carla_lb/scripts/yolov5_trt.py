"""
TensorRT YOLOv5 inference WITHOUT PyCUDA
Uses cuda-python (official NVIDIA bindings)
"""

import ctypes
import time
import cv2
import random
import numpy as np
import tensorrt as trt

import cuda
import cuda.cudart as cudart

LEN_ALL_RESULT = 38001
LEN_ONE_RESULT = 38


class Colors:
    def __init__(self):
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))


colors = Colors()


def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    tl = (line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1)
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3,
                    [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)


class YoLov5TRT(object):

    def __init__(self, engine_file_path, plugin, classes,
                 conf_thresh=0.3, iou_threshold=0.4):

        self.CONF_THRESH = conf_thresh
        self.IOU_THRESHOLD = iou_threshold

        ctypes.CDLL(plugin)

        self.categories = classes
        self.engine_file_path = engine_file_path

        TRT_LOGGER = trt.Logger(trt.Logger.INFO)

        with open(self.engine_file_path, "rb") as f:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # Allocate GPU buffers (NO PyCUDA)
        self.bindings = []
        self.device_buffers = []
        self.host_buffers = []

        for i in range(self.engine.num_bindings):
            name = self.engine.get_binding_name(i)
            shape = self.engine.get_binding_shape(i)
            dtype = trt.nptype(self.engine.get_binding_dtype(i))

            size = trt.volume(shape)
            # Allocate host buffer
            host_mem = np.empty(size, dtype=dtype)
            self.host_buffers.append(host_mem)

            # Allocate device buffer using cuda-python
            err, dev_ptr = cudart.cudaMalloc(host_mem.nbytes)
            assert err == cudart.cudaError_t.cudaSuccess
            self.device_buffers.append(dev_ptr)

            self.bindings.append(int(dev_ptr))

            if self.engine.binding_is_input(i):
                self.input_index = i
                self.input_h = shape[-2]
                self.input_w = shape[-1]
            else:
                self.output_index = i

        # CUDA stream
        err, self.stream = cudart.cudaStreamCreate()
        assert err == cudart.cudaError_t.cudaSuccess

    # -------------------------------------------------------

    def infer(self, image):
        # Preprocess
        input_tensor, image_raw, h, w = self.preprocess_image(image)

        # Copy host → device
        self.host_buffers[self.input_index][:] = input_tensor.ravel()

        err = cudart.cudaMemcpyAsync(
            self.device_buffers[self.input_index],
            self.host_buffers[self.input_index].ctypes.data,
            self.host_buffers[self.input_index].nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
            self.stream
        )
        assert err == cudart.cudaError_t.cudaSuccess

        # Run inference
        self.context.execute_async_v2(self.bindings, self.stream)

        # Copy device → host
        err = cudart.cudaMemcpyAsync(
            self.host_buffers[self.output_index].ctypes.data,
            self.device_buffers[self.output_index],
            self.host_buffers[self.output_index].nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
            self.stream
        )
        assert err == cudart.cudaError_t.cudaSuccess

        cudart.cudaStreamSynchronize(self.stream)

        # Post-process
        output = self.host_buffers[self.output_index]
        boxes, scores, class_ids = self.post_process(
            output[:LEN_ALL_RESULT], h, w
        )

        return boxes, scores, class_ids

    # -------------------------------------------------------

    def destroy(self):
        for buf in self.device_buffers:
            cudart.cudaFree(buf)
        cudart.cudaStreamDestroy(self.stream)

    # ------------------ PREPROCESS -------------------------

    def preprocess_image(self, raw_bgr_image):
        image_raw = raw_bgr_image
        h, w, _ = image_raw.shape

        image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
        r_w = self.input_w / w
        r_h = self.input_h / h

        if r_h > r_w:
            tw = self.input_w
            th = int(r_w * h)
            tx1 = tx2 = 0
            ty1 = int((self.input_h - th) / 2)
            ty2 = self.input_h - th - ty1
        else:
            tw = int(r_h * w)
            th = self.input_h
            tx1 = int((self.input_w - tw) / 2)
            tx2 = self.input_w - tw - tx1
            ty1 = ty2 = 0

        image = cv2.resize(image, (tw, th))
        image = cv2.copyMakeBorder(image, ty1, ty2, tx1, tx2,
                                   cv2.BORDER_CONSTANT, None, (128, 128, 128))
        image = image.astype(np.float32) / 255.0
        image = np.transpose(image, [2, 0, 1])
        image = np.expand_dims(image, 0)
        return image, image_raw, h, w

    # ------------------ POSTPROCESS ------------------------

    def xywh2xyxy(self, origin_h, origin_w, x):
        y = np.zeros_like(x)
        r_w = self.input_w / origin_w
        r_h = self.input_h / origin_h
        if r_h > r_w:
            y[:, 0] = x[:, 0] - x[:, 2] / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2 - (self.input_h - r_w * origin_h) / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2 - (self.input_h - r_w * origin_h) / 2
            y /= r_w
        else:
            y[:, 0] = x[:, 0] - x[:, 2] / 2 - (self.input_w - r_h * origin_w) / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2 - (self.input_w - r_h * origin_w) / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2
            y /= r_h
        return y

    def post_process(self, output, h, w):
        num = int(output[0])
        pred = np.reshape(output[1:], (-1, LEN_ONE_RESULT))[:num, :]
        pred = pred[:, :6]

        boxes = self.non_max_suppression(pred, h, w,
                                         self.CONF_THRESH, self.IOU_THRESHOLD)

        if len(boxes):
            return boxes[:, :4], boxes[:, 4], boxes[:, 5].astype(int)
        return [], [], []

    def bbox_iou(self, box1, box2):
        b1 = box1
        b2 = box2
        inter = (
            np.maximum(b1[:, 0], b2[:, 0]) *
            np.maximum(b1[:, 1], b2[:, 1])
        )

    def non_max_suppression(self, prediction, h, w,
                            conf_thres=0.4, nms_thres=0.4):

        boxes = prediction[prediction[:, 4] >= conf_thres]
        if not len(boxes):
            return np.array([])

        boxes[:, :4] = self.xywh2xyxy(h, w, boxes[:, :4])

        boxes[:, 0] = np.clip(boxes[:, 0], 0, w - 1)
        boxes[:, 2] = np.clip(boxes[:, 2], 0, w - 1)
        boxes[:, 1] = np.clip(boxes[:, 1], 0, h - 1)
        boxes[:, 3] = np.clip(boxes[:, 3], 0, h - 1)

        confs = boxes[:, 4]
        boxes = boxes[np.argsort(-confs)]

        keep = []
        while len(boxes):
            box = boxes[0]
            keep.append(box)

            if len(boxes) == 1:
                break

            ious = self.bbox_iou_np(box, boxes[1:])
            boxes = boxes[1:][ious < nms_thres]

        return np.stack(keep)

    def bbox_iou_np(self, box, boxes):
        x1 = np.maximum(box[0], boxes[:, 0])
        y1 = np.maximum(box[1], boxes[:, 1])
        x2 = np.minimum(box[2], boxes[:, 2])
        y2 = np.minimum(box[3], boxes[:, 3])

        inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)

        area1 = (box[2] - box[0]) * (box[3] - box[1])
        area2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])

        return inter / (area1 + area2 - inter + 1e-6)

