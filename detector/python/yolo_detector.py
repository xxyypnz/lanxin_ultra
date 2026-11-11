"""OpenCV DNN YOLOv8 ONNX detector used by the embedded C ABI wrapper."""

import json
import os

import cv2
import numpy as np


class YoloDetector:
    def __init__(self, model_path, labels_path):
        if not os.path.isfile(model_path):
            raise FileNotFoundError("YOLO ONNX model not found: " + model_path)
        if not os.path.isfile(labels_path):
            raise FileNotFoundError("label file not found: " + labels_path)

        with open(labels_path, "r", encoding="utf-8") as label_file:
            self.labels = [line.strip() for line in label_file if line.strip()]
        if not self.labels:
            raise ValueError("label file is empty")

        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.input_size = int(os.environ.get("DETECT_INPUT_SIZE", "640"))
        self.confidence_threshold = float(os.environ.get("DETECT_CONFIDENCE", "0.35"))
        self.nms_threshold = float(os.environ.get("DETECT_NMS", "0.45"))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect(self, rgb_bytes, width, height):
        expected_size = width * height * 3
        if width <= 0 or height <= 0 or len(rgb_bytes) != expected_size:
            raise ValueError("input must be tightly packed RGB888")

        image = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((height, width, 3)).copy()
        scale = min(self.input_size / width, self.input_size / height)
        resized_width = int(round(width * scale))
        resized_height = int(round(height * scale))
        resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        pad_x = (self.input_size - resized_width) // 2
        pad_y = (self.input_size - resized_height) // 2
        canvas = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized

        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0,
                                     (self.input_size, self.input_size),
                                     swapRB=False, crop=False)
        self.net.setInput(blob)
        output = np.asarray(self.net.forward())
        rows = self._rows(output)

        boxes = []
        scores = []
        class_ids = []
        for row in rows:
            if row.size < 6:
                continue
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])
            if score < self.confidence_threshold or class_id >= len(self.labels):
                continue

            center_x, center_y, box_width, box_height = map(float, row[:4])
            x1 = int(round((center_x - box_width / 2 - pad_x) / scale))
            y1 = int(round((center_y - box_height / 2 - pad_y) / scale))
            x2 = int(round((center_x + box_width / 2 - pad_x) / scale))
            y2 = int(round((center_y + box_height / 2 - pad_y) / scale))
            x1 = max(0, min(width - 1, x1))
            y1 = max(0, min(height - 1, y1))
            x2 = max(0, min(width - 1, x2))
            y2 = max(0, min(height - 1, y2))
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(score)
            class_ids.append(class_id)

        detections = []
        if boxes:
            indexes = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_threshold,
                                       self.nms_threshold)
            for index in np.asarray(indexes).reshape(-1):
                i = int(index)
                x, y, box_width, box_height = boxes[i]
                label = self.labels[class_ids[i]]
                score = scores[i]
                # OpenCV operates on the RGB array here; this color is red in RGB.
                cv2.rectangle(image, (x, y), (x + box_width, y + box_height), (255, 32, 32), 2)
                text = "{} {:.1f}%".format(label, score * 100.0)
                baseline_y = max(18, y - 6)
                cv2.putText(image, text, (x, baseline_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255, 32, 32), 2, cv2.LINE_AA)
                detections.append({
                    "label": label,
                    "confidence": round(score, 6),
                    "box": [x, y, x + box_width, y + box_height],
                })

        return image.tobytes(), json.dumps(detections, ensure_ascii=False, separators=(",", ":"))

    @staticmethod
    def _rows(output):
        values = np.squeeze(output)
        if values.ndim != 2:
            raise ValueError("unsupported YOLO output tensor shape: {}".format(output.shape))
        # YOLOv8 exports commonly use [4 + classes, candidates].
        if values.shape[0] < values.shape[1] and values.shape[0] >= 6:
            values = values.T
        if values.shape[1] < 6:
            raise ValueError("unsupported YOLO output row width: {}".format(values.shape[1]))
        return values


_detector = None


def initialize(model_path, labels_path):
    global _detector
    _detector = YoloDetector(model_path, labels_path)


def detect_rgb(rgb_bytes, width, height):
    if _detector is None:
        raise RuntimeError("detector has not been initialized")
    return _detector.detect(rgb_bytes, width, height)
