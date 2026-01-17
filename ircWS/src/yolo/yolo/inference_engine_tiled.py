#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.logging
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import MarkerTag

import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

from flask import Flask, Response
import threading
import time

YOLO_CONF_THRESH = 0.40
ENGINE_PATH = "/home/mrmnavjet/IRC2026/ircWS/src/yolo/yolo/cone_v1.engine"

FX = 475.26
CX = 324.94

COLOR_ORDER = ["orange", "red", "blue", "green", "yellow"]

COLOR_ID = {
    "orange": 1,
    "red": 2,
    "blue": 3,
    "green": 4,
    "yellow": 5
}

HSV_RANGES = {
    "red": [((0,60,50),(10,255,255)), ((170,60,50),(179,255,255))],
    "orange": [((8,80,80),(25,255,255))],
    "yellow": [((20,70,80),(38,255,255))],
    "green": [((35,60,60),(85,255,255))],
    "blue": [((90,60,60),(135,255,255))]
}

BOX_COLORS = {
    "red": (0,0,255),
    "blue": (255,0,0),
    "orange": (0,165,255),
    "green": (0,255,0),
    "yellow": (0,255,255)
}

app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()

@app.route("/video")
def video():
    def gen():
        global latest_frame
        while True:
            with frame_lock:
                frame = None if latest_frame is None else latest_frame.copy()
            if frame is None:
                time.sleep(0.005)
                continue
            ok, jpg = cv2.imencode(".jpg", frame)
            if not ok:
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                   jpg.tobytes() + b"\r\n")
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

def start_flask():
    app.run(host="0.0.0.0", port=5001, debug=False, threaded=True)

class TRTInfer:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        self.input_name = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)
        self.input_shape = self.engine.get_tensor_shape(self.input_name)
        self.output_shape = self.engine.get_tensor_shape(self.output_name)
        self.d_input = cuda.mem_alloc(trt.volume(self.input_shape) * np.float32().nbytes)
        self.d_output = cuda.mem_alloc(trt.volume(self.output_shape) * np.float32().nbytes)
        self.context.set_tensor_address(self.input_name, int(self.d_input))
        self.context.set_tensor_address(self.output_name, int(self.d_output))
        self.h_output = np.empty(self.output_shape, dtype=np.float32)

    def infer(self, img):
        cuda.memcpy_htod_async(self.d_input, img, self.stream)
        self.context.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
        self.stream.synchronize()
        return self.h_output

class InferenceEngine(Node):
    def __init__(self):
        super().__init__("inference_engine")
        self.bridge = CvBridge()
        self.trt = TRTInfer(ENGINE_PATH)
        self.latest_rgb = None
        self.latest_depth = None
        self.last_t = time.monotonic()
        self.fps = 0.0

        self.create_subscription(Image, "/zed/zed_node/rgb/color/rect/image", self.rgb_cb, 10)
        self.create_subscription(Image, "/zed/zed_node/depth/depth_registered", self.depth_cb, 10)

        self.pub = self.create_publisher(MarkerTag, "/marker_detect", 10)
        self.timer = self.create_timer(0.03, self.process)

        threading.Thread(target=start_flask, daemon=True).start()

    def rgb_cb(self, msg):
        self.latest_rgb = msg

    def depth_cb(self, msg):
        self.latest_depth = msg

    def detect_color(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        area = roi.shape[0] * roi.shape[1]
        best, mx = None, 0
        for c, ranges in HSV_RANGES.items():
            cnt = sum(cv2.countNonZero(cv2.inRange(hsv, lo, hi)) for lo, hi in ranges)
            if cnt > mx and cnt > 0.08 * area:
                mx, best = cnt, c
        return best

    def calculate_distance(self, depth, u, v):
        if depth is None:
            return None
        ps = 8
        h, w = depth.shape
        patch = depth[max(0,int(v-ps)):min(h,int(v+ps)),
                      max(0,int(u-ps)):min(w,int(u+ps))]
        vals = patch[(patch > 0.1) & (patch < 20.0)]
        return None if vals.size == 0 else float(np.median(vals))

    def process(self):
        if self.latest_rgb is None:
            return

        t = time.monotonic()
        self.fps = 0.9*self.fps + 0.1*(1.0/(t-self.last_t))
        self.last_t = t

        frame = self.bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
        depth = None
        if self.latest_depth:
            depth = self.bridge.imgmsg_to_cv2(self.latest_depth, "passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * 0.001

        H, W, _ = frame.shape
        img = cv2.resize(frame, (640,640))
        img = img.transpose(2,0,1)
        img = np.ascontiguousarray(img, dtype=np.float32)/255.0
        img = img[None]

        out = self.trt.infer(img)[0]
        sx, sy = W/640.0, H/640.0

        detections = {}

        for d in out.T:
            if d[4] < YOLO_CONF_THRESH:
                continue

            u = d[0]*sx
            v = d[1]*sy
            bw, bh = int(d[2]*sx), int(d[3]*sy)
            cx_i, cy_i = int(u), int(v)

            x1,y1 = max(0,cx_i-bw//2), max(0,cy_i-bh//2)
            x2,y2 = min(cx_i+bw//2,W), min(cy_i+bh//2,H)

            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                continue

            color = self.detect_color(roi)
            if color is None:
                continue

            dist = self.calculate_distance(depth, u, v)
            if dist is None:
                continue

            if color not in detections or d[4] > detections[color]["conf"]:
                detections[color] = {
                    "bbox": (x1,y1,x2,y2),
                    "u": u,
                    "v": v,
                    "dist": dist,
                    "conf": d[4]
                }

        for color in COLOR_ORDER:
            if color not in detections:
                continue

            d = detections[color]
            x1,y1,x2,y2 = d["bbox"]
            dist = d["dist"]
            u = d["u"]

            cone_y = (u - CX) * dist / FX

            msg = MarkerTag()
            msg.is_found = True
            msg.id = COLOR_ID[color]
            msg.x = float(dist)
            msg.y = float(cone_y)
            self.pub.publish(msg)

            cv2.rectangle(frame,(x1,y1),(x2,y2),BOX_COLORS[color],2)
            cv2.putText(frame,f"{dist:.2f} m",(x1,y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,BOX_COLORS[color],2)

        cv2.putText(frame,f"FPS: {self.fps:.1f}",(10,30),
                    cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)

        with frame_lock:
            global latest_frame
            latest_frame = frame.copy()

def main():
    rclpy.init()
    rclpy.logging.set_logger_level("inference_engine", rclpy.logging.LoggingSeverity.INFO)
    rclpy.spin(InferenceEngine())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
