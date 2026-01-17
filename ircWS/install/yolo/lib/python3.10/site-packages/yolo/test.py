#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import MarkerTag

import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

from flask import Flask, Response
import threading, time

YOLO_CONF_THRESH = 0.60
ENGINE_PATH = "/home/mrmnavjet/IRC2026/ircWS/src/yolo/yolo/cone_v1.engine"

COLOR_ID = {"red":1,"blue":2,"orange":3,"green":4,"yellow":5}
BOX_COLORS = {
    "red":(0,0,255),"blue":(255,0,0),
    "orange":(0,165,255),"green":(0,255,0),
    "yellow":(0,255,255)
}

HSV_RANGES = {
    "red":[((0,60,50),(10,255,255)),((170,60,50),(179,255,255))],
    "orange":[((8,80,80),(25,255,255))],
    "yellow":[((20,70,80),(38,255,255))],
    "green":[((35,60,60),(85,255,255))],
    "blue":[((90,60,60),(135,255,255))]
}

app = Flask(__name__)
latest_frame = None
lock = threading.Lock()

@app.route("/video")
def video():
    def gen():
        global latest_frame
        while True:
            with lock:
                f = None if latest_frame is None else latest_frame.copy()
            if f is None:
                time.sleep(0.01); continue
            _, jpg = cv2.imencode(".jpg", f)
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"+jpg.tobytes()+b"\r\n"
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

def start_flask():
    app.run(host="0.0.0.0", port=5001, threaded=True)

class TRT:
    def __init__(self,path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(path,"rb") as f, trt.Runtime(logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        self.in_name = self.engine.get_tensor_name(0)
        self.out_name = self.engine.get_tensor_name(1)
        self.din = cuda.mem_alloc(trt.volume(self.engine.get_tensor_shape(self.in_name))*4)
        self.dout = cuda.mem_alloc(trt.volume(self.engine.get_tensor_shape(self.out_name))*4)
        self.ctx.set_tensor_address(self.in_name,int(self.din))
        self.ctx.set_tensor_address(self.out_name,int(self.dout))
        self.hout = np.empty(self.engine.get_tensor_shape(self.out_name),np.float32)

    def infer(self,img):
        cuda.memcpy_htod_async(self.din,img,self.stream)
        self.ctx.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(self.hout,self.dout,self.stream)
        self.stream.synchronize()
        return self.hout

class NodeYOLO(Node):
    def __init__(self):
        super().__init__("yolo_tiled")
        self.bridge = CvBridge()
        self.trt = TRT(ENGINE_PATH)
        self.rgb = None
        self.depth = None

        self.input_buf = np.empty((1,3,640,640),np.float32)

        self.fps = 0.0
        self.last_t = time.time()

        self.create_subscription(Image,"/zed/zed_node/rgb/color/rect/image",self.cb_rgb,10)
        self.create_subscription(Image,"/zed/zed_node/depth/depth_registered",self.cb_depth,10)
        self.pub = self.create_publisher(MarkerTag,"/marker_detect",10)
        self.create_timer(0.05,self.process)

        threading.Thread(target=start_flask,daemon=True).start()

    def cb_rgb(self,m): self.rgb=m
    def cb_depth(self,m): self.depth=m

    def detect_color(self,roi):
        hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        area=roi.shape[0]*roi.shape[1]
        best=None; mx=0
        for c,rs in HSV_RANGES.items():
            cnt=0
            for lo,hi in rs:
                cnt+=cv2.countNonZero(cv2.inRange(hsv,lo,hi))
            if cnt>mx and cnt>0.10*area:
                mx=cnt; best=c
        return best

    def run_tiled(self,frame):
        H,W,_=frame.shape
        dets=[]
        for ty in range(2):
            for tx in range(2):
                x0,y0=tx*W//2,ty*H//2
                x1,y1=min(W,x0+W//2),min(H,y0+H//2)
                tile=frame[y0:y1,x0:x1]

                img=cv2.resize(tile,(640,640))
                self.input_buf[0]=img.transpose(2,0,1)/255.0

                out=self.trt.infer(self.input_buf)[0]
                sx,sy=(x1-x0)/640,(y1-y0)/640

                for d in out.T:
                    if d[4]<YOLO_CONF_THRESH: continue
                    cx=int(d[0]*sx)+x0; cy=int(d[1]*sy)+y0
                    bw=int(d[2]*sx); bh=int(d[3]*sy)
                    dets.append((cx-bw//2,cy-bh//2,cx+bw//2,cy+bh//2))
        return dets

    def process(self):
        global latest_frame
        if self.rgb is None: return

        now=time.time()
        dt=now-self.last_t
        self.last_t=now
        if dt>0: self.fps=0.9*self.fps+0.1*(1.0/dt)

        frame=self.bridge.imgmsg_to_cv2(self.rgb,"bgr8")
        dets=self.run_tiled(frame)

        depth=None
        if dets and self.depth:
            depth=self.bridge.imgmsg_to_cv2(self.depth,"passthrough")
            if depth.dtype==np.uint16: depth=depth.astype(np.float32)*0.001

        for x1,y1,x2,y2 in dets:
            roi=frame[y1:y2,x1:x2]
            if roi.size==0: continue
            color=self.detect_color(roi)
            if not color: continue

            cx=(x1+x2)//2; cy=(y1+y2)//2
            dist=0.0
            if depth is not None:
                d=depth[cy-2:cy+2,cx-2:cx+2]
                v=d[(d>0.1)&(d<20)]
                if v.size: dist=float(np.median(v))

            cv2.rectangle(frame,(x1,y1),(x2,y2),BOX_COLORS[color],2)
            cv2.putText(frame,f"{color} {dist:.2f}m",(x1,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,BOX_COLORS[color],2)

            msg=MarkerTag()
            msg.is_found=True
            msg.id=COLOR_ID[color]
            msg.x=dist
            msg.y=0.0
            self.pub.publish(msg)

        cv2.putText(frame,f"FPS: {self.fps:.1f}",(15,30),
                    cv2.FONT_HERSHEY_SIMPLEX,1.0,(0,255,0),2)

        with lock: latest_frame=frame.copy()

def main():
    rclpy.init()
    rclpy.spin(NodeYOLO())
    rclpy.shutdown()

if __name__=="__main__":
    main()
