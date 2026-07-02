#!/usr/bin/env python3
"""並排比較兩個 engine 的即時偵測結果：best.engine vs best(10).engine"""
import sys, types, importlib.metadata as _md

# ── torchvision patch（含 NMS）──
_real_ver = _md.version
_md.version = lambda n: "0.18.0" if n == "torchvision" else _real_ver(n)
import torch as _torch
_tv = types.ModuleType("torchvision"); _tv.__version__ = "0.18.0"
_ops = types.ModuleType("torchvision.ops")
def _nms(boxes, scores, iou_threshold):
    if boxes.numel() == 0:
        return _torch.empty((0,), dtype=_torch.int64, device=boxes.device)
    x1,y1,x2,y2 = boxes[:,0],boxes[:,1],boxes[:,2],boxes[:,3]
    areas=(x2-x1)*(y2-y1); order=scores.argsort(descending=True); keep=[]
    while order.numel()>0:
        i=order[0]; keep.append(i.item())
        if order.numel()==1: break
        xx1=_torch.maximum(x1[i],x1[order[1:]]); yy1=_torch.maximum(y1[i],y1[order[1:]])
        xx2=_torch.minimum(x2[i],x2[order[1:]]); yy2=_torch.minimum(y2[i],y2[order[1:]])
        w=(xx2-xx1).clamp(min=0); h=(yy2-yy1).clamp(min=0); inter=w*h
        iou=inter/(areas[i]+areas[order[1:]]-inter); order=order[1:][iou<=iou_threshold]
    return _torch.tensor(keep,dtype=_torch.int64,device=boxes.device)
_ops.nms=_nms; _tv.ops=_ops
sys.modules["torchvision"]=_tv; sys.modules["torchvision.ops"]=_ops

from ultralytics import YOLO
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

BASE = "/home/iclab/ros2_kid/src/strategy/strategy/mar"
MODELS = {
    "best":     f"{BASE}/best.engine",
    "best(10)": f"{BASE}/best(10).engine",
}
IMGSZ = 320
CONF  = 0.5
TOPIC = "/camera1/image_raw"


class Comparer(Node):
    def __init__(self):
        super().__init__("accuracy_comparer")
        self.bridge = CvBridge()
        print("載入兩個 engine...")
        self.models = {}
        for tag, path in MODELS.items():
            m = YOLO(path, task="detect")
            dummy = np.zeros((IMGSZ, IMGSZ, 3), dtype=np.uint8)
            m(dummy, imgsz=IMGSZ, verbose=False, device=0)
            self.models[tag] = m
            print(f"  ✓ {tag}")
        self.sub = self.create_subscription(Image, TOPIC, self.cb, 10)
        print("\n開始比較（Ctrl+C 結束）。每幀印出兩個模型的偵測：\n")
        print(f"{'':>10s} | {'best':^28s} | {'best(10)':^28s}")
        print("-"*72)

    def fmt(self, r):
        """把一個模型的偵測結果整理成短字串"""
        if len(r.boxes) == 0:
            return "(無偵測)"
        parts = []
        # 依信心排序，最多顯示 3 個
        idx = r.boxes.conf.argsort(descending=True)[:3]
        for i in idx:
            cls = int(r.boxes.cls[int(i)])
            cf  = float(r.boxes.conf[int(i)])
            parts.append(f"id{cls}:{cf:.2f}")
        return " ".join(parts)

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        out = {}
        for tag, m in self.models.items():
            r = m(frame, imgsz=IMGSZ, conf=CONF, verbose=False, device=0)[0]
            out[tag] = self.fmt(r)
        # 並排印
        print(f"{'':>10s} | {out['best']:^28s} | {out['best(10)']:^28s}")


def main():
    rclpy.init()
    node = Comparer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n結束比較。")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()