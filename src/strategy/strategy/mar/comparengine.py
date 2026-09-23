#!/usr/bin/env python3
"""比較兩個不同模型的 engine：best.engine(320) vs best(9).engine(640)"""
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
# 每個 engine 配它自己的尺寸
MODELS = {
    "best(320)":    {"path": f"{BASE}/best.engine",    "imgsz": 320},
    "best(9)(640)": {"path": f"{BASE}/best(9).engine", "imgsz": 640},
}
CONF  = 0.5
TOPIC = "/camera1/image_raw"
N_FRAMES = 100


class Comparer(Node):
    def __init__(self):
        super().__init__("engine_comparer")
        self.bridge = CvBridge()
        print("載入兩個 engine 中...")
        self.models = {}
        for tag, cfg in MODELS.items():
            try:
                m = YOLO(cfg["path"], task="detect")
                dummy = np.zeros((cfg["imgsz"], cfg["imgsz"], 3), dtype=np.uint8)
                m(dummy, imgsz=cfg["imgsz"], verbose=False, device=0)
                self.models[tag] = (m, cfg["imgsz"])
                print(f"  ✓ {tag:14s} 載入成功 (imgsz={cfg['imgsz']})")
            except Exception as e:
                print(f"  ✗ {tag:14s} 載入失敗: {e}")

        self.times = {t: [] for t in self.models}
        self.dets  = {t: [] for t in self.models}
        self.frame_i = 0
        self.sub = self.create_subscription(Image, TOPIC, self.cb, 10)
        print(f"\n開始比較（目標 {N_FRAMES} 幀）...\n")

    def cb(self, msg):
        if self.frame_i >= N_FRAMES:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        line = f"[幀 {self.frame_i+1:3d}] "
        best_cls = {}
        for tag, (m, sz) in self.models.items():
            r = m(frame, imgsz=sz, conf=CONF, verbose=False, device=0)[0]
            inf_ms = r.speed["inference"]
            self.times[tag].append(inf_ms)
            n = len(r.boxes)
            self.dets[tag].append(n)
            if n > 0:
                top = r.boxes[int(r.boxes.conf.argmax())]
                best_cls[tag] = int(top.cls[0])
            else:
                best_cls[tag] = None
            line += f"{tag}={inf_ms:5.1f}ms({n}) "
        agree = "✓一致" if len(set(best_cls.values())) == 1 else "✗不同"
        print(line + agree)
        self.frame_i += 1
        if self.frame_i >= N_FRAMES:
            self.report()
            rclpy.shutdown()

    def report(self):
        print("\n" + "="*64)
        print(f"{'模型':16s}{'平均(ms)':>10s}{'FPS':>8s}{'平均偵測數':>12s}")
        print("="*64)
        for tag in self.models:
            t = np.array(self.times[tag]); d = np.array(self.dets[tag])
            avg = t.mean()
            print(f"{tag:16s}{avg:>10.2f}{1000/avg:>8.1f}{d.mean():>12.2f}")
        print("="*64)
        tags = list(self.models)
        if len(tags) == 2:
            a, b = tags
            ta, tb = np.mean(self.times[a]), np.mean(self.times[b])
            faster = a if ta < tb else b
            print(f"\n{faster} 比較快（{max(ta,tb)/min(ta,tb):.2f} 倍）")
            print("註：兩者尺寸不同，速度差含尺寸因素，非純模型差異")


def main():
    rclpy.init()
    node = Comparer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()