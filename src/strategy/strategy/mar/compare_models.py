#!/usr/bin/env python3
"""
比較 .pt / .onnx / .engine 三種格式的速度與偵測一致性
資料來源：相機即時畫面 /camera1/image_raw
"""
import sys, types, importlib.metadata as _md

# ── torchvision patch（含 NMS，必要）──────────────────────────────
_real_ver = _md.version
_md.version = lambda n: "0.18.0" if n == "torchvision" else _real_ver(n)
import torch as _torch
_tv = types.ModuleType("torchvision"); _tv.__version__ = "0.18.0"
_ops = types.ModuleType("torchvision.ops")
def _nms(boxes, scores, iou_threshold):
    if boxes.numel() == 0:
        return _torch.empty((0,), dtype=_torch.int64, device=boxes.device)
    x1, y1, x2, y2 = boxes[:,0], boxes[:,1], boxes[:,2], boxes[:,3]
    areas = (x2-x1)*(y2-y1); order = scores.argsort(descending=True); keep=[]
    while order.numel() > 0:
        i = order[0]; keep.append(i.item())
        if order.numel() == 1: break
        xx1=_torch.maximum(x1[i],x1[order[1:]]); yy1=_torch.maximum(y1[i],y1[order[1:]])
        xx2=_torch.minimum(x2[i],x2[order[1:]]); yy2=_torch.minimum(y2[i],y2[order[1:]])
        w=(xx2-xx1).clamp(min=0); h=(yy2-yy1).clamp(min=0); inter=w*h
        iou=inter/(areas[i]+areas[order[1:]]-inter); order=order[1:][iou<=iou_threshold]
    return _torch.tensor(keep, dtype=_torch.int64, device=boxes.device)
_ops.nms=_nms; _tv.ops=_ops
sys.modules["torchvision"]=_tv; sys.modules["torchvision.ops"]=_ops

from ultralytics import YOLO
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ── 設定（依你的環境改）────────────────────────────────────────
BASE = "/home/iclab/ros2_kid/src/strategy/strategy/mar"
MODELS = {
    "pt":     f"{BASE}/best(9).pt",
    "onnx":   f"{BASE}/best(9).onnx",
    "engine": f"{BASE}/best(9).engine",
}
IMGSZ = 640
CONF  = 0.5
TOPIC = "/camera1/image_raw"
N_FRAMES = 100          # 比較幾幀後輸出統計


class Comparer(Node):
    def __init__(self):
        super().__init__("model_comparer")
        self.bridge = CvBridge()

        print("載入三種模型中...")
        self.models = {}
        for tag, path in MODELS.items():
            try:
                m = YOLO(path)
                # 各自 warmup 一次
                dummy = np.zeros((IMGSZ, IMGSZ, 3), dtype=np.uint8)
                m(dummy, imgsz=IMGSZ, verbose=False, device=0)
                self.models[tag] = m
                print(f"  ✓ {tag:6s} 載入成功")
            except Exception as e:
                print(f"  ✗ {tag:6s} 載入失敗: {e}")

        # 統計累積
        self.times = {t: [] for t in self.models}      # 各格式推論耗時(ms)
        self.dets  = {t: [] for t in self.models}      # 各格式每幀偵測數
        self.frame_i = 0

        self.sub = self.create_subscription(Image, TOPIC, self.cb, 10)
        print(f"\n開始比較（目標 {N_FRAMES} 幀）...\n")

    def cb(self, msg):
        if self.frame_i >= N_FRAMES:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        line = f"[幀 {self.frame_i+1:3d}] "
        per_frame_best = {}
        for tag, m in self.models.items():
            r = m(frame, imgsz=IMGSZ, conf=CONF, verbose=False, device=0)[0]
            inf_ms = r.speed["inference"]
            self.times[tag].append(inf_ms)

            # 取最高信心的偵測（拿來比一致性）
            n = len(r.boxes)
            self.dets[tag].append(n)
            if n > 0:
                top = r.boxes[int(r.boxes.conf.argmax())]
                cls = int(top.cls[0]); cf = float(top.conf[0])
                per_frame_best[tag] = (cls, round(cf, 2))
            else:
                per_frame_best[tag] = None
            line += f"{tag}={inf_ms:5.1f}ms({n}) "

        # 一致性檢查：三種格式的 top 類別是否相同
        cls_set = {v[0] if v else None for v in per_frame_best.values()}
        flag = "✓" if len(cls_set) == 1 else "✗不一致"
        print(line + flag)

        self.frame_i += 1
        if self.frame_i >= N_FRAMES:
            self.report()
            rclpy.shutdown()

    def report(self):
        print("\n" + "="*60)
        print("統計結果")
        print("="*60)
        print(f"{'格式':8s} {'平均(ms)':>10s} {'FPS':>8s} {'平均偵測數':>10s}")
        for tag in self.models:
            t = np.array(self.times[tag])
            d = np.array(self.dets[tag])
            avg = t.mean()
            print(f"{tag:8s} {avg:>10.2f} {1000/avg:>8.1f} {d.mean():>10.2f}")
        print("="*60)

        # 速度倍率（以 pt 為基準）
        if "pt" in self.times and "engine" in self.times:
            pt_avg = np.mean(self.times["pt"])
            en_avg = np.mean(self.times["engine"])
            print(f"\nengine 比 pt 快 {pt_avg/en_avg:.2f} 倍")


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