#!/usr/bin/env python3
# coding=utf-8

import json
import time
import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloDetectorNode(Node):
    """
    ROS image topic -> YOLO -> publish /yolo/objects

    Classes:
        ball       -> yellow
        blue-pole  -> blue

    Terminal:
        即時顯示目前看到的目標、信心度、中心座標、FPS
    """

    def __init__(self):
        super().__init__('yolo_detector_node')

        # =========================
        # Parameters
        # =========================

        self.declare_parameter(
            'model_path',
            '/home/iclab/ros2_kid/src/strategy/strategy/us/best.engine'
        )

        # ROS camera topic
        self.declare_parameter(
            'image_topic',
            '/camera1/image_raw'
        )

        self.declare_parameter('img_w', 320)
        self.declare_parameter('img_h', 240)

        # YOLO inference size
        self.declare_parameter('imgsz', 320)

        # confidence threshold
        self.declare_parameter('conf', 0.25)

        # 是否顯示 OpenCV 視窗
        self.declare_parameter('show_image', False)

        # =========================
        # Read parameters
        # =========================

        self.model_path = self.get_parameter(
            'model_path'
        ).value

        self.image_topic = self.get_parameter(
            'image_topic'
        ).value

        self.img_w = int(
            self.get_parameter('img_w').value
        )

        self.img_h = int(
            self.get_parameter('img_h').value
        )

        self.imgsz = int(
            self.get_parameter('imgsz').value
        )

        self.conf = float(
            self.get_parameter('conf').value
        )

        self.show_image = bool(
            self.get_parameter('show_image').value
        )

        # =========================
        # Publisher
        # =========================

        self.pub_objects = self.create_publisher(
            String,
            '/yolo/objects',
            10
        )

        # =========================
        # CvBridge
        # =========================

        self.bridge = CvBridge()

        self.latest_frame = None
        self.latest_image_time = 0.0

        # =========================
        # Load YOLO model
        # =========================

        self.get_logger().info(
            f'Loading YOLO model: {self.model_path}'
        )

        if self.model_path.endswith(
            ('.onnx', '.engine')
        ):
            self.model = YOLO(
                self.model_path,
                task='detect'
            )
        else:
            self.model = YOLO(
                self.model_path
            )

        self.get_logger().info(
            f'Model classes raw: {self.model.names}'
        )

        # =========================
        # Class name override
        # =========================
        #
        # TensorRT .engine 有時候 class 名稱
        # 會變成 class0 / class1
        #
        # 這裡強制：
        #
        # 0 = ball
        # 1 = blue-pole
        #

        self.class_name_override = {
            0: 'ball',
            1: 'blue-pole'
        }

        # =========================
        # ROS Image Subscriber
        # =========================

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Subscribed image topic: '
            f'{self.image_topic}'
        )

        self.get_logger().info(
            f'Output size='
            f'{self.img_w}x{self.img_h}, '
            f'imgsz={self.imgsz}, '
            f'conf={self.conf}'
        )

        # =========================
        # FPS
        # =========================

        self.frame_id = 0

        self.last_time = time.time()

        self.fps = 0.0

        # =========================
        # Timer
        # =========================
        #
        # 0.05 秒一次
        # 理論最大 20 Hz
        #
        # 實際 FPS 仍看 YOLO inference 速度
        #

        self.timer = self.create_timer(
            0.05,
            self.loop
        )

        self.get_logger().info(
            'YOLO detector started.'
        )

    # ==========================================================
    # Image callback
    # ==========================================================

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            self.latest_frame = frame

            self.latest_image_time = time.time()

        except Exception as e:

            self.get_logger().warn(
                f'cv_bridge convert failed: {e}'
            )

    # ==========================================================
    # Get class name
    # ==========================================================

    def get_class_name(self, cls_id):

        raw_name = str(
            self.model.names.get(
                cls_id,
                f'class{cls_id}'
            )
        )

        # 如果 TensorRT engine
        # 顯示 class0 / class1
        if raw_name.startswith('class'):

            return self.class_name_override.get(
                cls_id,
                raw_name
            )

        return raw_name

    # ==========================================================
    # Class -> color
    # ==========================================================

    def class_to_color(self, class_name: str) -> str:

        name = str(
            class_name
        ).lower().strip()

        # 足球
        if name == 'ball':
            return 'yellow'

        # 藍色球柱 / 障礙物
        if name in [
            'blue-pole',
            'blue_pole',
            'blue pole',
            'pole',
            'obstacle',
            'blue-obstacle',
            'blue_obstacle',
            'blue obstacle',
        ]:
            return 'blue'

        return name

    # ==========================================================
    # Bounding box -> object dictionary
    # ==========================================================

    def box_to_object(
        self,
        class_name,
        conf,
        xyxy
    ):

        x1, y1, x2, y2 = xyxy

        # 限制座標不要超出畫面
        x1 = int(
            max(
                0,
                min(
                    self.img_w - 1,
                    x1
                )
            )
        )

        y1 = int(
            max(
                0,
                min(
                    self.img_h - 1,
                    y1
                )
            )
        )

        x2 = int(
            max(
                0,
                min(
                    self.img_w - 1,
                    x2
                )
            )
        )

        y2 = int(
            max(
                0,
                min(
                    self.img_h - 1,
                    y2
                )
            )
        )

        # bbox width / height
        w = max(
            0,
            x2 - x1
        )

        h = max(
            0,
            y2 - y1
        )

        # 中心點
        cx = int(
            x1 + w / 2
        )

        cy = int(
            y1 + h / 2
        )

        # 面積
        area = int(
            w * h
        )

        # 長寬比
        aspect_ratio = (
            float(w / h)
            if h > 0
            else 0.0
        )

        return {
            'class': str(class_name),

            'color':
                self.class_to_color(
                    class_name
                ),

            'confidence':
                float(conf),

            'bbox': [
                x1,
                y1,
                w,
                h
            ],

            'centroid': [
                cx,
                cy
            ],

            'area':
                area,

            'aspect_ratio':
                aspect_ratio,
        }

    # ==========================================================
    # Main YOLO loop
    # ==========================================================

    def loop(self):

        # =========================
        # 尚未收到影像
        # =========================

        if self.latest_frame is None:

            self.get_logger().warn(
                f'Waiting image from '
                f'{self.image_topic} ...'
            )

            return

        # =========================
        # Copy latest image
        # =========================

        frame = self.latest_frame.copy()

        # resize 成固定 320x240
        frame = cv2.resize(
            frame,
            (
                self.img_w,
                self.img_h
            )
        )

        # =========================
        # FPS
        # =========================

        now = time.time()

        dt = (
            now -
            self.last_time
        )

        self.last_time = now

        if dt > 0:

            self.fps = (
                1.0 / dt
            )

        # =========================
        # YOLO inference
        # =========================

        results = self.model.predict(
            frame,
            imgsz=self.imgsz,
            conf=self.conf,
            verbose=False,
        )

        # =========================
        # Parse objects
        # =========================

        objects = []

        if (
            len(results) > 0
            and results[0].boxes is not None
        ):

            for box in results[0].boxes:

                cls_id = int(
                    box.cls[0]
                )

                conf = float(
                    box.conf[0]
                )

                class_name = (
                    self.get_class_name(
                        cls_id
                    )
                )

                xyxy = (
                    box.xyxy[0]
                    .tolist()
                )

                obj = (
                    self.box_to_object(
                        class_name,
                        conf,
                        xyxy
                    )
                )

                objects.append(
                    obj
                )

        # =========================
        # Publish /yolo/objects
        # =========================

        msg = String()

        msg.data = json.dumps(
            {
                'frame_id':
                    self.frame_id,

                'fps':
                    round(
                        self.fps,
                        2
                    ),

                'image_width':
                    self.img_w,

                'image_height':
                    self.img_h,

                'objects':
                    objects,
            },
            ensure_ascii=False
        )

        self.pub_objects.publish(
            msg
        )

        # =========================
        # 即時 Terminal 顯示
        # =========================
        #
        # 每一次 YOLO inference
        # 都會執行
        #

        self.print_terminal(
            objects
        )

        # =========================
        # Show OpenCV image
        # =========================

        if self.show_image:

            annotated = (
                results[0].plot()
            )

            cv2.imshow(
                'YOLO ball + obstacle',
                annotated
            )

            if (
                cv2.waitKey(1)
                & 0xFF
                == ord('q')
            ):

                self.get_logger().info(
                    'Pressed q, shutdown'
                )

                rclpy.shutdown()

        self.frame_id += 1

    # ==========================================================
    # Terminal output
    # ==========================================================

    def print_terminal(self, objects):
        if not objects:
            self.get_logger().info('[YOLO] NONE')
            return

        text = '[YOLO] '

        for obj in objects:
            text += (
                f'{obj["class"]} '
                f'{obj["confidence"] * 100:.1f}% | '
            )

        self.get_logger().info(text)
        # ==========================================================
        # Destroy node
        # ==========================================================

        def destroy_node(self):

            try:

                cv2.destroyAllWindows()

            except Exception:

                pass

            super().destroy_node()


# ==============================================================
# Main
# ==============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        YoloDetectorNode()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()