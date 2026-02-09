#!/usr/bin/env python3
from datetime import datetime
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

TOPIC = "image_topic"
FPS = 30

def ts_now() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.pub = self.create_publisher(ByteMultiArray, TOPIC, 10)
        self.timer = self.create_timer(1.0 / FPS, self.publish_once)
        self.frame_id = 0
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().warn("Webcam not found. Using synthetic images.")
            self.cap = None
        self.get_logger().info("Publishing started")

    def _synthetic(self, text: str) -> np.ndarray:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, text, (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        return img

    def _get_frame(self) -> np.ndarray:
        if self.cap is None:
            return self._synthetic("Synthetic (no camera)")
        ok, frame = self.cap.read()
        if ok and frame is not None:
            return frame
        self.get_logger().warn("Camera read failed. Using synthetic frame.")
        return self._synthetic("Synthetic (camera busy)")

    def publish_once(self):
        ts = ts_now()
        try:
            frame = self._get_frame()
            ok, buf = cv2.imencode(".jpg", frame)
            if not ok:
                raise RuntimeError("JPEG encode failed")
            msg = ByteMultiArray()
            msg.data = buf.tobytes()
            self.pub.publish(msg)
            self.get_logger().info(f"Frame={self.frame_id} | Time={ts} | Status=Sent")
        except Exception as e:
            self.get_logger().error(f"Frame={self.frame_id} | Time={ts} | Status=Error ({e})")
        finally:
            self.frame_id += 1

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
