#!/usr/bin/env python3
import json
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

TOPIC = "image_topic"
OUTPUT_DIR = "output/images"
METADATA_FILE = "output/metadata.json"

def human_ts(epoch: float) -> str:
    base = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(epoch))
    ms = int((epoch % 1) * 1000)
    return f"{base}.{ms:03d}"

def setup_output():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(os.path.dirname(METADATA_FILE), exist_ok=True)
    if not os.path.exists(METADATA_FILE):
        with open(METADATA_FILE, "w", encoding="utf-8") as f:
            json.dump([], f)

def append_metadata(record: dict):
    with open(METADATA_FILE, "r+", encoding="utf-8") as f:
        data = json.load(f)
        data.append(record)
        f.seek(0)
        json.dump(data, f, indent=2)
        f.truncate()

def to_bytes_ros(data) -> bytes:
    if isinstance(data, (bytes, bytearray, memoryview)):
        return bytes(data)
    if len(data) > 0 and isinstance(data[0], (bytes, bytearray, memoryview)):
        return b"".join(data)
    return bytes(data)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        setup_output()
        self.frame_id = 0
        self.create_subscription(ByteMultiArray, TOPIC, self.on_msg, 10)
        self.get_logger().info("Subscriber started")

    def on_msg(self, msg: ByteMultiArray):
        epoch = time.time()
        ts = human_ts(epoch)
        try:
            img_bytes = to_bytes_ros(msg.data)
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("image decode failed")
            cv2.putText(image, f"Timestamp: {ts}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            filename = f"image_{self.frame_id:06d}.jpg"
            path = os.path.join(OUTPUT_DIR, filename)
            if not cv2.imwrite(path, image):
                raise RuntimeError(f"image save failed: {path}")
            append_metadata({
                "saved_file_name": filename,
                "timestamp_used": ts,
                #"timestamp_epoch": epoch,
            })
            self.get_logger().info(f"Frame={self.frame_id} | Time={ts} | Status=Saved | File={path}")
        except Exception as e:
            self.get_logger().error(f"Frame={self.frame_id} | Time={ts} | Status=Error ({e})")
        finally:
            self.frame_id += 1

def main():
    rclpy.init()
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()