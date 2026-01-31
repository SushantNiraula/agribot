import os
import time
import json
import uuid
import subprocess
import requests

import rclpy
from rclpy.node import Node


class CaptureAndUpload(Node):
    """
    Captures JPEG using libcamera-still and uploads to server via HTTP.
    Includes metadata: robot_id, ts, field_id, odom (optional later).
    """

    def __init__(self):
        super().__init__("capture_and_upload")

        self.declare_parameter("server_url", "http://10.35.23.117:5000")
        self.declare_parameter("robot_id", "agribot-01")
        self.declare_parameter("field_id", "field-01")
        self.declare_parameter("interval_sec", 1.5)
        self.declare_parameter("save_dir", "/tmp/agribot_frames")
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)

        self.server_url = str(self.get_parameter("server_url").value).rstrip("/")
        self.robot_id = str(self.get_parameter("robot_id").value)
        self.field_id = str(self.get_parameter("field_id").value)
        self.interval = float(self.get_parameter("interval_sec").value)
        self.save_dir = str(self.get_parameter("save_dir").value)
        self.quality = int(self.get_parameter("jpeg_quality").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.timer = self.create_timer(self.interval, self.tick)

        self.get_logger().info(
            f"Capture+Upload started → {self.server_url} every {self.interval}s "
            f"({self.width}x{self.height}, q={self.quality})"
        )

    def _capture_jpeg(self, out_path: str) -> bool:
        # libcamera-still is stable, but slow if you fully warm up every time.
        # Use short timeout and no preview for speed.
        cmd = [
            "libcamera-still",
            "-n",  # no preview
            "-t", "1",  # minimal delay
            "--width", str(self.width),
            "--height", str(self.height),
            "-q", str(self.quality),
            "-o", out_path,
        ]
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception as e:
            self.get_logger().warn(f"Capture failed: {e}")
            return False

    def _upload(self, image_path: str, meta: dict) -> bool:
        url = f"{self.server_url}/api/inspection/upload"
        print(f"Uploading to {url} ...")
        try:
            with open(image_path, "rb") as f:
                files = {"image": (os.path.basename(image_path), f, "image/jpeg")}
                data = {"meta": json.dumps(meta)}
                r = requests.post(url, files=files, data=data, timeout=10)
            if r.status_code != 200:
                self.get_logger().warn(f"Upload failed {r.status_code}: {r.text[:120]}")
                return False
            return True
        except Exception as e:
            self.get_logger().warn(f"Upload exception: {e}")
            return False

    def tick(self):
        ts = time.time()
        frame_id = str(uuid.uuid4())
        out_path = os.path.join(self.save_dir, f"{frame_id}.jpg")

        ok = self._capture_jpeg(out_path)
        if not ok:
            return

        meta = {
            "ts": float(ts),
            "robot_id": self.robot_id,
            "field_id": self.field_id,
            "frame_id": frame_id,
            # optional: attach odom later (x,y,yaw)
            "odom": None,
        }

        uploaded = self._upload(out_path, meta)
        if uploaded:
            self.get_logger().info(f"Uploaded frame={frame_id} ts={ts:.2f}")
            # optional: delete local after successful upload
            try:
                os.remove(out_path)
            except Exception:
                pass


def main():
    rclpy.init()
    node = CaptureAndUpload()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
