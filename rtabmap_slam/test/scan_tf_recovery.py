"""Probe first-map recovery when the scan's static transform arrives late."""

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def run(late_tf: bool) -> int:
    rclpy.init()
    node = rclpy.create_node("startup_input_probe")
    best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    cloud_pub = node.create_publisher(PointCloud2, "/scan_cloud", best)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    static = StaticTransformBroadcaster(node)
    dynamic = TransformBroadcaster(node)
    received = []

    def on_map(msg: OccupancyGrid) -> None:
        if msg.info.width and msg.info.height:
            received.append([msg.info.width, msg.info.height])

    node.create_subscription(
        OccupancyGrid,
        "/map",
        on_map,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    parameters = {
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "subscribe_depth": "false",
        "subscribe_rgb": "false",
        "subscribe_scan_cloud": "true",
        "subscribe_odom": "true",
        "approx_sync": "true",
        "qos_scan": "2",
        "qos_odom": "1",
        "wait_for_transform": "0.2",
        "database_path": "",
        "Reg/Strategy": "1",
        "Grid/3D": "true",
        "Grid/CellSize": "0.1",
        "Grid/NormalsSegmentation": "false",
        "Grid/MinGroundHeight": "-0.3",
        "Grid/MaxGroundHeight": "0.1",
        "Grid/RangeMax": "20.0",
        "Grid/RangeMin": "0.1",
    }
    args = [
        str(Path(get_package_prefix("rtabmap_slam")) / "lib/rtabmap_slam/rtabmap"),
        "--ros-args",
        "--log-level",
        "warn",
    ]
    parameters = {
        key: (value if "/" in key or key == "database_path" else yaml.safe_load(value))
        for key, value in parameters.items()
    }
    Path("/tmp/rtab-startup-params.yaml").write_text(
        yaml.safe_dump({"/**": {"ros__parameters": parameters}})
    )
    args += ["--params-file", "/tmp/rtab-startup-params.yaml"]
    log_path = Path("/tmp/rtab-startup-probe.log")
    with log_path.open("w") as log:
        process = subprocess.Popen(
            args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        start = time.monotonic()
        static_sent = False
        cloud_sent = False
        try:
            while time.monotonic() - start < 18 and not received:
                elapsed = time.monotonic() - start
                stamp = node.get_clock().now().to_msg()
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = "odom"
                transform.child_frame_id = "base_link"
                transform.transform.rotation.w = 1.0
                dynamic.sendTransform(transform)
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.orientation.w = 1.0
                for i in [0, 7, 14, 21, 28, 35]:
                    odom.pose.covariance[i] = 0.001
                odom_pub.publish(odom)
                if not static_sent and (not late_tf or elapsed > 8):
                    transform.header.frame_id = "base_link"
                    transform.child_frame_id = "lidar"
                    transform.transform.translation.z = 0.2
                    static.sendTransform(transform)
                    static_sent = True
                if elapsed > 4 and cloud_pub.get_subscription_count() > 0:
                    header = Header(stamp=stamp, frame_id="lidar")
                    points = [
                        (x / 10.0, y / 10.0, -0.2)
                        for x in range(5, 41, 2)
                        for y in range(-20, 21, 2)
                    ]
                    points += [
                        (4.0, y / 10.0, z / 10.0)
                        for y in range(-20, 21, 2)
                        for z in range(0, 21, 2)
                    ]
                    cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
                    cloud_sent = True
                rclpy.spin_once(node, timeout_sec=0.05)
                if process.poll() is not None:
                    break
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
    content = log_path.read_text()
    result = {
        "lateTf": late_tf,
        "cloudSent": cloud_sent,
        "map": received[:1],
        "conversionFailure": "Could not convert 3d laser scan msg" in content,
        "processExit": process.returncode,
    }
    print(json.dumps(result), flush=True)
    passed = (
        bool(received)
        and process.returncode == 0
        and (not late_tf or result["conversionFailure"])
    )
    if not passed:
        print(content[-5000:], flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--late-tf", action="store_true")
    args = parser.parse_args()
    raise SystemExit(run(args.late_tf))
