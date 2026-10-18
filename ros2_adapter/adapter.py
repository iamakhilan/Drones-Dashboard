#!/usr/bin/env python3
"""
===============================================================================
AeroVIO Ground Station — ROS 2 Live Telemetry Adapter
===============================================================================
Real rclpy node that subscribes to the ROS 2 topic contracts documented in
README.md ("ROS 2 Topic Contracts"), normalizes them into the exact telemetry
JSON schema broadcast by bridge_server.py, and forwards frames to the bridge
via POST /ingest.

Subscribed topics (defaults; override with --topic-* flags):
  /drones_controller/position_error           geometry_msgs/Vector3Stamped
  /drones_controller/velocity_error           geometry_msgs/Vector3Stamped
  /drones_controller/desired_force            geometry_msgs/Vector3Stamped
  /drones_controller/desired_attitude         geometry_msgs/PoseStamped
  /drones_controller/attitude_error           geometry_msgs/Vector3Stamped
  /drones_controller/angular_velocity_error   geometry_msgs/Vector3Stamped
  /drones_controller/desired_moment           geometry_msgs/Vector3Stamped
  /drones_controller/fsm_state                std_msgs/String
  /drone/vio/odometry                         nav_msgs/Odometry      (T_WB)
  /drone/vio/camera_pose                      geometry_msgs/PoseStamped (T_WC)
  /imu/data                                   sensor_msgs/Imu

Frame schema pushed to POST /ingest (matches bridge_server.py LIVE payload):
  {
    "type": "telemetry",
    "data": {
      "pose_wb": {"position": {"x","y","z"}, "orientation": {"qx","qy","qz","qw"}},
      "pose_wc": {...},                # optional; bridge computes T_WC=T_WB*T_BC if absent
      "twist":  {"linear": {"vx","vy","vz"}, "angular": {"p","q","r"}},
      "imu":    {"accel": {"ax","ay","az"}, "gyro": {"gx","gy","gz"}},
      "controller": {
        "position_error": {"x","y","z","norm"},
        "velocity_error": {"x","y","z","norm"},
        "desired_force":  {"x","y","z","norm"},
        "attitude_generator": {"desired_attitude": {"qx","qy","qz","qw"}},
        "so3_controller":   {"attitude_error": {"x","y","z","norm"},
                             "angular_velocity_error": {"x","y","z","norm"},
                             "desired_moment": {"x","y","z","norm"}},
        "fsm_state": "NAVIGATING", ...
      }
    }
  }
===============================================================================
"""

import argparse
import json
import math
import sys
import time
import urllib.request

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Vector3Stamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


def _vec(v):
    norm = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    return {"x": round(v.x, 5), "y": round(v.y, 5), "z": round(v.z, 5), "norm": round(norm, 5)}


if HAS_RCLPY:
    class AeroVIOAdapter(Node):
        """ROS 2 node normalizing drones-controller topics into bridge telemetry frames."""

        def __init__(self, topics):
            super().__init__("aerovio_ros2_adapter")
            self.latest = {
                "pose_wb": None,
                "pose_wc": None,
                "twist": None,
                "imu": None,
                "controller": {},
            }
            self._last_pose_time = 0.0

            ap_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

            # Odometry & direct PoseStamped subscriptions
            self.create_subscription(Odometry, topics["odom"], self._on_odom, 10)
            self.create_subscription(PoseStamped, topics.get("pose", "/ap/v1/pose/filtered"), self._on_pose, ap_qos)
            self.create_subscription(TwistStamped, topics.get("twist", "/ap/v1/twist/filtered"), self._on_twist, ap_qos)
            self.create_subscription(PoseStamped, topics["camera_pose"], self._on_camera_pose, 10)

            # IMU subscriptions
            self.create_subscription(Imu, topics["imu"], self._on_imu, 10)
            self.create_subscription(Imu, "/ap/v1/imu/experimental/data", self._on_imu, ap_qos)
            self.create_subscription(Vector3Stamped, topics["pos_err"],
                                     lambda m: self._set_ctrl("position_error", _vec(m.vector)), 10)
            self.create_subscription(Vector3Stamped, topics["vel_err"],
                                     lambda m: self._set_ctrl("velocity_error", _vec(m.vector)), 10)
            self.create_subscription(Vector3Stamped, topics["force"],
                                     lambda m: self._set_ctrl("desired_force", _vec(m.vector)), 10)
            self.create_subscription(Vector3Stamped, topics["moment"],
                                     lambda m: self._set_so3("desired_moment", _vec(m.vector)), 10)
            self.create_subscription(Vector3Stamped, topics["att_err"],
                                     lambda m: self._set_so3("attitude_error", _vec(m.vector)), 10)
            self.create_subscription(Vector3Stamped, topics["ang_vel_err"],
                                     lambda m: self._set_so3("angular_velocity_error", _vec(m.vector)), 10)
            self.create_subscription(PoseStamped, topics["desired_attitude"], self._on_desired_attitude, 10)
            self.create_subscription(String, topics["fsm_state"],
                                     lambda m: self._set_ctrl("fsm_state", m.data), 10)

            self.get_logger().info("AeroVIO ROS 2 adapter subscriptions active.")

        def _set_ctrl(self, key, value):
            self.latest["controller"][key] = value

        def _set_so3(self, key, value):
            self.latest["controller"].setdefault("so3_controller", {})[key] = value

        def _on_odom(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            lv = msg.twist.twist.linear
            av = msg.twist.twist.angular
            self.latest["pose_wb"] = {
                "position": {"x": round(p.x, 4), "y": round(p.y, 4), "z": round(p.z, 4)},
                "orientation": {"qx": round(q.x, 4), "qy": round(q.y, 4),
                                "qz": round(q.z, 4), "qw": round(q.w, 4)},
            }
            self.latest["twist"] = {
                "linear": {"vx": round(lv.x, 4), "vy": round(lv.y, 4), "vz": round(lv.z, 4)},
                "angular": {"p": round(av.x, 4), "q": round(av.y, 4), "r": round(av.z, 4)},
            }
            self._last_pose_time = time.time()

        def _on_pose(self, msg: PoseStamped):
            p = msg.pose.position
            q = msg.pose.orientation
            self.latest["pose_wb"] = {
                "position": {"x": round(p.x, 4), "y": round(p.y, 4), "z": round(p.z, 4)},
                "orientation": {"qx": round(q.x, 4), "qy": round(q.y, 4),
                                "qz": round(q.z, 4), "qw": round(q.w, 4)},
            }
            self._last_pose_time = time.time()

        def _on_twist(self, msg: TwistStamped):
            lv = msg.twist.linear
            av = msg.twist.angular
            self.latest["twist"] = {
                "linear": {"vx": round(lv.x, 4), "vy": round(lv.y, 4), "vz": round(lv.z, 4)},
                "angular": {"p": round(av.x, 4), "q": round(av.y, 4), "r": round(av.z, 4)},
            }

        def _on_camera_pose(self, msg):
            p = msg.pose.position
            q = msg.pose.orientation
            self.latest["pose_wc"] = {
                "position": {"x": round(p.x, 4), "y": round(p.y, 4), "z": round(p.z, 4)},
                "orientation": {"qx": round(q.x, 4), "qy": round(q.y, 4),
                                "qz": round(q.z, 4), "qw": round(q.w, 4)},
            }

        def _on_desired_attitude(self, msg):
            q = msg.pose.orientation
            self.latest["controller"].setdefault("attitude_generator", {})["desired_attitude"] = {
                "qx": round(q.x, 4), "qy": round(q.y, 4),
                "qz": round(q.z, 4), "qw": round(q.w, 4),
            }

        def _on_imu(self, msg):
            a = msg.linear_acceleration
            w = msg.angular_velocity
            self.latest["imu"] = {
                "accel": {"ax": round(a.x, 3), "ay": round(a.y, 3), "az": round(a.z, 3)},
                "gyro": {"gx": round(w.x, 4), "gy": round(w.y, 4), "gz": round(w.z, 4)},
            }

        def build_frame(self):
            """Assemble one normalized telemetry frame; None until a pose has arrived."""
            if self.latest["pose_wb"] is None:
                return None
            data = {
                "pose_wb": self.latest["pose_wb"],
                "twist": self.latest["twist"] or {
                    "linear": {"vx": 0, "vy": 0, "vz": 0},
                    "angular": {"p": 0, "q": 0, "r": 0},
                },
                "imu": self.latest["imu"] or {
                    "accel": {"ax": 0, "ay": 0, "az": 9.81},
                    "gyro": {"gx": 0, "gy": 0, "gz": 0},
                },
                "controller": dict(self.latest["controller"]),
            }
            if self.latest["pose_wc"] is not None:
                data["pose_wc"] = self.latest["pose_wc"]
            return {"type": "telemetry", "source": "ros2_adapter", "data": data}


def post_frame(endpoint, frame):
    """POST one telemetry frame to the bridge /ingest endpoint."""
    body = json.dumps(frame).encode("utf-8")
    req = urllib.request.Request(
        endpoint, data=body,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as resp:
        return resp.status == 200


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="AeroVIO ROS 2 live telemetry adapter (rclpy -> bridge POST /ingest)")
    parser.add_argument("--endpoint", default="http://127.0.0.1:8000/ingest",
                        help="Bridge ingest URL (default: http://127.0.0.1:8000/ingest)")
    parser.add_argument("--rate", type=float, default=25.0,
                        help="Forward rate cap in Hz (default: 25)")
    parser.add_argument("--topic-odom", default="/drone/vio/odometry",
                        help="nav_msgs/Odometry body odometry topic (T_WB)")
    parser.add_argument("--topic-pose", default="/ap/v1/pose/filtered",
                        help="geometry_msgs/PoseStamped body pose topic")
    parser.add_argument("--topic-twist", default="/ap/v1/twist/filtered",
                        help="geometry_msgs/TwistStamped body twist topic")
    parser.add_argument("--topic-camera-pose", default="/drone/vio/camera_pose",
                        help="geometry_msgs/PoseStamped camera optical pose topic (T_WC)")
    parser.add_argument("--topic-imu", default="/imu/data",
                        help="sensor_msgs/Imu topic")
    parser.add_argument("--topic-pos-err", default="/drones_controller/position_error")
    parser.add_argument("--topic-vel-err", default="/drones_controller/velocity_error")
    parser.add_argument("--topic-force", default="/drones_controller/desired_force")
    parser.add_argument("--topic-desired-attitude", default="/drones_controller/desired_attitude")
    parser.add_argument("--topic-att-err", default="/drones_controller/attitude_error")
    parser.add_argument("--topic-ang-vel-err", default="/drones_controller/angular_velocity_error")
    parser.add_argument("--topic-moment", default="/drones_controller/desired_moment")
    parser.add_argument("--topic-fsm-state", default="/drones_controller/fsm_state")
    args = parser.parse_args(argv)

    if not HAS_RCLPY:
        print("[Adapter] ERROR: rclpy is not available. Source your ROS 2 environment "
              "(e.g. 'source /opt/ros/humble/setup.bash') and retry.", file=sys.stderr)
        return 2

    topics = {
        "odom": args.topic_odom,
        "pose": args.topic_pose,
        "twist": args.topic_twist,
        "camera_pose": args.topic_camera_pose,
        "imu": args.topic_imu,
        "pos_err": args.topic_pos_err,
        "vel_err": args.topic_vel_err,
        "force": args.topic_force,
        "desired_attitude": args.topic_desired_attitude,
        "att_err": args.topic_att_err,
        "ang_vel_err": args.topic_ang_vel_err,
        "moment": args.topic_moment,
        "fsm_state": args.topic_fsm_state,
    }

    rclpy.init()
    node = AeroVIOAdapter(topics)
    min_interval = 1.0 / max(args.rate, 0.1)
    last_forward = 0.0

    print(f"[Adapter] Forwarding normalized telemetry to {args.endpoint} @ <= {args.rate} Hz")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.time()
            if now - last_forward < min_interval:
                continue
            frame = node.build_frame()
            if frame is None:
                continue
            try:
                post_frame(args.endpoint, frame)
                last_forward = now
            except Exception as e:
                print(f"[Adapter] Bridge unreachable at {args.endpoint}: {e}", file=sys.stderr)
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
