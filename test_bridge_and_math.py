#!/usr/bin/env python3
"""
Test suite validating mathematical parity, frame conventions, and interface contracts
between the AeroVIO ground station bridge server and the reference drone implementation
(https://github.com/Ravish2807/Drones_S5_CD2_3D_Mapping_using_normal_monocular_camera_controlling_Visual-Inertial-Odometry-VIO/tree/drones-controller).
"""

import unittest
import numpy as np

# Import bridge server controller and transform functions and parameters
from bridge_server import (
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    vee,
    hat,
    compute_cascaded_controller,
    compute_camera_optical_pose,
    VEHICLE_MASS,
    GRAVITY,
    INERTIA_J,
    KP_POS,
    KV_POS,
    KR_ATT,
    KOMEGA_RATE,
    MAX_VELOCITY,
    POSITION_TOLERANCE,
    VELOCITY_TOLERANCE,
    P_BC,
    R_BC,
)


class TestReferenceControllerMath(unittest.TestCase):
    """
    Validates exact cascaded position and SO(3) controller equations
    against drones_controller/position_controller.py and so3_controller.py.
    """

    def test_hover_equilibrium(self):
        """
        At desired position p = p_d with v = 0, yaw = 2.0:
        - Position error e_p = 0
        - Velocity error e_v = 0
        - Desired force F_d = m * g * e3 = [0, 0, 1.5 * 9.81] = [0, 0, 14.715] N
        - Target reached flag is True
        - Commanded velocity is zeroed because target is reached
        """
        p_d = [20.0, 5.0, 20.0]
        p = [20.0, 5.0, 20.0]
        v = [0.0, 0.0, 0.0]
        q_hover = [0.0, 0.0, 0.0, 1.0]

        out = compute_cascaded_controller(p=p, v=v, q_xyzw=q_hover, omega=[0.0, 0.0, 0.0], p_d=p_d, v_d=[0, 0, 0], yaw_d=0.0)

        self.assertEqual(out["position_error"]["norm"], 0.0)
        self.assertEqual(out["velocity_error"]["norm"], 0.0)
        self.assertTrue(out["target_reached"])
        self.assertAlmostEqual(out["desired_force"]["norm"], 1.5 * 9.81, places=3)
        self.assertAlmostEqual(out["desired_force"]["z"], 14.715, places=3)
        self.assertEqual(out["command_velocity"]["speed"], 0.0)
        self.assertEqual(out["so3_controller"]["attitude_error"]["norm"], 0.0)
        self.assertEqual(out["so3_controller"]["desired_moment"]["norm"], 0.0)

    def test_tracking_error_sign(self):
        """
        Verify that tracking error is strictly e_p = p_d - p and e_v = v_d - v
        (as defined in drones_controller/position_controller.py line 55).
        """
        p = [10.0, 2.0, 5.0]
        p_d = [12.0, 5.0, 6.0]
        v = [0.1, -0.2, 0.0]
        v_d = [0.0, 0.0, 0.0]

        out = compute_cascaded_controller(p=p, v=v, q_xyzw=[0, 0, 0, 1], p_d=p_d, v_d=v_d, yaw_d=0.0)

        self.assertAlmostEqual(out["position_error"]["x"], 2.0)
        self.assertAlmostEqual(out["position_error"]["y"], 3.0)
        self.assertAlmostEqual(out["position_error"]["z"], 1.0)
        self.assertAlmostEqual(out["velocity_error"]["x"], -0.1)
        self.assertAlmostEqual(out["velocity_error"]["y"], 0.2)
        self.assertAlmostEqual(out["velocity_error"]["z"], 0.0)

    def test_velocity_command_saturation(self):
        """
        Verify saturation of commanded velocity to MAX_VELOCITY = 0.5 m/s
        (as in test_position_controller.py test_velocity_command_saturation).
        """
        p = [0.0, 0.0, 0.0]
        p_d = [5.0, 0.0, 0.0]  # Raw command = Kp * ep = 0.5 * 5.0 = 2.5 m/s > 0.5 m/s
        v = [0.0, 0.0, 0.0]

        out = compute_cascaded_controller(p=p, v=v, q_xyzw=[0, 0, 0, 1], p_d=p_d, v_d=[0, 0, 0], yaw_d=0.0)

        self.assertFalse(out["target_reached"])
        self.assertAlmostEqual(out["command_velocity"]["speed"], 0.5, places=4)
        self.assertAlmostEqual(out["command_velocity"]["vx"], 0.5, places=4)
        self.assertAlmostEqual(out["command_velocity"]["vy"], 0.0, places=4)
        self.assertAlmostEqual(out["command_velocity"]["vz"], 0.0, places=4)

    def test_attitude_generator_orthonormality(self):
        """
        Verify that AttitudeGenerator generates a valid SO(3) rotation matrix:
        R_d^T R_d = I and det(R_d) = +1.
        """
        p = [0.0, 0.0, 0.0]
        p_d = [5.0, -3.0, 10.0]
        v = [0.0, 0.0, 0.0]
        yaw_d = 1.25

        out = compute_cascaded_controller(p=p, v=v, q_xyzw=[0, 0, 0, 1], p_d=p_d, yaw_d=yaw_d)
        qd = out["attitude_generator"]["desired_attitude"]
        R_d = quaternion_to_rotation_matrix(qd["qx"], qd["qy"], qd["qz"], qd["qw"])

        # Check SO(3) properties
        np.testing.assert_allclose(R_d.T @ R_d, np.eye(3), atol=1e-6)
        self.assertAlmostEqual(np.linalg.det(R_d), 1.0, places=5)

    def test_so3_geometric_error_and_moment(self):
        """
        Verify SO(3) attitude error e_R = 1/2*(R_d^T R - R^T R_d)^\vee
        and moment M_d opposes attitude and rate errors.
        """
        # Yaw setpoint 0, but drone is tilted by 10 degrees roll
        roll = np.deg2rad(10.0)
        c, s = np.cos(roll), np.sin(roll)
        R_tilt = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        q_tilt = rotation_matrix_to_quaternion(R_tilt)

        p = [20.0, 5.0, 20.0]
        out = compute_cascaded_controller(p=p, v=[0, 0, 0], q_xyzw=q_tilt, omega=[0.1, 0.0, 0.0], p_d=p, yaw_d=0.0)

        # Attitude error norm must be greater than zero
        self.assertGreater(out["so3_controller"]["attitude_error"]["norm"], 0.0)
        # Desired moment x should be negative to counteract positive roll error and positive roll rate
        self.assertLess(out["so3_controller"]["desired_moment"]["x"], 0.0)


class TestCoordinateTransforms(unittest.TestCase):
    """
    Validates extrinsic and frame transformations against
    tests/test_transforms.py from the reference VIO implementation:
    T_WC = T_WB @ T_BC <=> p_WC = p_WB + R_WB @ p_BC, R_WC = R_WB @ R_BC
    """

    def test_pure_translation_forward(self):
        """
        Drone position p_WB = [1.0, 2.0, 3.0]
        Identity rotation R_WB = Eye(3)
        p_BC = [0.1, 0.0, 0.0]
        Expected p_WC = [1.1, 2.0, 3.0]
        """
        p_wb = np.array([1.0, 2.0, 3.0])
        q_wb = np.array([0.0, 0.0, 0.0, 1.0])

        p_wc, q_wc = compute_camera_optical_pose(p_wb, q_wb)

        np.testing.assert_allclose(p_wc, [1.1, 2.0, 3.0], atol=1e-10)

    def test_rotated_drone_translation(self):
        """
        Drone rotated 90 degrees around Z-axis (Yaw = +90 deg):
        R_WB faces +Y. Since camera is +0.10m in +X_body,
        in world frame it must be at [0.0, 0.1, 1.0].
        """
        # Yaw 90 deg quaternion: qz = sin(45 deg), qw = cos(45 deg)
        qz = np.sin(np.pi / 4)
        qw = np.cos(np.pi / 4)
        q_wb = np.array([0.0, 0.0, qz, qw])
        p_wb = np.array([0.0, 0.0, 1.0])

        p_wc, q_wc = compute_camera_optical_pose(p_wb, q_wb)

        np.testing.assert_allclose(p_wc, [0.0, 0.1, 1.0], atol=1e-10)

    def test_opencv_camera_frame_extrinsics(self):
        """
        Check that R_BC matches OpenCV optical convention:
        +Z optical = +X body (forward)
        +X optical = -Y body (right)
        +Y optical = -Z body (down)
        """
        np.testing.assert_allclose(P_BC, [0.10, 0.0, 0.0])
        np.testing.assert_allclose(R_BC, [
            [0.0,  0.0,  1.0],
            [-1.0, 0.0,  0.0],
            [0.0, -1.0,  0.0]
        ])

        # Test T_WC = T_WB @ T_BC 4x4 matrix equivalence
        p_wb = np.array([4.0, 0.0, 2.0])
        q_wb = np.array([0.0, 0.0, 0.0, 1.0])
        R_wb = quaternion_to_rotation_matrix(q_wb[0], q_wb[1], q_wb[2], q_wb[3])

        p_wc, q_wc = compute_camera_optical_pose(p_wb, q_wb)
        R_wc = quaternion_to_rotation_matrix(q_wc[0], q_wc[1], q_wc[2], q_wc[3])

        T_WB = np.eye(4)
        T_WB[:3, :3] = R_wb
        T_WB[:3, 3] = p_wb

        T_BC = np.eye(4)
        T_BC[:3, :3] = R_BC
        T_BC[:3, 3] = P_BC

        T_WC = T_WB @ T_BC
        np.testing.assert_allclose(T_WC[:3, 3], p_wc, atol=1e-10)
        np.testing.assert_allclose(T_WC[:3, :3], R_wc, atol=1e-10)


class TestSystemParametersAgainstConfig(unittest.TestCase):
    """
    Verifies that system parameters in bridge_server strictly match
    config/vehicle.yaml and config/controller.yaml in the reference repo.
    """

    def test_vehicle_parameters(self):
        self.assertEqual(VEHICLE_MASS, 1.5)
        self.assertEqual(GRAVITY, 9.81)
        np.testing.assert_allclose(INERTIA_J, [0.02, 0.02, 0.04])

    def test_controller_gains_and_tolerances(self):
        np.testing.assert_allclose(KP_POS, [0.5, 0.5, 0.5])
        np.testing.assert_allclose(KV_POS, [0.2, 0.2, 0.2])
        np.testing.assert_allclose(KR_ATT, [4.0, 4.0, 2.0])
        np.testing.assert_allclose(KOMEGA_RATE, [0.8, 0.8, 0.5])
        self.assertEqual(MAX_VELOCITY, 0.5)
        self.assertEqual(POSITION_TOLERANCE, 0.10)
        self.assertEqual(VELOCITY_TOLERANCE, 0.05)


class TestBridgeHttpServer(unittest.TestCase):
    """
    Spawns AeroVIOHandler on a local ephemeral port and verifies
    HTTP REST API contracts and truthful ROS 2 status.
    """

    @classmethod
    def setUpClass(cls):
        import socketserver
        import threading
        from bridge_server import AeroVIOHandler

        class EphemeralServer(socketserver.ThreadingTCPServer):
            allow_reuse_address = True

        cls.httpd = EphemeralServer(('127.0.0.1', 0), AeroVIOHandler)
        cls.port = cls.httpd.server_address[1]
        cls.server_thread = threading.Thread(target=cls.httpd.serve_forever, daemon=True)
        cls.server_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.server_close()

    def test_status_endpoint(self):
        import urllib.request
        import json

        url = f"http://127.0.0.1:{self.port}/api/status"
        with urllib.request.urlopen(url, timeout=2.0) as resp:
            self.assertEqual(resp.status, 200)
            data = json.loads(resp.read().decode('utf-8'))
            self.assertEqual(data["server"], "AeroVIO Ground Station Bridge")
            self.assertIn("mode", data)
            # Must truthfully report ros2_connected as False on Windows host without active ROS 2
            self.assertFalse(data["ros2_connected"])
            self.assertIn("ROS 2", data["ros2_detail"])

    def test_health_endpoint(self):
        import urllib.request
        import json

        url = f"http://127.0.0.1:{self.port}/api/health"
        with urllib.request.urlopen(url, timeout=2.0) as resp:
            self.assertEqual(resp.status, 200)
            data = json.loads(resp.read().decode('utf-8'))
            self.assertEqual(data["server"], "AeroVIO Ground Station Bridge")
            self.assertEqual(data["vehicle_parameters"]["mass_kg"], 1.5)
            self.assertEqual(data["vehicle_parameters"]["kp"], [0.5, 0.5, 0.5])
            self.assertEqual(data["vehicle_parameters"]["kv"], [0.2, 0.2, 0.2])
            self.assertEqual(data["vehicle_parameters"]["kr"], [4.0, 4.0, 2.0])
            self.assertEqual(data["vehicle_parameters"]["komega"], [0.8, 0.8, 0.5])

    def test_command_endpoint(self):
        import urllib.request
        import json

        url = f"http://127.0.0.1:{self.port}/api/command"
        payload = json.dumps({"target_p": [15.0, 2.0, 10.0], "target_yaw": 1.57}).encode('utf-8')
        req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            self.assertEqual(resp.status, 200)
            data = json.loads(resp.read().decode('utf-8'))
            self.assertEqual(data["status"], "success")
    def test_pointcloud_endpoint(self):
        import urllib.request
        import json

        url = f"http://127.0.0.1:{self.port}/api/pointcloud"
        with urllib.request.urlopen(url, timeout=2.0) as resp:
            self.assertEqual(resp.status, 200)
            data = json.loads(resp.read().decode('utf-8'))
            self.assertIsInstance(data, list)
            # The PCD has 442 genuine landmarks
            self.assertEqual(len(data), 442)
            self.assertEqual(len(data[0]), 3)

    def test_camera_status_endpoint(self):
        import urllib.request
        import json

        url = f"http://127.0.0.1:{self.port}/api/camera_status"
        with urllib.request.urlopen(url, timeout=2.0) as resp:
            self.assertEqual(resp.status, 200)
            data = json.loads(resp.read().decode('utf-8'))
            self.assertEqual(data["topic"], "/camera/image_raw")
            self.assertEqual(data["encoding"], "mono8")
            self.assertTrue(data["demo_image_available"])
            self.assertEqual(data["demo_image_url"], "/monocular_tracking_feed.png")


if __name__ == '__main__':
    unittest.main()
