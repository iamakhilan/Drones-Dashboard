# 🛰️ AeroVIO Ground Station — Monocular Visual-Inertial Odometry & 3D Mapping

A modern, high-precision SaaS-style ground control interface designed for autonomous drones equipped with Monocular Visual-Inertial Odometry (VIO), real-time 3D spatial reconstruction, and ROS 2 computational graph telemetry.

![Dashboard Preview](stitch_screen_5f02e90f849a4a9b90ee2d2da12b516a.png)

---

## ⚡ Features

- **Keyframe Summary & Filter KPI Cards**: Harris-FAST corner detector metrics (248 pts), estimation rate (60 Hz), sliding window MSCKF clones (11/15), and displacement tracking.
- **Monocular Optical Feed Viewport**: Real-time KLT tracker with 248 keypoint overlays, optical flow vectors, pinhole + Brown-Conrady lens model parameters, and FOV diagnostics.
- **3D Spatial Reconstruction & Pose**: Synthetic 3D isometric SLAM grid projection, global NED origin triad, camera frustums along trajectory splines, and coordinate HUD overlay.
- **Precision Telemetry Matrix**: Global NED position (North, East, Down), linear kinematics with ADIS-16470 IMU accelerations, and SO(3) quaternion attitude/angular rates.
- **Analytics & Diagnostics Charts**: Clean line charts showing trajectory drift vs. RTK baseline and pre-integration innovation error σ (Ax/Ay/Az noise).
- **ROS 2 Computational Pipeline**: Live worker thread table monitoring camera drivers, feature tracking, IMU publishers, and MSCKF estimator nodes.

---

## 🚀 Quick Start

1. Clone this repository:
   ```bash
   git clone https://github.com/iamakhilan/Drones-Dashboard.git
   cd Drones-Dashboard
   ```

2. Open `index.html` in any modern web browser or serve it locally:
   ```bash
   # Using Python built-in HTTP server:
   python -m http.server 8000
   ```
   Then navigate to `http://localhost:8000` in your browser.

---

## 🛠️ Tech Stack

- **UI / Styling**: HTML5, Tailwind CSS with Forms & Container Plugins, SF Mono / Inter fonts
- **Visual Design**: Generated via Google Stitch (Screen ID: `5f02e90f849a4a9b90ee2d2da12b516a`)
