# 3D Ultrasound Reconstruction & Data Pipeline (ROS) Year: 2022

> **Project Context:** This repository contains the data engineering pipeline from a university robotics project. 
> **Goal:** Create a 3D volume reconstruction from a 2D ultrasound probe mounted on a robotic arm and meassure distance needle to blood vessel.
>

## Tech Stack
* **Middleware:** ROS (Robot Operating System)
* **Languages:** Python (NumPy, SciPy)
* **Computer Vision:** OpenCV, Open3D
* **Data Formats:** `.npy` (Binary), ROS messages

## My Contributions

### 1. Data Ingestion & Synchronization (`CommunicationManager.py`)
This module handles the real-time data acquisition limitation where image and position data arrived asynchronously.
* Implemented a handshake mechanism via ROS topics (`/trigger_send_position`).
* Ensured **temporal alignment** of ultrasound images with the robotic arm's coordinate system.
* Serialized the synchronized datasets into binary NumPy files for efficient storage.

### 2. Data Processing & Segmentation (`ImageProcessor.py`)
Preprocessing of the noisy ultrasound raw data using computer vision techniques.
* **Noise Reduction:** Applied Gaussian blur and Sobel operators.
* **Segmentation:** Developed a custom ellipse-filter to detect needle cross-sections and vessels in the image data.
* **Optical Flow:** Experimented with Lucas-Kanade for motion tracking.

### 3. 3D Volume Reconstruction (`VolumeCreator.py`)
The core data product: converting 2D slices into a metric 3D point cloud.
* **Coordinate Transformation:** Mapped 2D pixel data to 3D table coordinates using the probe's physical dimensions.
* **Data Quality Filter:** Implemented logic to detect and discard scanlines with "jitter" or positional shifts (`filter_position_data`) to ensure a clean 3D model.
* **Visualization:** Generated Point Clouds using Open3D for distance measurements.

## Data Structure
The pipeline processes raw data stored as synchronized NumPy arrays:
* `_images.npy`: Array of 2D raw pixel data.
* `_positions.npy`: Corresponding x, y, z coordinates of the robotic effector.

This code is provided "as is" from the research prototype phase. Paths to local datasets (e.g., `D:\data`) are hardcoded and would be parameterized in a production environment