# ROS 2 Image Publisher–Subscriber System

A simple ROS 2 project that publishes image frames from a camera (or synthetic source) and processes them in a subscriber node by overlaying timestamps and saving results to disk.

---

## Overview

This project implements a small distributed system using ROS 2 with two Python nodes:

- **Publisher node**: Continuously captures images from a webcam (or generates synthetic images if the camera is unavailable) and publishes them over a ROS 2 topic.
- **Subscriber node**: Receives the images, overlays a timestamp, saves the processed images to disk, and records metadata for each saved image.

The system demonstrates basic inter-node communication, image processing with OpenCV, and file output handling.

---

## Prerequisites

- **ROS 2** (Foxy Fitzroy or newer) installed and sourced  
- **Python 3**
- **OpenCV for Python**
- **NumPy**
- **ffmpeg** (required only for a known VirtualBox webcam initialization workaround on Linux VMs)

---

## Project Setup

Clone the repository:

```bash
git clone <repository-url>
cd <repository-name>
```

Ensure your ROS 2 environment is sourced before proceeding.

---

## Installing Dependencies

Install Python dependencies:

```bash
pip install opencv-python numpy
```

On Ubuntu systems (or Linux VMs), install `ffmpeg` if not already present:

```bash
sudo apt update
sudo apt install ffmpeg
```

---

## Configuration

No configuration files or environment variables are required.

Ensure that:
- Your ROS 2 environment is sourced.
- The camera device is accessible (if using a webcam).

---

## Running the Application

Open **two terminals**, each with the ROS 2 environment sourced.

### Start the Publisher

```bash
python publisher/image_publisher.py
```

This starts publishing image frames to the ROS 2 topic.

### Start the Subscriber

```bash
python subscriber/image_subscriber.py
```

This subscribes to the image stream, processes each frame, and saves outputs to disk.

---

## Output

All outputs are saved under the `output/` directory:

- **Images**:  
  ```
  output/images/image_000000.jpg
  output/images/image_000001.jpg
  ...
  ```
  Each image includes a timestamp overlay.

- **Metadata**:  
  ```
  output/metadata.json
  ```
  Contains an entry for each saved image, including:
  - Saved file name
  - Timestamp used on the image
  - Epoch timestamp

Console logs from both nodes indicate publishing/subscribing status, frame numbers, timestamps, and any errors.

---

## Project Structure

```text
.
├── publisher/
│   └── image_publisher.py
├── subscriber/
│   └── image_subscriber.py
├── output/
│   ├── images/
│   └── metadata.json
└── README.md
```

---

## Troubleshooting

### Camera works in Cheese but not in the publisher (Linux VM)

On some Linux VirtualBox setups, the webcam may not initialize correctly for OpenCV-based applications.

Run this once to initialize the camera:

```bash
ffplay -loglevel quiet -t 1 /dev/video0
```

Then re-run the publisher.

### No camera available

If the webcam cannot be opened, the publisher automatically falls back to generating synthetic images.

---

## Notes

- This project uses `std_msgs/ByteMultiArray` to transmit JPEG-encoded image data.
- The subscriber includes robust handling to support different ROS Python data representations, particularly on Windows and virtualized environments.
- The system is intended as a simple demonstration and is not optimized for high-throughput or large-scale deployments.
