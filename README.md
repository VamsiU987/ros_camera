# ROS 2 Image Publisher–Subscriber System

A ROS 2–based image streaming project where a publisher node sends image frames and a subscriber node processes and saves them with timestamp metadata.

---

## Overview

This project implements a small distributed system using **ROS 2** and **Python**:

- **Publisher node**  
  Captures images from a webcam (or generates synthetic images if the camera is unavailable) and publishes them continuously over a ROS 2 topic.

- **Subscriber node**  
  Subscribes to the image stream, overlays a timestamp on each frame, saves the processed images to disk, and records metadata in a JSON file.

The project is designed to be easy to run, inspect, and verify, following the requirements of the provided task documentation.

---

## Prerequisites

Before cloning the project, ensure the following are available on your system:

### Common (All Platforms)
- **Python 3**
- **ROS 2** (Foxy Fitzroy or newer)
- **Git**

### Linux
- `ffmpeg` (used for webcam initialization in some VM environments)

### macOS
- Homebrew package manager

### Windows
- Windows 10 or newer
- ROS 2 installed via the official Windows installer
- Command Prompt or PowerShell

---

## Project Setup

Clone the repository:

```bash
git clone <repository-url>
cd <repository-name>
```

Ensure that the ROS 2 environment is sourced before running any commands.

---

## Installing Dependencies

### Linux (Ubuntu)

```bash
sudo apt update
sudo apt install python3-pip ffmpeg -y
pip install opencv-python numpy
```

### macOS

Using Homebrew:

```bash
brew install ffmpeg
pip3 install opencv-python numpy
```

### Windows

Install Python dependencies:

```bat
pip install opencv-python numpy
```

Ensure that the ROS 2 installation directory is sourced in the terminal before running the project.

---

## Configuration

No configuration files or environment variables are required.

Make sure:
- The ROS 2 environment is sourced in every terminal.
- A webcam is available (optional; synthetic images are used as a fallback).

---

## Running the Application

You must run the publisher and subscriber in **separate terminals**.

### Linux / macOS

**Terminal 1 – Publisher**
```bash
python3 publisher/image_publisher.py
```

**Terminal 2 – Subscriber**
```bash
python3 subscriber/image_subscriber.py
```

### Windows

**Terminal 1 – Publisher**
```bat
python publisher\image_publisher.py
```

**Terminal 2 – Subscriber**
```bat
python subscriber\image_subscriber.py
```

After starting both nodes, images will be published, processed, and saved automatically.

---

## Output

All output files are written to the `output/` directory.

### Images
Saved processed images:
```
output/images/image_000000.jpg
output/images/image_000001.jpg
...
```

Each image includes a timestamp overlay.

### Metadata
Metadata file:
```
output/metadata.json
```

Each entry contains:
- Saved file name
- Timestamp used on the image
- Epoch timestamp

Both nodes log status information (frame number, timestamps, success or errors) to the console.

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

### Linux / VirtualBox Webcam Issue

In some Linux virtual machine setups, the webcam may not initialize correctly for OpenCV even though it works in applications like *Cheese*.

Run the following command **once** to initialize the camera:

```bash
ffplay -loglevel quiet -t 1 /dev/video0
```

Then restart the publisher node.

To avoid repeating this manually, you may create a shell alias.

### No Webcam Available

If no webcam is detected, the publisher automatically switches to synthetic image generation. The rest of the pipeline continues to work normally.

---

## Notes

- Image data is transmitted using `std_msgs/ByteMultiArray` with JPEG-encoded frames.
- The subscriber includes robust handling for different ROS Python message representations, ensuring compatibility across Linux, macOS, and Windows.
- The project is intended as a clear and simple demonstration of ROS 2 inter-node communication and image processing.
