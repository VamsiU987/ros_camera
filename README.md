# ROS 2 Image Publisher and Subscriber

A ROS 2 Python project that publishes image frames from a camera (or synthetic source) and processes them in a subscriber by saving timestamped images and metadata.

---

## Overview

This project consists of two ROS 2 Python nodes:

- **Image Publisher**  
  Publishes image frames continuously to a ROS 2 topic.  
  If a webcam is available, it is used automatically. Otherwise, synthetic images are generated.

- **Image Subscriber**  
  Subscribes to the image topic, overlays a timestamp on each frame, saves images to disk, and records metadata in a JSON file.

The project is designed to run on **Windows, macOS, and Linux** using ROS 2 Jazzy Jalisco.

---

## Prerequisites

### Common (All Platforms)
- Python 3
- Git
- ROS 2 Jazzy Jalisco

### Windows
- Windows 10 or newer
- Pixi package manager
- ROS 2 Jazzy (Windows binary ZIP)

### macOS
- macOS with Homebrew installed
- ROS 2 Jazzy installed via Homebrew

### Linux
- Ubuntu 24.04 LTS (recommended)
- ROS 2 Jazzy installed via APT
- `v4l-utils` and `ffmpeg`

---

## Project Setup

Clone the repository:

```bash
git clone https://github.com/VamsiU987/ros_camera.git
cd ros_camera
```

Expected structure:

```text
publisher/
subscriber/
output/
README.md
```

---

## Installing Dependencies

### Windows (Pixi + ROS 2 Binary)

```cmd
iwr -useb https://pixi.sh/install.ps1 | iex
```

```cmd
mkdir C:\pixi_ws
cd C:\pixi_ws
irm https://raw.githubusercontent.com/ros2/ros2/refs/heads/jazzy/pixi.toml -OutFile pixi.toml
pixi install
```

Download the ROS 2 Jazzy Windows binary ZIP and extract it into:

```text
C:\pixi_ws\ros2-windows
```

Verify:
```cmd
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
ros2 -h
```

---

### macOS

```bash
brew update
brew install ros/jazzy/ros-jazzy-desktop
```

```bash
source /opt/homebrew/opt/ros-jazzy/setup.bash
echo "source /opt/homebrew/opt/ros-jazzy/setup.bash" >> ~/.zshrc
ros2 --help
```

---

### Linux (Ubuntu)

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-opencv v4l-utils ffmpeg
```

```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
ros2 --help
```

---

## Configuration

No configuration files are required.

Ensure:
- `ROS_DOMAIN_ID` is the same for publisher and subscriber.
- A webcam is optional; synthetic images are used if unavailable.

---

## Running the Application

Two terminals are required.

### Windows

**Publisher**
```cmd
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\work\ros_camera
set ROS_DOMAIN_ID=0
python publisher\image_publisher.py
```

**Subscriber**
```cmd
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\work\ros_camera
set ROS_DOMAIN_ID=0
python subscriber\image_subscriber.py
```

---

### macOS

**Publisher**
```bash
source /opt/homebrew/opt/ros-jazzy/setup.bash
cd ~/work/ros_camera
export ROS_DOMAIN_ID=0
python3 publisher/image_publisher.py
```

**Subscriber**
```bash
source /opt/homebrew/opt/ros-jazzy/setup.bash
cd ~/work/ros_camera
export ROS_DOMAIN_ID=0
python3 subscriber/image_subscriber.py
```

---

### Linux

**Publisher**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/work/ros_camera
export ROS_DOMAIN_ID=0
python3 publisher/image_publisher.py
```

**Subscriber**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/work/ros_camera
export ROS_DOMAIN_ID=0
python3 subscriber/image_subscriber.py
```

---

## Output

Outputs are written to:

- `output/images/` — saved images with timestamp overlay
- `output/metadata.json` — metadata for each image

---

## Troubleshooting

### VirtualBox Camera Issue (Linux VM)

If the webcam does not initialize correctly:

```bash
ffplay -loglevel quiet -t 1 /dev/video0
```

Then restart the publisher.

---

## Notes

- Image data is transmitted using `std_msgs/ByteMultiArray`.
- Synthetic images are used automatically when a webcam is not available.
- External webcams are recommended on macOS.
