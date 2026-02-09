# ROS 2 Image Publisher and Subscriber

A ROS 2 Python project that publishes image frames from a camera (or synthetic source) and processes them in a subscriber by saving timestamped images and metadata.

---

## Overview

This project implements a simple ROS 2 image pipeline consisting of two Python nodes:

- **Image Publisher**  
  Publishes image frames continuously to a ROS 2 topic.  
  If a webcam is available, it is used automatically. Otherwise, synthetic images are generated.

- **Image Subscriber**  
  Subscribes to the image topic, overlays a timestamp on each frame, saves images to disk, and records metadata in a JSON file.

The project is intended to be easy to run and verify on **Windows, macOS, and Linux** using ROS 2 Jazzy Jalisco.

---

## Prerequisites

### Common (All Platforms)
- **Git**
- **ROS 2 Jazzy Jalisco**

> Python is not listed separately because it is installed as part of the ROS 2 setup on all supported platforms.

---

### Windows
- Windows 10 or newer
- **Pixi** package manager
- ROS 2 Jazzy (Windows binary ZIP)

---

### macOS
- macOS
- **Homebrew**
- ROS 2 Jazzy installed via Homebrew

---

### Linux
- Ubuntu 24.04 LTS (recommended)
- ROS 2 Jazzy installed via APT
- `v4l-utils`
- `ffmpeg`

---

## Project Setup

### Recommended Working Directory

The setup guides use a directory named **`work`** to keep commands consistent.

- Windows: `C:\work`
- macOS / Linux: `~/work`

If the directory does not exist, create it first.  
You may use a different location, but **all paths in the commands below must be updated accordingly**.

---

### Clone the Repository

#### Windows (Command Prompt)

```cmd
mkdir C:\work
cd C:\work
git clone https://github.com/VamsiU987/ros_camera.git
cd ros_camera
```

#### macOS / Linux

```bash
mkdir -p ~/work
cd ~/work
git clone https://github.com/VamsiU987/ros_camera.git
cd ros_camera
```

---

## Installing Dependencies

> **Important**  
> If you use different paths than shown below, replace them consistently in all commands.

---

### Windows (Pixi + ROS 2 Binary)

> Run the following steps in **PowerShell as Administrator**.

1. Install Pixi:
   ```powershell
   iwr -useb https://pixi.sh/install.ps1 | iex
   ```

2. Verify Pixi:
   ```powershell
   pixi --version
   ```

3. Create Pixi workspace and install dependencies:
   ```powershell
   mkdir C:\pixi_ws
   cd C:\pixi_ws
   irm https://raw.githubusercontent.com/ros2/ros2/refs/heads/jazzy/pixi.toml -OutFile pixi.toml
   pixi install
   ```

4. Download the ROS 2 Jazzy Windows binary ZIP and extract it into:
   ```text
   C:\pixi_ws\ros2-windows
   ```

5. Verify ROS 2 (use **cmd.exe**):
   ```cmd
   cd C:\pixi_ws
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

Source and verify ROS 2:
```bash
source /opt/homebrew/opt/ros-jazzy/setup.bash
echo "source /opt/homebrew/opt/ros-jazzy/setup.bash" >> ~/.zshrc
ros2 --help
echo $ROS_DISTRO
```

---

### Linux (Ubuntu)

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-opencv v4l-utils ffmpeg
```

Source and verify ROS 2:
```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
ros2 --help
echo $ROS_DISTRO
```

---

## Running the Application

> You must run the publisher and subscriber in **two separate terminals**.

---

### Windows

Open **two `cmd.exe` windows**.

**Terminal 1 — Publisher**
```cmd
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\work\ros_camera
set ROS_DOMAIN_ID=0
python publisher\image_publisher.py
```

**Terminal 2 — Subscriber**
```cmd
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\work\ros_camera
set ROS_DOMAIN_ID=0
python subscriber\image_subscriber.py
```

---

### macOS

Open **two Terminal windows**.

**Terminal 1 — Publisher**
```bash
source /opt/homebrew/opt/ros-jazzy/setup.bash
cd ~/work/ros_camera
export ROS_DOMAIN_ID=0
python3 publisher/image_publisher.py
```

**Terminal 2 — Subscriber**
```bash
source /opt/homebrew/opt/ros-jazzy/setup.bash
cd ~/work/ros_camera
export ROS_DOMAIN_ID=0
python3 subscriber/image_subscriber.py
```

---

### Linux

Open **two Terminal windows**.

**Terminal 1 — Publisher**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/work/ros_camera
export ROS_DOMAIN_ID=0
python3 publisher/image_publisher.py
```

**Terminal 2 — Subscriber**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/work/ros_camera
export ROS_DOMAIN_ID=0
python3 subscriber/image_subscriber.py
```

---

## Output

Outputs are written to the `output/` directory:

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

- `ROS_DOMAIN_ID` must match in both terminals.
- Synthetic images are used automatically if a webcam is not available.
- On macOS, grant camera access to Terminal under **System Settings → Privacy & Security → Camera**.
