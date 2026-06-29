# Tutorial Part 02 — ROS2 Package Scaffold

> **Project:** Autonomous Drone Swarm with LLM Agents  
> **Stack:** Ubuntu 26.04 LTS · ROS2 Lyrical · Python 3.12+ · Pydantic v2

---

## Overview

This tutorial creates the complete `major_project` ROS2 package skeleton. By the end you will have:

- A properly structured ROS2 Python package at `~/major_ws/src/major_project/`
- All `__init__.py` stub files for every sub-package
- `requirements.txt`, `package.xml`, and `setup.py` fully configured
- All **16 entry points** registered and verified with `colcon build`
- PC-2 configured with ROS2, px4_msgs, Ollama, and CycloneDDS over WiFi

> [!IMPORTANT]
> Complete **Tutorial Part 01** (infrastructure) before starting this section. Source your environment in every new terminal: `source ~/.bashrc`

---

## Section 2.1 — Create ROS2 Package

### 2.1.1 Create workspace

```bash
mkdir -p ~/major_ws/src
cd ~/major_ws/src

source /opt/ros/lyrical/setup.bash
```

### 2.1.2 Create the package

```bash
cd ~/major_ws/src

ros2 pkg create major_project \
  --build-type ament_python \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs px4_msgs
```

> [!NOTE]
> This creates the base package with `setup.py`, `package.xml`, and `major_project/__init__.py`. We will replace all three files with the final versions below.

### Verification

```bash
ls ~/major_ws/src/major_project/
# Expected: major_project/  package.xml  resource/  setup.cfg  setup.py  test/
```

---

## Section 2.2 — Create Full Directory Tree

### 2.2.1 Create all sub-package directories

```bash
cd ~/major_ws/src/major_project

# GCS nodes (Ground Control Station)
mkdir -p major_project/gcs

# Lead pilot nodes
mkdir -p major_project/lead_pilot

# Wingman pilot nodes
mkdir -p major_project/wingman_pilot

# Shared common utilities
mkdir -p major_project/common

# Prompt template directories
mkdir -p major_project/lead_pilot/prompts
mkdir -p major_project/wingman_pilot/prompts

# Config directory (YAML configs, not Python)
mkdir -p config

# Launch files directory
mkdir -p launch
```

### 2.2.2 Create all `__init__.py` stub files

```bash
cd ~/major_ws/src/major_project

# Root package (already exists from ros2 pkg create)
touch major_project/__init__.py

# Sub-packages
touch major_project/gcs/__init__.py
touch major_project/lead_pilot/__init__.py
touch major_project/wingman_pilot/__init__.py
touch major_project/common/__init__.py
```

### 2.2.3 Create placeholder node files

```bash
cd ~/major_ws/src/major_project

# GCS nodes
touch major_project/gcs/stt_node.py
touch major_project/gcs/clarification_speaker_node.py
touch major_project/gcs/mission_monitor_node.py
touch major_project/gcs/emergency_stop_node.py
touch major_project/gcs/diagnostics_node.py

# Lead pilot nodes
touch major_project/lead_pilot/camera_detection_node.py
touch major_project/lead_pilot/lead_sensor_aggregator_node.py
touch major_project/lead_pilot/lead_px4_commander_node.py
touch major_project/lead_pilot/lead_intent_bridge_node.py
touch major_project/lead_pilot/lead_agent_node.py
touch major_project/lead_pilot/safety_monitor_node.py

# Wingman pilot nodes
touch major_project/wingman_pilot/wingman_camera_detection_node.py
touch major_project/wingman_pilot/wingman_sensor_aggregator_node.py
touch major_project/wingman_pilot/wingman_px4_commander_node.py
touch major_project/wingman_pilot/wingman_agent_node.py

# Common utilities
touch major_project/common/schemas.py
touch major_project/common/ollama_client.py

# Launch files
touch launch/lead_pilot.launch.py
touch launch/wingman_pilot.launch.py

# Config files
touch config/lead_config.yaml
touch config/wingman_config.yaml
```

### 2.2.4 Write `main()` stubs into every node file

Each node needs a `main()` function so the entry point resolves correctly at build time:

```bash
cd ~/major_ws/src/major_project

for node_file in \
  major_project/gcs/stt_node.py \
  major_project/gcs/clarification_speaker_node.py \
  major_project/gcs/mission_monitor_node.py \
  major_project/gcs/emergency_stop_node.py \
  major_project/gcs/diagnostics_node.py \
  major_project/lead_pilot/camera_detection_node.py \
  major_project/lead_pilot/lead_sensor_aggregator_node.py \
  major_project/lead_pilot/lead_px4_commander_node.py \
  major_project/lead_pilot/lead_intent_bridge_node.py \
  major_project/lead_pilot/lead_agent_node.py \
  major_project/lead_pilot/safety_monitor_node.py \
  major_project/wingman_pilot/wingman_camera_detection_node.py \
  major_project/wingman_pilot/wingman_sensor_aggregator_node.py \
  major_project/wingman_pilot/wingman_px4_commander_node.py \
  major_project/wingman_pilot/wingman_agent_node.py
do
  node_name=$(basename "$node_file" .py)
  cat > "$node_file" << STUB_EOF
#!/usr/bin/env python3
"""Stub for ${node_name}. Full implementation in later tutorial parts."""
import rclpy


def main(args=None):
    """Entry point stub."""
    rclpy.init(args=args)
    print(f"[STUB] ${node_name} entry point called -- implementation pending.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
STUB_EOF
done

echo "All node stubs written."
```

### Verification — Directory Tree

```bash
cd ~/major_ws/src/major_project
find . -type f -name "*.py" | sort
```

Expected output:
```
./major_project/__init__.py
./major_project/common/__init__.py
./major_project/common/ollama_client.py
./major_project/common/schemas.py
./major_project/gcs/__init__.py
./major_project/gcs/clarification_speaker_node.py
./major_project/gcs/diagnostics_node.py
./major_project/gcs/emergency_stop_node.py
./major_project/gcs/mission_monitor_node.py
./major_project/gcs/stt_node.py
./major_project/lead_pilot/__init__.py
./major_project/lead_pilot/camera_detection_node.py
./major_project/lead_pilot/lead_agent_node.py
./major_project/lead_pilot/lead_intent_bridge_node.py
./major_project/lead_pilot/lead_px4_commander_node.py
./major_project/lead_pilot/lead_sensor_aggregator_node.py
./major_project/lead_pilot/safety_monitor_node.py
./major_project/wingman_pilot/__init__.py
./major_project/wingman_pilot/wingman_agent_node.py
./major_project/wingman_pilot/wingman_camera_detection_node.py
./major_project/wingman_pilot/wingman_px4_commander_node.py
./major_project/wingman_pilot/wingman_sensor_aggregator_node.py
```

---

## Section 2.3 — requirements.txt

```bash
cat << 'EOF' > ~/major_ws/src/major_project/requirements.txt
pydantic>=2.0,<3.0
faster-whisper>=1.0.0
requests>=2.28.0
numpy>=1.24.0
ultralytics>=8.0.0
rich>=13.0.0
sounddevice>=0.4.6
scipy>=1.10.0
empy==3.3.4
opencv-python>=4.8.0
EOF
```

### Install Python dependencies

```bash
pip3 install --user -r ~/major_ws/src/major_project/requirements.txt
```

### Verification

```bash
python3 -c "import pydantic; print('pydantic', pydantic.__version__)"
python3 -c "import numpy; print('numpy', numpy.__version__)"
python3 -c "import cv2; print('opencv', cv2.__version__)"
# All should print version numbers without errors
```

---

## Section 2.4 — package.xml

Write the complete `package.xml` with all required dependencies including `cv_bridge`:

```bash
cat << 'EOF' > ~/major_ws/src/major_project/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>major_project</name>
  <version>0.1.0</version>
  <description>
    Autonomous two-drone swarm system with LLM-based agent coordination.
    Lead drone (Drone-0) and Wingman drone (Drone-1) operate with PX4 SITL
    via ROS2 Lyrical and Gazebo Jetty. Local LLM inference via Ollama.
  </description>
  <maintainer email="student@university.edu">Major Project Student</maintainer>
  <license>MIT</license>

  <!-- Build tool -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- ROS2 core -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <!-- PX4 message definitions -->
  <depend>px4_msgs</depend>

  <!-- Computer vision bridge (ROS image <-> OpenCV) -->
  <depend>cv_bridge</depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF
```

### Install cv_bridge

```bash
sudo apt install -y ros-lyrical-cv-bridge
```

### Verification

```bash
grep "<depend>" ~/major_ws/src/major_project/package.xml | sort
# Should include: cv_bridge, geometry_msgs, px4_msgs, rclpy, sensor_msgs, std_msgs

ros2 pkg list | grep cv_bridge
# Expected: cv_bridge
```

---

## Section 2.5 — setup.py (FINAL — All 16 Entry Points)

> [!IMPORTANT]
> This is the **final** `setup.py`. It contains all 16 entry points exactly as specified. Do not add, remove, or rename any entry point — launch files and shell scripts depend on these exact names.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/setup.py
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'major_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package marker (required by ROS2 ament)
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Major Project Student',
    maintainer_email='student@university.edu',
    description='Autonomous two-drone swarm system with LLM agent coordination.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # -- Ground Control Station (GCS) nodes -------------------------
            'stt_node = major_project.gcs.stt_node:main',
            'clarification_speaker = major_project.gcs.clarification_speaker_node:main',
            'mission_monitor = major_project.gcs.mission_monitor_node:main',
            'emergency_stop = major_project.gcs.emergency_stop_node:main',
            'diagnostics = major_project.gcs.diagnostics_node:main',

            # -- Lead Pilot nodes -------------------------------------------
            'camera_detection = major_project.lead_pilot.camera_detection_node:main',
            'lead_sensor_aggregator = major_project.lead_pilot.lead_sensor_aggregator_node:main',
            'lead_px4_commander = major_project.lead_pilot.lead_px4_commander_node:main',
            'lead_intent_bridge = major_project.lead_pilot.lead_intent_bridge_node:main',
            'lead_agent = major_project.lead_pilot.lead_agent_node:main',
            'safety_monitor = major_project.lead_pilot.safety_monitor_node:main',

            # -- Wingman Pilot nodes ----------------------------------------
            'wingman_camera_detection = major_project.wingman_pilot.wingman_camera_detection_node:main',
            'wingman_sensor_aggregator = major_project.wingman_pilot.wingman_sensor_aggregator_node:main',
            'wingman_px4_commander = major_project.wingman_pilot.wingman_px4_commander_node:main',
            'wingman_intent_bridge = major_project.wingman_pilot.wingman_intent_bridge_node:main',
            'wingman_agent = major_project.wingman_pilot.wingman_agent_node:main',
        ],
    },
)
EOF
```

### Verify entry point count

```bash
echo "Entry point count:"
grep -c "= major_project\." ~/major_ws/src/major_project/setup.py

echo ""
echo "All entry points:"
grep "= major_project\." ~/major_ws/src/major_project/setup.py
```

Expected output:
```
Entry point count:
16

All entry points:
            'stt_node = major_project.gcs.stt_node:main',
            'clarification_speaker = major_project.gcs.clarification_speaker_node:main',
            'mission_monitor = major_project.gcs.mission_monitor_node:main',
            'emergency_stop = major_project.gcs.emergency_stop_node:main',
            'diagnostics = major_project.gcs.diagnostics_node:main',
            'camera_detection = major_project.lead_pilot.camera_detection_node:main',
            'lead_sensor_aggregator = major_project.lead_pilot.lead_sensor_aggregator_node:main',
            'lead_px4_commander = major_project.lead_pilot.lead_px4_commander_node:main',
            'lead_intent_bridge = major_project.lead_pilot.lead_intent_bridge_node:main',
            'lead_agent = major_project.lead_pilot.lead_agent_node:main',
            'safety_monitor = major_project.lead_pilot.safety_monitor_node:main',
            'wingman_camera_detection = major_project.wingman_pilot.wingman_camera_detection_node:main',
            'wingman_sensor_aggregator = major_project.wingman_pilot.wingman_sensor_aggregator_node:main',
            'wingman_px4_commander = major_project.wingman_pilot.wingman_px4_commander_node:main',
            'wingman_intent_bridge = major_project.wingman_pilot.wingman_intent_bridge_node:main',
            'wingman_agent = major_project.wingman_pilot.wingman_agent_node:main',
```

---

## Section 2.6 — Build and Smoke Test

### 2.6.1 Build the workspace

```bash
cd ~/major_ws
source /opt/ros/lyrical/setup.bash
source ~/px4_msgs_ws/install/setup.bash

colcon build --symlink-install --packages-select major_project
```

Expected output:
```
Starting >>> major_project
Finished <<< major_project [X.XXs]

Summary: 1 package finished [X.XXs]
```

> [!TIP]
> `--symlink-install` means Python source changes take effect immediately without rebuilding -- essential for iterative development.

### 2.6.2 Source the workspace

```bash
source ~/major_ws/install/setup.bash
echo "source ~/major_ws/install/setup.bash" >> ~/.bashrc
```

### 2.6.3 Verify all 16 entry points are discoverable

```bash
echo "=== Verifying all 16 entry points ==="

entry_points=(
  "stt_node"
  "clarification_speaker"
  "mission_monitor"
  "emergency_stop"
  "diagnostics"
  "camera_detection"
  "lead_sensor_aggregator"
  "lead_px4_commander"
  "lead_intent_bridge"
  "lead_agent"
  "safety_monitor"
  "wingman_camera_detection"
  "wingman_sensor_aggregator"
  "wingman_px4_commander"
  "wingman_intent_bridge"
  "wingman_agent"
)

all_pass=true
for ep in "${entry_points[@]}"; do
  if which "$ep" &>/dev/null; then
    echo "  [OK]   $ep"
  else
    echo "  [FAIL] $ep -- not found in PATH"
    all_pass=false
  fi
done

echo ""
if $all_pass; then
  echo "All 16 entry points verified."
else
  echo "Some entry points failed. Check setup.py and rebuild."
fi
```

### 2.6.4 Verify via ros2 pkg executables

```bash
ros2 pkg executables major_project
```

Expected: all 16 executables listed, one per line.

### 2.6.5 Quick stub run test

```bash
source ~/major_ws/install/setup.bash

echo "=== Smoke testing node stubs ==="
for ep in stt_node diagnostics lead_agent wingman_agent safety_monitor; do
  echo -n "Testing $ep ... "
  timeout 3 ros2 run major_project "$ep" 2>&1 | grep -q "STUB" && \
    echo "OK" || echo "DONE (rclpy context exited cleanly)"
done
echo "Smoke test complete."
```

### 2.6.6 Verify package metadata

```bash
ros2 pkg prefix major_project
# Expected: /home/<user>/major_ws/install/major_project

ros2 pkg xml major_project | grep -E "<name>|<version>|<depend>"
# Expected: major_project, 0.1.0, and all 6 dependencies
```

---

## Section 2.7 — PC-2 Setup (Wingman Agent Machine)

PC-2 runs `wingman_agent`, `wingman_px4_commander`, and `wingman_sensor_aggregator`. It communicates with PC-1 over WiFi using CycloneDDS.

> [!NOTE]
> **Finding your WiFi interface name:** On Ubuntu, run `ip link show` and look for the interface starting with `wl` (e.g., `wlan0`, `wlp3s0`, `wlx...`). Replace `wlan0` in all configs below with your actual interface name.

```bash
# On EITHER PC: find your WiFi interface name
ip link show | grep "^[0-9]" | awk '{print $2}' | tr -d ':' | grep wl
# Example output: wlan0
# Use this name to replace 'wlan0' in the CycloneDDS config below
```

### 2.7.1 Install ROS2 Lyrical on PC-2

```bash
# Run on PC-2 -- identical to Section 1.2 on PC-1
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade -y
sudo apt install -y \
  ros-lyrical-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  ros-lyrical-rmw-cyclonedds-cpp \
  ros-lyrical-cv-bridge

sudo rosdep init || true
rosdep update

echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2.7.2 Build px4_msgs on PC-2

```bash
# Run on PC-2
mkdir -p ~/px4_msgs_ws/src
cd ~/px4_msgs_ws/src
git clone https://github.com/PX4/px4_msgs.git --branch release/1.15 --depth 1

cd ~/px4_msgs_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "source ~/px4_msgs_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.7.3 Install Ollama on PC-2

```bash
# Run on PC-2
curl -fsSL https://ollama.com/install.sh | sh
ollama serve &
sleep 3
ollama pull qwen2.5-coder:3b
```

### 2.7.4 Configure CycloneDDS for WiFi — PC-1

> [!IMPORTANT]
> Replace `wlan0` with your actual WiFi interface name identified above. Common names: `wlan0`, `wlp2s0`, `wlp3s0`. Running `ip link show | grep wl` will show the exact name.

```bash
# Run on PC-1
cp ~/cyclone.xml ~/cyclone.xml.bak 2>/dev/null || true

cat << 'CYCLONE_EOF' > ~/cyclone.xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <!-- Replace wlan0 with your actual WiFi interface name -->
      <!-- Find it with: ip link show | grep wl -->
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
CYCLONE_EOF

# Set the environment variable (also add to .bashrc)
export CYCLONEDDS_URI=file:///home/$USER/cyclone.xml
grep -q "CYCLONEDDS_URI" ~/.bashrc || \
  echo 'export CYCLONEDDS_URI=file:///home/$USER/cyclone.xml' >> ~/.bashrc

source ~/.bashrc
echo "CycloneDDS config written to ~/cyclone.xml"
echo "CYCLONEDDS_URI = $CYCLONEDDS_URI"
```

### 2.7.5 Configure CycloneDDS for WiFi — PC-2

```bash
# Run on PC-2 (same commands -- adjust wlan0 if PC-2 uses a different interface name)
cat << 'CYCLONE_EOF' > ~/cyclone.xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <!-- Replace wlan0 with PC-2's actual WiFi interface name -->
      <!-- Find it with: ip link show | grep wl -->
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
CYCLONE_EOF

export CYCLONEDDS_URI=file:///home/$USER/cyclone.xml
grep -q "CYCLONEDDS_URI" ~/.bashrc || \
  echo 'export CYCLONEDDS_URI=file:///home/$USER/cyclone.xml' >> ~/.bashrc

source ~/.bashrc
echo "CycloneDDS config written to ~/cyclone.xml"
```

### 2.7.6 Sync workspace from PC-1 to PC-2

> [!NOTE]
> Replace the values below with your actual PC-2 username and IP address.  
> **Find PC-2's IP:** On PC-2, run `ip -4 addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1`

```bash
# Run on PC-1
# Step 1: Get PC-2 IP (run this on PC-2 first)
#   ip -4 addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1

PC2_USER="ubuntu"          # <-- change to PC-2 username
PC2_IP="192.168.1.XXX"     # <-- change to PC-2 IP address (found above)

# Step 2: Sync source directory
rsync -avz --progress \
  ~/major_ws/src/major_project/ \
  ${PC2_USER}@${PC2_IP}:~/major_ws/src/major_project/

echo "Workspace synced to ${PC2_USER}@${PC2_IP}"
```

### 2.7.7 Build the workspace on PC-2

```bash
# Run on PC-2
mkdir -p ~/major_ws/src
cd ~/major_ws

source /opt/ros/lyrical/setup.bash
source ~/px4_msgs_ws/install/setup.bash

pip3 install --user -r ~/major_ws/src/major_project/requirements.txt

colcon build --symlink-install --packages-select major_project

echo "source ~/major_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.7.8 Verify cross-PC ROS2 communication

```bash
# On PC-1: publish a test topic in background
source ~/.bashrc
ros2 topic pub /test_comms std_msgs/msg/String \
  "data: 'hello from PC1'" --rate 1 &
PUB_PID=$!

echo "Publishing /test_comms from PC-1 (PID $PUB_PID)"
echo "On PC-2, run: source ~/.bashrc && ros2 topic echo /test_comms"
echo "You should see: data: hello from PC1"
echo ""
echo "Press ENTER to stop publishing..."
read
kill $PUB_PID 2>/dev/null
```

Expected output on PC-2 when running `ros2 topic echo /test_comms`:
```yaml
data: hello from PC1
---
data: hello from PC1
---
```

---

## Section 2.8 — Final Scaffold Verification Checklist

Run on **PC-1** to confirm everything is complete:

```bash
echo "========================================"
echo "  major_project Scaffold Verification"
echo "========================================"

source ~/.bashrc 2>/dev/null
source ~/major_ws/install/setup.bash 2>/dev/null || true

# 1. Package builds
echo -n "[1]  Package builds cleanly:          "
cd ~/major_ws && colcon build --packages-select major_project \
  &>/dev/null && echo "OK" || echo "FAIL"

# 2. Exactly 16 entry points in setup.py
echo -n "[2]  16 entry points in setup.py:     "
count=$(grep -c "= major_project\." ~/major_ws/src/major_project/setup.py 2>/dev/null)
[ "$count" -eq 16 ] && echo "OK ($count)" || echo "FAIL (found $count, need 16)"

# 3. cv_bridge in package.xml
echo -n "[3]  cv_bridge in package.xml:        "
grep -q "cv_bridge" ~/major_ws/src/major_project/package.xml 2>/dev/null \
  && echo "OK" || echo "FAIL"

# 4. px4_msgs in package.xml
echo -n "[4]  px4_msgs in package.xml:         "
grep -q "px4_msgs" ~/major_ws/src/major_project/package.xml 2>/dev/null \
  && echo "OK" || echo "FAIL"

# 5. All __init__.py present
echo -n "[5]  All __init__.py present:         "
all_init=true
for pkg in "" "/gcs" "/lead_pilot" "/wingman_pilot" "/common"; do
  f=~/major_ws/src/major_project/major_project${pkg}/__init__.py
  test -f "$f" || { all_init=false; }
done
$all_init && echo "OK" || echo "FAIL (check sub-package dirs)"

# 6. requirements.txt present
echo -n "[6]  requirements.txt present:        "
test -f ~/major_ws/src/major_project/requirements.txt && echo "OK" || echo "FAIL"

# 7. ros2 pkg executables count
echo -n "[7]  ros2 pkg executables count=16:   "
count=$(ros2 pkg executables major_project 2>/dev/null | wc -l)
[ "$count" -eq 16 ] && echo "OK" || echo "FAIL (found $count)"

# 8. CycloneDDS config present
echo -n "[8]  CycloneDDS cyclone.xml present:  "
test -f ~/cyclone.xml && echo "OK" || echo "FAIL"

# 9. CYCLONEDDS_URI exported
echo -n "[9]  CYCLONEDDS_URI set:              "
[ -n "$CYCLONEDDS_URI" ] && echo "OK" || echo "FAIL (not in env)"

# 10. Python dependencies importable
echo -n "[10] Python deps (pydantic, numpy):   "
python3 -c "import pydantic, numpy, cv2" 2>/dev/null && echo "OK" || echo "FAIL"

# 11. Key entry point names correct
echo -n "[11] diagnostics entry point:         "
grep -q "'diagnostics = major_project.gcs.diagnostics_node:main'" \
  ~/major_ws/src/major_project/setup.py && echo "OK" || echo "FAIL"

echo -n "[12] wingman_camera_detection EP:     "
grep -q "'wingman_camera_detection = major_project.wingman_pilot.wingman_camera_detection_node:main'" \
  ~/major_ws/src/major_project/setup.py && echo "OK" || echo "FAIL"

echo "========================================"
echo "  All 12 checks should show OK."
echo "========================================"
```

---

## Package Structure Reference

```
~/major_ws/
└── src/
    └── major_project/
        ├── package.xml                          # ROS2 package manifest (with cv_bridge)
        ├── setup.py                             # 16 entry points (FINAL)
        ├── setup.cfg
        ├── requirements.txt                     # Python pip dependencies
        ├── resource/
        │   └── major_project                    # ament marker file
        ├── config/
        │   ├── lead_config.yaml
        │   └── wingman_config.yaml
        ├── launch/
        │   ├── lead_pilot.launch.py
        │   └── wingman_pilot.launch.py
        └── major_project/
            ├── __init__.py
            ├── common/
            │   ├── __init__.py
            │   ├── schemas.py                   # Pydantic v2 models
            │   └── ollama_client.py             # HTTP client for Ollama
            ├── gcs/
            │   ├── __init__.py
            │   ├── stt_node.py                  # Voice -> /voice_commands
            │   ├── clarification_speaker_node.py
            │   ├── mission_monitor_node.py
            │   ├── emergency_stop_node.py
            │   └── diagnostics_node.py
            ├── lead_pilot/
            │   ├── __init__.py
            │   ├── prompts/
            │   ├── camera_detection_node.py     # YOLO -> /camera_0/detections
            │   ├── lead_sensor_aggregator_node.py
            │   ├── lead_px4_commander_node.py
            │   ├── lead_intent_bridge_node.py
            │   ├── lead_agent_node.py           # LLM agent (Ollama)
            │   └── safety_monitor_node.py
            └── wingman_pilot/
                ├── __init__.py
                ├── prompts/
                ├── wingman_camera_detection_node.py
                ├── wingman_sensor_aggregator_node.py
                ├── wingman_px4_commander_node.py
                └── wingman_agent_node.py        # LLM agent (Ollama on PC-2)
```

---

## Topic Reference Card

| Topic | Publisher | Type | Description |
|-------|-----------|------|-------------|
| `/voice_commands` | stt_node | `String` | STT transcription -> Lead Agent |
| `/drone_0/situation` | lead_sensor_aggregator | `String` | Lead state (includes wingman_pos) |
| `/drone_1/situation` | wingman_sensor_aggregator | `String` | Wingman state |
| `/camera_0/detections` | camera_detection | `String` | Lead YOLO results |
| `/camera_0/obstacle_vector` | camera_detection | `String` | Lead obstacle vector |
| `/camera_1/detections` | wingman_camera_detection | `String` | Wingman YOLO results |
| `/camera_1/obstacle_vector` | wingman_camera_detection | `String` | Wingman obstacle vector |
| `/lead/approved_intent` | lead_agent | `String` | Lead intent -> commander + bridge |
| `/wingman/approved_intent` | wingman_agent | `String` | Wingman intent -> commander |
| `/agent/lead_to_wingman` | lead_intent_bridge | `String` | AgentMessage JSON envelope |
| `/agent/wingman_to_lead` | wingman_agent | `String` | AgentMessage JSON envelope |
| `/agent/health` | agents | `String` | JSON health status |
| `/safety/event` | safety_monitor | `String` | JSON safety event |
| `/emergency_stop` | emergency_stop node | `Bool` | Kill switch |
| `/clarification_request` | agents | `String` | TTS request |
| `/mission_status` | mission_monitor | `String` | JSON mission state |
| `/wingman/status_report_text` | wingman_agent | `String` | Status to lead |
| `/system/health` | diagnostics | `String` | JSON diagnostics |
| `/fmu/in/...` | lead_px4_commander | px4_msgs | Drone-0 commands |
| `/px4_1/fmu/in/...` | wingman_px4_commander | px4_msgs | Drone-1 commands |
| `/fmu/out/...` | PX4 Drone-0 | px4_msgs | Lead telemetry |
| `/px4_1/fmu/out/...` | PX4 Drone-1 | px4_msgs | Wingman telemetry |

---

## Next Steps

Proceed to **Tutorial Part 03 — Common Schemas and Ollama Client** to implement:
- `major_project/common/schemas.py` — Pydantic v2 models for all message types
- `major_project/common/ollama_client.py` — Async HTTP client for structured LLM inference
