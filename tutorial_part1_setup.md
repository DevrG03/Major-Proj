# Multi-Drone SLM Pilot System — Step-by-Step Tutorial
## Part 1: System Setup (Ubuntu 26.04 LTS + ROS2 Lyrical)

> **Conventions used in this tutorial:**
> - `[PC-1]` = Lead Pilot machine (runs Gazebo server, Lead SLM, Drone-0)
> - `[PC-2]` = Wingman machine (runs Wingman SLM, Drone-1)
> - `[BOTH]` = run on both PCs
> - Commands prefixed with `#` are comments, not run
> - Every code block is copy-paste ready
> - When a step says "verify:", run the verification command before proceeding

---

## PART 0: Prerequisites

### 0.1 What You Need Before Starting

**Hardware:**
- 2 PCs running Ubuntu 26.04 LTS (fresh install recommended)
- Both PCs on the same WiFi network (same router, same subnet)
- Minimum 16 GB RAM per PC (Gazebo + PX4 + Ollama is memory-heavy)
- Minimum 50 GB free disk per PC
- Microphone connected to PC-1

**Know your network details before starting:**
```bash
# [BOTH] Run this on each PC and note the output
ip addr show | grep "inet " | grep -v 127.0.0.1
# Example output: inet 192.168.1.10/24 — note this IP
# PC-1 IP: 192.168.1.10  (example — yours will differ)
# PC-2 IP: 192.168.1.11  (example — yours will differ)
hostname
# Note each machine's hostname too
```

**Write down:**
```
PC-1 IP:  _______________
PC-2 IP:  _______________
PC-1 hostname: _______________
PC-2 hostname: _______________
WiFi interface name (usually wlan0 or wlp3s0): _______________
```

**Find your WiFi interface name:**
```bash
ip link show | grep -E "^[0-9]+: w"
# The name after the number (e.g., wlan0, wlp2s0, wlp3s0)
```

---

## PART 1: PC-1 Base System

### 1.1 Update Ubuntu 26.04

```bash
# [PC-1] Update all packages
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y
```

**Verify:**
```bash
lsb_release -a
# Must show: Ubuntu 26.04 LTS
```

### 1.2 Install System Dependencies

```bash
# [PC-1] Core build tools
sudo apt install -y \
  build-essential \
  cmake \
  ninja-build \
  git \
  git-lfs \
  curl \
  wget \
  gnupg2 \
  lsb-release \
  software-properties-common \
  python3-pip \
  python3-venv \
  python3-dev \
  python3-setuptools \
  portaudio19-dev \
  libportaudio2 \
  ffmpeg \
  espeak-ng \
  net-tools \
  openssh-server \
  htop \
  tmux
```

```bash
# [PC-1] Enable SSH so you can control PC-1 remotely if needed
sudo systemctl enable ssh
sudo systemctl start ssh
```

### 1.3 Install ROS2 Lyrical

```bash
# [PC-1] Step 1: Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
# [PC-1] Step 2: Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

```bash
# [PC-1] Step 3: Update and install ROS2 Lyrical desktop
sudo apt update
sudo apt install -y ros-lyrical-desktop
```

```bash
# [PC-1] Step 4: Install ROS2 development tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-lyrical-rmw-cyclonedds-cpp \
  ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-sim \
  ros-lyrical-px4-msgs
```

> **Note on ros-lyrical-px4-msgs:** If this binary package is not yet available
> (Lyrical is new), you will build px4_msgs from source in Step 1.8.
> Run `apt search px4-msgs` to check. If not found, skip this line.

> **Verify ros-gz package names before installing:**
> ```bash
> # [PC-1] Check which ros-gz packages exist for your distro
> apt-cache search "ros-lyrical-ros-gz" 2>/dev/null || apt-cache search "ros-lyrical-gz"
> # ROS2 Lyrical may ship packages as ros-lyrical-ros-gz-bridge
> # or under a different gz alias. Use the name shown in the search output.
> # If nothing appears, the gz bridge must be installed from source (rare for LTS).
> ```

```bash
# [PC-1] Step 5: Initialise rosdep
sudo rosdep init
rosdep update
```

```bash
# [PC-1] Step 6: Add ROS2 to shell permanently
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify:**
```bash
ros2 --version
# Should show: ros2 <version> (lyrical)
printenv ROS_DISTRO
# Should show: lyrical
```

### 1.4 Configure CycloneDDS

```bash
# [PC-1] Install CycloneDDS RMW (may already be installed above)
sudo apt install -y ros-lyrical-rmw-cyclonedds-cpp
```

```bash
# [PC-1] Set DDS implementation and domain
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
source ~/.bashrc
```

```bash
# [PC-1] Create CycloneDDS config for WiFi
sudo mkdir -p /etc/cyclonedds
```

```bash
# [PC-1] Write CycloneDDS config
# Replace wlan0 with YOUR WiFi interface name from step 0.1
# Replace 192.168.1.10 with YOUR PC-1 IP
# Replace 192.168.1.11 with YOUR PC-2 IP
cat << 'EOF' | sudo tee /etc/cyclonedds/cyclonedds.xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="wlan0" multicast="true" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.10"/>
        <Peer address="192.168.1.11"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
```

```bash
# [PC-1] Point CycloneDDS to config
echo 'export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
```

**Verify:**
```bash
printenv RMW_IMPLEMENTATION
# rmw_cyclonedds_cpp
printenv ROS_DOMAIN_ID
# 42
printenv CYCLONEDDS_URI
# file:///etc/cyclonedds/cyclonedds.xml
```

### 1.5 Create ROS2 Workspace

```bash
# [PC-1]
mkdir -p ~/major_ws/src
cd ~/major_ws

# Create venv WITH system site-packages so ROS2 Python tools are visible
# (without --system-site-packages, catkin_pkg / rosidl_adapter are missing)
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

# Pin empy — ROS2 Lyrical needs 3.x, not 4.x
pip install 'empy==3.3.4'

colcon build
echo "source ~/major_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/major_ws/.venv/bin/activate" >> ~/.bashrc
source ~/.bashrc
```

**Verify:**
```bash
ls ~/major_ws/
# Should show: build  install  log  src
```

### 1.6 Install Python Dependencies

> **Note:** The venv from step 1.5 must be active (`source ~/major_ws/.venv/bin/activate`). Use `pip`, not `pip3`, so packages install into the venv.

```bash
# [PC-1] Install all Python packages needed (venv must be active)
pip install \
  pydantic==2.* \
  faster-whisper \
  requests \
  numpy \
  ultralytics \
  rich \
  pyaudio \
  sounddevice \
  scipy
```

**Verify each critical package:**
```bash
python3 -c "import pydantic; print('pydantic', pydantic.__version__)"
python3 -c "import faster_whisper; print('faster_whisper ok')"
python3 -c "import ultralytics; print('ultralytics ok')"
python3 -c "import rich; print('rich ok')"
```

### 1.7 Install PX4 Dependencies

```bash
# [PC-1] PX4 requires these system packages
sudo apt install -y \
  astyle \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer1.0-dev \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly \
  libeigen3-dev \
  libopencv-dev \
  protobuf-compiler \
  python3-jinja2 \
  python3-jsonschema \
  python3-toml \
  python3-numpy \
  python3-packaging \
  python3-kconfiglib \
  python3-lxml
```

```bash
# [PC-1] PX4 Python build tools (venv must be active)
pip install \
  kconfiglib \
  jinja2 \
  jsonschema \
  pyros-genmsg \
  packaging \
  toml \
  future \
  empy==3.3.4 \
  pyserial
```

> **Important:** empy must be version 3.3.4, not 4.x. PX4 build fails with empy 4.

```bash
# Verify empy version
python3 -c "import em; print(em.__version__)"
# Must print 3.3.4
# If it shows 4.x, force downgrade:
pip3 install 'empy==3.3.4' --force-reinstall
```

### 1.8 Clone and Build px4_msgs (if binary not available)

```bash
# [PC-1] Check if binary px4_msgs is available first
apt-cache search ros-lyrical-px4-msgs
# If found, skip this section — it was installed in step 1.3
# If NOT found, proceed:

cd ~/major_ws/src
git clone https://github.com/PX4/px4_msgs.git --branch main
cd ~/major_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

**Verify:**
```bash
ros2 interface list | grep px4_msgs | head -5
# Should list px4_msgs message types
```

### 1.9 Clone PX4 Autopilot

```bash
# [PC-1] Clone PX4 (this will take several minutes — it's a large repo)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```

```bash
# [PC-1] Check out a stable release tag
# Find the latest stable tag:
git tag | grep v1 | sort -V | tail -5
# Pick the latest v1.x stable (e.g., v1.15.0 or v1.14.3)
# Replace v1.15.0 with the latest tag you see:
git checkout v1.15.0
git submodule update --init --recursive
```

```bash
# [PC-1] Run PX4 ubuntu setup script (installs additional dependencies)
bash Tools/setup/ubuntu.sh --no-nuttx
# Answer 'y' to prompts
# This takes 5-10 minutes
```

```bash
# [PC-1] Reload shell after ubuntu.sh adds groups
source ~/.bashrc
```

### 1.10 Install Gazebo

> On Ubuntu 26.04 (Resolute), the OSRF apt repo does not yet have pre-built Gazebo packages. **Do not try `gz-harmonic` or `gz-ionic` via apt — they will not be found.**
> Instead, PX4's own setup script installs Gazebo automatically. Run it and let it handle Gazebo.

```bash
# [PC-1] PX4 setup script — installs Gazebo Jetty (gz-sim 10.x) automatically
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-nuttx
source ~/.bashrc
```

```bash
# [PC-1] Verify Gazebo
gz sim --version
# Should show: 10.x.x  (Gazebo Jetty)
```

```bash
# [PC-1] Install ROS2-Gazebo bridge
sudo apt install -y ros-lyrical-ros-gz-bridge ros-lyrical-ros-gz-sim
```

### 1.11 Build PX4 SITL

> **GCC 15 note:** Ubuntu 26.04 ships GCC 15.2.0 which has stricter `-Wmaybe-uninitialized` checks that trigger false positives in PX4's Gz plugins. The build will fail at `SpacecraftThrusterModel.cpp` / `GenericMotorModel.cpp` without the workaround below.

```bash
# [PC-1] First build (takes 10-20 minutes on first run)
cd ~/PX4-Autopilot

# Demote the GCC 15 false-positive warning from error to warning
cmake build/px4_sitl_default -DCMAKE_CXX_FLAGS="-Wno-error=maybe-uninitialized"

make px4_sitl
```

```bash
# [PC-1] Test single drone launch (just verifies it builds and runs)
# This will open Gazebo — close it after you see the drone
make px4_sitl gz_x500_mono_cam
# After Gazebo opens and drone appears, press Ctrl+C to stop
```

**Verify:**
```bash
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
# File must exist
```

### 1.12 Build MicroXRCE-DDS Agent

```bash
# [PC-1]
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

**Verify:**
```bash
MicroXRCEAgent --version
# Shows version string — agent is installed
```

### 1.13 Install Ollama

```bash
# [PC-1]
curl -fsSL https://ollama.com/install.sh | sh
```

```bash
# [PC-1] Start Ollama service
sudo systemctl enable ollama
sudo systemctl start ollama
# Wait 5 seconds for service to start
sleep 5
```

```bash
# [PC-1] Pull the SLM model (downloads ~2GB)
ollama pull qwen2.5-coder:3b
```

```bash
# [PC-1] Test the model runs
ollama run qwen2.5-coder:3b "Reply with only: OK"
# Should print: OK (may take 30-60s on first run — model loads into RAM)
```

```bash
# [PC-1] Configure Ollama to listen on all interfaces
# (so PC-2 can also call PC-1's Ollama if needed)
sudo systemctl edit ollama
# This opens a systemd override editor — add these lines:
# [Service]
# Environment="OLLAMA_HOST=0.0.0.0:11434"
# Save and exit (Ctrl+X if nano, :wq if vim)
sudo systemctl restart ollama
```

**Verify Ollama API is reachable:**
```bash
curl http://localhost:11434/api/tags
# Should return JSON listing the qwen2.5-coder:3b model
```

---

## PART 2: PC-2 Base System

> **All steps in Part 2 run on PC-2 only, unless noted.**

### 2.1 — 2.5: Mirror PC-1 Setup

Run these on PC-2, identical to Part 1 (Steps 1.1 – 1.5 only):
- Step 1.1: Update Ubuntu
- Step 1.2: Install system dependencies
- Step 1.3: Install ROS2 Lyrical
- Step 1.4: Configure CycloneDDS (same config, same IPs — both PCs list both IPs as peers)
- Step 1.5: Create ROS2 workspace + venv (this creates `~/major_ws/.venv` with `--system-site-packages` and pins `empy==3.3.4`)

> **Do NOT run Step 1.6 from PC-1** — PC-2 does not need STT, audio, or vision packages. Use the PC-2 specific package list below instead.

### 2.6 Install Python Dependencies on PC-2

PC-2 only runs the Wingman NLU and commander nodes — no microphone, no camera detection, no Whisper.

```bash
# [PC-2] Activate venv (created in Step 1.5)
source ~/major_ws/.venv/bin/activate

# Install only what PC-2 needs
pip install \
  'pydantic==2.*' \
  requests \
  numpy \
  rich \
  kconfiglib jinja2 jsonschema pyros-genmsg packaging toml future pyserial
```

**Verify:**
```bash
python3 -c "import pydantic; print('pydantic', pydantic.__version__)"
python3 -c "import rich; print('rich ok')"
pip show empy | grep Version
# Must show: Version: 3.3.4
```

### 2.7 Install px4_msgs on PC-2

```bash
# [PC-2] Clone and build px4_msgs
cd ~/major_ws/src
git clone https://github.com/PX4/px4_msgs.git --branch main
cd ~/major_ws
colcon build --packages-select px4_msgs
source install/setup.bash
echo "source ~/major_ws/install/setup.bash" >> ~/.bashrc
```

### 2.8 Install PX4 on PC-2 (No Gazebo)

```bash
# [PC-2] Clone PX4 (same version as PC-1 — MUST match exactly)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.15.0   # Use same tag as PC-1
git submodule update --init --recursive
bash Tools/setup/ubuntu.sh --no-nuttx
source ~/.bashrc
```

```bash
# [PC-2] Build PX4 SITL (same GCC 15 workaround as PC-1)
cd ~/PX4-Autopilot
cmake build/px4_sitl_default -DCMAKE_CXX_FLAGS="-Wno-error=maybe-uninitialized"
make px4_sitl
```

> **PC-2 does NOT need Gazebo** — it will connect to PC-1's Gazebo server over the network.
> Skip steps 1.10 (Gazebo install) for PC-2.

### 2.9 Install Ollama on PC-2

```bash
# [PC-2] Same as PC-1 step 1.13
curl -fsSL https://ollama.com/install.sh | sh
sudo systemctl enable ollama
sudo systemctl start ollama
sleep 5
ollama pull qwen2.5-coder:3b
sudo systemctl edit ollama
# Add: [Service]
#      Environment="OLLAMA_HOST=0.0.0.0:11434"
sudo systemctl restart ollama
```

### 2.10 MicroXRCE-DDS Agent on PC-2

```bash
# [PC-2] Same as PC-1 step 1.12
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

---

## PART 3: Cross-PC Network Verification

### 3.1 Verify SSH Between PCs

```bash
# [PC-1] SSH into PC-2 to confirm network works
ssh user@192.168.1.11   # Replace with PC-2's actual IP
# If it connects, exit:
exit
```

```bash
# [PC-2] SSH into PC-1
ssh user@192.168.1.10
exit
```

### 3.2 Test Basic ROS2 Cross-PC Topic Exchange

**Terminal on PC-1:**
```bash
# [PC-1]
source ~/.bashrc
ros2 topic pub /network_test std_msgs/msg/String \
  "data: 'hello_from_pc1'" --rate 1
```

**Terminal on PC-2 (simultaneously):**
```bash
# [PC-2]
source ~/.bashrc
ros2 topic echo /network_test
# Must print messages like: data: hello_from_pc1
```

**If no messages appear on PC-2:**

Step A — Check firewall:
```bash
# [BOTH] Disable firewall temporarily to diagnose
sudo ufw disable
# Retry the topic test above
# If it works now, re-enable and add rules:
sudo ufw enable
sudo ufw allow in proto udp from 192.168.1.0/24
sudo ufw allow in proto tcp from 192.168.1.0/24
```

Step B — Force unicast discovery (if multicast is blocked by router):
```bash
# [BOTH] Edit /etc/cyclonedds/cyclonedds.xml
# Change <AllowMulticast>true</AllowMulticast>
# to    <AllowMulticast>false</AllowMulticast>
# Save, then:
source ~/.bashrc
# Retry topic test
```

Step C — Verify same ROS_DOMAIN_ID:
```bash
# [BOTH]
printenv ROS_DOMAIN_ID
# Both must print: 42
```

**Verify success:**
```bash
# [PC-2] After seeing messages from PC-1, also publish back:
ros2 topic pub /network_test_reverse std_msgs/msg/String \
  "data: 'hello_from_pc2'" --rate 1
# [PC-1] Verify:
ros2 topic echo /network_test_reverse
# Must print: data: hello_from_pc2
```

### 3.3 Measure Round-Trip Latency

```bash
# [PC-1] Install topic tools if not present
sudo apt install -y ros-lyrical-topic-tools || true

# [PC-1] Measure latency to PC-2
ros2 topic delay /network_test_reverse
# This shows the difference between send timestamp and receive timestamp
# Target: < 5ms on same WiFi network
# Acceptable: < 20ms
# Problematic: > 50ms (check router, interference, distance)
```

### 3.4 Explicit ROS_DOMAIN_ID Verification (Cross-PC)

This is a mandatory check. If the two PCs have different `ROS_DOMAIN_ID` values, DDS will not exchange any topics — all other troubleshooting will be a dead end.

```bash
# [PC-1] Check domain ID
printenv ROS_DOMAIN_ID
# Must print: 42

# [PC-2] Check domain ID
printenv ROS_DOMAIN_ID
# Must also print: 42
```

**If one PC shows a different value or empty:**
```bash
# [Both PCs — whichever is wrong]
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
source ~/.bashrc
printenv ROS_DOMAIN_ID   # must now show 42
```

**Double-check the value is sourced from .bashrc, not a stale terminal:**
```bash
# [BOTH] Open a fresh terminal and check again — env vars from >> ~/.bashrc
# only take effect in NEW terminals, not the current one.
# If you see an old value in a terminal that was open before you added the line,
# close it and open a new one.
```

> **Golden rule:** Any terminal that will run `ros2` commands must have been opened AFTER the `export ROS_DOMAIN_ID=42` line was added to `~/.bashrc`.

---

## PART 4: Single-Drone SITL Verification

Before attempting multi-drone, verify single-drone works end-to-end.

### 4.1 Launch Single Drone SITL

Open 3 terminals on PC-1:

**Terminal 1 — PX4 SITL:**
```bash
# [PC-1 T1]
cd ~/PX4-Autopilot
make px4_sitl gz_x500_mono_cam
# Wait until you see: INFO [commander] Ready for takeoff!
# Gazebo should open showing one drone
```

**Terminal 2 — XRCE-DDS Bridge:**
```bash
# [PC-1 T2]
source ~/.bashrc
MicroXRCEAgent udp4 -p 8888
# Wait until you see client connections logged
```

**Terminal 3 — Verify ROS2 topics:**
```bash
# [PC-1 T3]
source ~/.bashrc
ros2 topic list | grep fmu
```

**Expected output (must see these topics):**
```
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/out/battery_status
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status
```

```bash
# [PC-1 T3] Echo position to confirm data flows
ros2 topic echo /fmu/out/vehicle_local_position --once
# Must print position data (x, y, z fields)
```

```bash
# [PC-1 T3] Check publish rate
ros2 topic hz /fmu/out/vehicle_local_position
# Must show: average rate > 10 Hz
```

### 4.2 Test Manual Offboard Command

```bash
# [PC-1 T3] Create quick takeoff test
cat << 'EOF' > /tmp/test_takeoff.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import time

class TakeoffTest(Node):
    def __init__(self):
        super().__init__('takeoff_test')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.timer = self.create_timer(0.1, self.tick)
        self.count = 0

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, z=-5.0):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, z]  # NED: negative z = up
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def arm_and_offboard(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0; cmd.param2 = 6.0  # offboard mode
        cmd.target_system = 1; cmd.target_component = 1
        cmd.source_system = 1; cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)
        time.sleep(0.1)
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def tick(self):
        self.publish_offboard_mode()
        self.publish_setpoint(z=-5.0)
        if self.count == 20:  # after 2 seconds of setpoints
            self.arm_and_offboard()
        self.count += 1

def main():
    rclpy.init()
    node = TakeoffTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
python3 /tmp/test_takeoff.py
```

**Expected:** Drone in Gazebo arms and rises to 5 metres above ground.

Press `Ctrl+C` to stop. Close Gazebo (`Ctrl+C` in Terminal 1).

---

## PART 5: Two-Drone SITL on PC-1

### 5.1 Understand Multi-Vehicle Namespacing

PX4 multi-vehicle SITL works like this:
- Instance 0 (`-i 0`): topics at `/fmu/in/...` and `/fmu/out/...` (no prefix)
- Instance 1 (`-i 1`): topics at `/px4_1/fmu/in/...` and `/px4_1/fmu/out/...`
- Instance N (`-i N`): topics at `/px4_N/fmu/in/...` and `/px4_N/fmu/out/...`

Each instance binds a different UDP port: instance 0 = port 14540, instance 1 = port 14541, etc.

### 5.2 Launch Two Drones

You need 3 terminals on PC-1. Use `tmux` to keep things organised:

```bash
# [PC-1] Start tmux session
tmux new-session -s sitl
# Split into 3 panes: Ctrl+B then " (horizontal split), Ctrl+B then % (vertical)
# Or just open 3 separate terminals
```

**`PX4_GZ_MODEL_POSE` format:** `"x,y,z,roll,pitch,yaw"` — all in metres and radians.
- `x,y` = ground position offset; `z=0` = ground level (do NOT set negative; PX4 spawns on terrain)
- `roll,pitch,yaw = 0,0,0` = upright with North-facing heading
- Example: `"5,0,0,0,0,0"` = 5 metres East of origin, upright

**Terminal 1 — Drone 0 (Lead):**
```bash
# [PC-1 T1]
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500_mono_cam \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_UXRCE_DDS_KEY=1 \
./build/px4_sitl_default/bin/px4 -i 0 -d
# Wait for: INFO [commander] Ready for takeoff!
# Gazebo opens with Drone-0
```

**Terminal 2 — Drone 1 (Wingman):**
```bash
# [PC-1 T2] After Drone-0 is ready (about 10 seconds after T1):
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500_mono_cam \
PX4_GZ_MODEL_POSE="5,0,0,0,0,0" \
PX4_UXRCE_DDS_KEY=2 \
./build/px4_sitl_default/bin/px4 -i 1 -d
# Drone-1 appears in existing Gazebo window, 5m East of Drone-0
```

**Terminal 3 — XRCE-DDS Agent (serves BOTH drones):**
```bash
# [PC-1 T3]
source ~/.bashrc
MicroXRCEAgent udp4 -p 8888
# You should see TWO client connections logged
```

### 5.3 Verify Both Drones Have ROS2 Topics

```bash
# [PC-1] New terminal
source ~/.bashrc

# Check Drone-0 topics
ros2 topic list | grep "^/fmu"
# Must show: /fmu/in/trajectory_setpoint etc.

# Check Drone-1 topics
ros2 topic list | grep "^/px4_1"
# Must show: /px4_1/fmu/in/trajectory_setpoint etc.

# Confirm both publish position
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo /px4_1/fmu/out/vehicle_local_position --once
# Both must return position data
```

### 5.4 Verify Independent Control

```bash
# [PC-1] Quick test: take off ONLY Drone-0
# Edit /tmp/test_takeoff.py — it already uses /fmu/in/... (Drone-0)
# Run it and verify only Drone-0 rises; Drone-1 stays on ground
python3 /tmp/test_takeoff.py
# Drone-0 rises, Drone-1 stays still -> namespacing works correctly
```

Press `Ctrl+C`. Keep Gazebo and PX4 SITL running for the next step.

---

## PART 6: Ollama Latency Benchmark [CRITICAL]

> **This step determines whether the architecture is viable.**
> PX4 offboard mode requires setpoint updates faster than 2 Hz (one update per 500ms).
> The SLM NLU pipeline must complete faster than this.

### 6.1 Create the Benchmark Script

```bash
# [PC-1]
mkdir -p ~/major_ws/src/major_project/benchmark
cat << 'EOF' > ~/major_ws/src/major_project/benchmark/latency_benchmark.py
"""
Latency benchmark for Ollama SLM inference in the NLU pipeline.
Tests: local inference + remote inference (PC-2 calling PC-1's Ollama).
Measures: mean, median, p95, p99 across 30 trials per command type.
"""
import time
import statistics
import json
import requests
import sys

SYSTEM_PROMPT = """You parse UAV voice commands. Output ONLY valid JSON matching this schema:
{"action": string, "altitude": number|null, "distance": number|null,
 "direction": string|null, "confidence": "high"|"medium"|"low",
 "clarification_question": string|null}
Valid actions: takeoff, move, hover, land, rtl, search"""

TEST_COMMANDS = {
    "clear":     "take off to 5 meters",
    "ambiguous": "go that way",
    "oos":       "what is the weather today",
}

def run_inference(prompt, host="localhost", port=11434):
    url = f"http://{host}:{port}/api/generate"
    payload = {
        "model": "qwen2.5-coder:3b",
        "prompt": f"Command: {prompt}",
        "system": SYSTEM_PROMPT,
        "stream": False,
        "format": "json",
        "options": {"num_ctx": 512, "temperature": 0}
    }
    t_start = time.perf_counter()
    try:
        r = requests.post(url, json=payload, timeout=30)
        t_end = time.perf_counter()
        latency = t_end - t_start
        return latency, r.status_code == 200, r.json().get("response", "")
    except Exception as e:
        return None, False, str(e)

def benchmark(host="localhost", port=11434, n_trials=30):
    print(f"\n{'='*60}")
    print(f"Benchmarking Ollama at {host}:{port} — {n_trials} trials per command")
    print(f"{'='*60}")

    results = {}
    for cmd_type, cmd_text in TEST_COMMANDS.items():
        latencies = []
        failures = 0
        print(f"\n[{cmd_type.upper()}] '{cmd_text}'")
        for i in range(n_trials):
            lat, success, response = run_inference(cmd_text, host, port)
            if lat is not None and success:
                latencies.append(lat * 1000)  # convert to ms
                print(f"  Trial {i+1:2d}: {lat*1000:6.0f}ms", end="")
                # Check if output is valid JSON
                try:
                    parsed = json.loads(response)
                    print(f" [JSON OK] confidence={parsed.get('confidence','?')}")
                except:
                    print(f" [JSON FAIL] raw={response[:50]}")
                    failures += 1
            else:
                print(f"  Trial {i+1:2d}: FAILED")
                failures += 1

        if latencies:
            sorted_lat = sorted(latencies)
            results[cmd_type] = {
                "mean":   statistics.mean(latencies),
                "median": statistics.median(latencies),
                "p95":    sorted_lat[int(len(sorted_lat) * 0.95)],
                "p99":    sorted_lat[int(len(sorted_lat) * 0.99)],
                "min":    min(latencies),
                "max":    max(latencies),
                "failures": failures,
            }

    print(f"\n{'='*60}")
    print("SUMMARY (milliseconds)")
    print(f"{'='*60}")
    print(f"{'Type':<12} {'Mean':>8} {'Median':>8} {'P95':>8} {'P99':>8} {'Fails':>6}")
    print("-"*60)
    all_means = []
    for cmd_type, r in results.items():
        print(f"{cmd_type:<12} {r['mean']:>8.0f} {r['median']:>8.0f} {r['p95']:>8.0f} {r['p99']:>8.0f} {r['failures']:>6}")
        all_means.append(r['mean'])

    if all_means:
        overall_mean = statistics.mean(all_means)
        print(f"\nOverall mean latency: {overall_mean:.0f}ms")
        print(f"PX4 offboard budget:  500ms (2 Hz)")
        if overall_mean < 400:
            print("RESULT: ✓ VIABLE — latency is within budget (with margin)")
        elif overall_mean < 500:
            print("RESULT: ⚠ MARGINAL — use async NLU pattern (last-known setpoint at 10Hz)")
        else:
            print("RESULT: ✗ TOO SLOW — must use async NLU. See tutorial Part 9 async section.")

    return results

if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 11434
    benchmark(host=host, port=port, n_trials=30)
EOF
```

### 6.2 Run Local Benchmark (PC-1)

```bash
# [PC-1]
python3 ~/major_ws/src/major_project/benchmark/latency_benchmark.py localhost 11434
# This takes about 5-10 minutes (30 trials × 3 command types)
# Save the output — you'll need these numbers for your paper
python3 ~/major_ws/src/major_project/benchmark/latency_benchmark.py localhost 11434 \
  | tee ~/major_ws/src/major_project/benchmark/results_local_pc1.txt
```

### 6.3 Run Remote Benchmark (PC-2 calling PC-1's Ollama)

```bash
# [PC-2] Call PC-1's Ollama over WiFi
python3 ~/major_ws/src/major_project/benchmark/latency_benchmark.py 192.168.1.10 11434 \
  | tee ~/benchmark_results_remote.txt
# Replace 192.168.1.10 with PC-1's actual IP
```

### 6.4 Interpret Results and Decide Architecture

**Decision table:**

| Mean latency | Decision |
|---|---|
| < 400ms | Synchronous NLU — SLM runs in the ROS2 callback, simple architecture |
| 400–800ms | Async NLU — SLM runs in background thread, ROS2 publishes last-known setpoint at 10Hz independently |
| > 800ms | Async NLU + reduce model context (`num_ctx=256`) or use `qwen2.5-coder:1.5b` |

**If async NLU is needed** (add this to your notes — we implement it in Part 9):
- The NLU node has two threads: inference thread (slow, runs SLM) and setpoint thread (fast, 10Hz)
- Setpoint thread always publishes the last valid intent
- Inference thread updates the intent when a new result is ready
- This satisfies PX4 offboard timing regardless of SLM speed

---

*End of Part 1 of the tutorial. Continue in tutorial_part2_package.md*
