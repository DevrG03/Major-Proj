# Multi-Drone SLM Pilot System — Complete Tutorial
> Single consolidated reference. Follow sections 1–10 in order. No prior parts needed.

---

## Section 1: Infrastructure & Environment Setup

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

> **Version pin:** The agent version MUST match the client bundled in PX4. PX4 v1.17 bundles client v2.4.0 — use agent **v2.4.3** (v2.4.0–v2.4.2 have a broken fastdds `2.12.x` tag reference).
> **GCC 15 note:** Building without `-fno-stack-protector` causes "stack smashing detected" at runtime on Ubuntu 26.04.

```bash
# [PC-1]
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
git fetch --tags
git checkout v2.4.3
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-fno-stack-protector -D_FORTIFY_SOURCE=0" \
  -DCMAKE_C_FLAGS="-fno-stack-protector -D_FORTIFY_SOURCE=0"

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
ollama pull qwen3.5:2b
```

```bash
# [PC-1] Test the model runs
ollama run qwen3.5:2b "Reply with only: OK"
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
# Should return JSON listing the qwen3.5:2b model
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
ollama pull qwen3.5:2b
sudo systemctl edit ollama
# Add: [Service]
#      Environment="OLLAMA_HOST=0.0.0.0:11434"
sudo systemctl restart ollama
```

### 2.10 MicroXRCE-DDS Agent on PC-2

```bash
# [PC-2] Same as PC-1 step 1.12 — same version pin and GCC 15 flags
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
git fetch --tags
git checkout v2.4.3
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-fno-stack-protector -D_FORTIFY_SOURCE=0" \
  -DCMAKE_C_FLAGS="-fno-stack-protector -D_FORTIFY_SOURCE=0"

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

> **Note:** `ros2 topic delay` only works on stamped messages (with a `header` field). `std_msgs/String` has no header, so use `ping` for network latency and `ros2 topic hz` for ROS2 publish rate.

```bash
# [PC-1] Measure network round-trip latency to PC-2
ping -c 20 <PC2_IP>
# Excellent: < 5ms (Ethernet or 5GHz WiFi close to router)
# Good:      5–20ms (5GHz WiFi)
# Acceptable: 20–100ms (2.4GHz WiFi) — fine for this project since
#             Ollama inference (500-2000ms) dominates the command loop
# Problematic: > 100ms or packet loss > 1% (check router/interference)
```

```bash
# [PC-1] Verify ROS2 message rate from PC-2
ros2 topic hz /network_test_reverse
# Should show ~1.0 Hz (matching the --rate 1 publisher on PC-2)
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
        "model": "qwen3.5:2b",
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
| > 800ms | Async NLU + reduce model context (`num_ctx=8192`) or use `qwen3.5:1.5b` |

**If async NLU is needed** (add this to your notes — we implement it in Part 9):
- The NLU node has two threads: inference thread (slow, runs SLM) and setpoint thread (fast, 10Hz)
- Setpoint thread always publishes the last valid intent
- Inference thread updates the intent when a new result is ready
- This satisfies PX4 offboard timing regardless of SLM speed

---
---

## Section 2: ROS2 Package Scaffold

### 2.1 Create workspace and package structure

```bash
# [PC-1]
cd ~/major_ws/src
ros2 pkg create major_project \
  --build-type ament_python \
  --dependencies rclpy px4_msgs std_msgs sensor_msgs geometry_msgs
```

```bash
# [PC-1]
cd ~/major_ws/src/major_project/major_project

mkdir -p common
mkdir -p lead_pilot
mkdir -p wingman_pilot
mkdir -p gcs
mkdir -p benchmark

# Create __init__.py in every directory
touch common/__init__.py
touch lead_pilot/__init__.py
touch wingman_pilot/__init__.py
touch gcs/__init__.py

# Create placeholder files
touch common/schemas.py
touch common/normaliser.py
touch common/confidence_gate.py
touch common/ollama_client.py
touch common/tool_registry.py
touch common/context_manager.py
touch common/agent_memory.py
touch lead_pilot/lead_px4_commander_node.py
touch lead_pilot/lead_sensor_aggregator_node.py
touch lead_pilot/lead_intent_bridge_node.py
touch lead_pilot/lead_agent_node.py
touch lead_pilot/safety_monitor_node.py
touch wingman_pilot/wingman_px4_commander_node.py
touch wingman_pilot/wingman_sensor_aggregator_node.py
touch wingman_pilot/wingman_agent_node.py
touch gcs/stt_node.py
touch gcs/clarification_speaker_node.py
touch gcs/mission_monitor_node.py
touch gcs/emergency_stop_node.py
touch gcs/camera_detection_node.py

cd ~/major_ws/src/major_project
mkdir -p launch config
touch launch/lead_pilot.launch.py
touch launch/wingman_pilot.launch.py
touch config/lead_config.yaml
touch config/wingman_config.yaml
```

### 2.2 requirements.txt

```bash
cat << 'EOF' > ~/major_ws/src/major_project/requirements.txt
# Python dependencies — install with: pip3 install -r requirements.txt
pydantic>=2.0,<3.0
faster-whisper>=1.0.0
requests>=2.28.0
numpy>=1.24.0
ultralytics>=8.0.0
rich>=13.0.0
pyaudio>=0.2.13
sounddevice>=0.4.6
scipy>=1.10.0
empy==3.3.4
EOF
```

```bash
# Install from requirements.txt:
pip3 install -r ~/major_ws/src/major_project/requirements.txt
```

### 2.3 setup.py (final)

```bash
# [PC-1]
cat << 'EOF' > ~/major_ws/src/major_project/setup.py
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'major_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Devrajsinh Gohil',
    maintainer_email='202511004@dau.ac.in',
    description='Rank-based multi-SLM drone pilot system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stt_node = major_project.gcs.stt_node:main',
            'clarification_speaker = major_project.gcs.clarification_speaker_node:main',
            'mission_monitor = major_project.gcs.mission_monitor_node:main',
            'emergency_stop = major_project.gcs.emergency_stop_node:main',
            'camera_detection = major_project.lead_pilot.camera_detection_node:main',
            'lead_sensor_aggregator = major_project.lead_pilot.lead_sensor_aggregator_node:main',
            'lead_px4_commander = major_project.lead_pilot.lead_px4_commander_node:main',
            'lead_intent_bridge = major_project.lead_pilot.lead_intent_bridge_node:main',
            'lead_agent = major_project.lead_pilot.lead_agent_node:main',
            'safety_monitor = major_project.lead_pilot.safety_monitor_node:main',
            'wingman_sensor_aggregator = major_project.wingman_pilot.wingman_sensor_aggregator_node:main',
            'wingman_px4_commander = major_project.wingman_pilot.wingman_px4_commander_node:main',
            'wingman_agent = major_project.wingman_pilot.wingman_agent_node:main',
        ],
    },
)
EOF
```

### 2.4 package.xml

```bash
# [PC-1] package.xml is generated by ros2 pkg create — verify it contains:
# <depend>rclpy</depend>
# <depend>px4_msgs</depend>
# <depend>std_msgs</depend>
# <depend>sensor_msgs</depend>
# <depend>geometry_msgs</depend>
# If not present, add them inside the <package> block:
cat << 'EOF' > ~/major_ws/src/major_project/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>major_project</name>
  <version>0.1.0</version>
  <description>Rank-based multi-SLM drone pilot system</description>
  <maintainer email="202511004@dau.ac.in">Devrajsinh Gohil</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>px4_msgs</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

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

---

## Section 3: Common Modules

### 3.1 Pydantic Schemas (schemas.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/schemas.py
"""
All Pydantic v2 schemas for the multi-drone SLM pilot system.
FlightIntent: ported from minor project (unchanged).
WingmanOrder, StatusReport, SituationalAwareness: new for major project.
"""
from __future__ import annotations
from typing import Optional, Literal
from pydantic import BaseModel, Field, field_validator
from dataclasses import dataclass
import json


# ─────────────────────────────────────────────
# Existing schema from minor project
# ─────────────────────────────────────────────

class FlightIntent(BaseModel):
    action: Literal[
        "takeoff", "move", "hover", "land", "rtl",
        "search", "search_stop", "search_resume", "search_expand",
        "hold", "follow_lead"
    ]
    altitude: Optional[float] = Field(None, ge=0.5, le=50.0,
        description="Target altitude in metres (0.5–50)")
    distance: Optional[float] = Field(None, ge=0.1, le=100.0,
        description="Distance to travel in metres (0.1–100)")
    direction: Optional[str] = None
    speed: Optional[float] = Field(None, ge=0.1, le=10.0,
        description="Speed in m/s (0.1–10)")
    heading: Optional[float] = Field(None, ge=0.0, le=360.0,
        description="Target heading in degrees (0–360)")
    then: Optional[FlightIntent] = None
    confidence: Literal["high", "medium", "low"]
    clarification_question: Optional[str] = None

    @field_validator('direction')
    @classmethod
    def validate_direction(cls, v):
        if v is None:
            return v
        valid = {'north','south','east','west','northeast','northwest',
                 'southeast','southwest','forward','backward','left','right','up','down'}
        if v.lower() not in valid:
            return None
        return v.lower()


# Pydantic v2 requires model_rebuild() for self-referential types (then: Optional["FlightIntent"])
FlightIntent.model_rebuild()


# ─────────────────────────────────────────────
# New schemas for multi-drone major project
# ─────────────────────────────────────────────

class DronePosition(BaseModel):
    x: float = 0.0          # metres, NED frame
    y: float = 0.0
    z: float = 0.0           # negative = up in NED
    heading: float = 0.0     # degrees, 0=north
    speed: float = 0.0       # m/s


class SituationalAwareness(BaseModel):
    """Structured sensor state per drone. Injected into SLM prompt as text."""
    drone_id: str            # "LEAD" or "WINGMAN"
    position: DronePosition
    battery_pct: float = Field(ge=0.0, le=100.0)
    flight_mode: str         # PX4 mode string
    gps_fix: bool = True
    altitude_baro: float = 0.0
    camera_summary: str = "No camera data"

    def to_prompt_block(self) -> str:
        pos = self.position
        return (
            f"[DRONE | {self.drone_id}] "
            f"pos:({pos.x:.1f},{pos.y:.1f},{pos.z:.1f}m) "
            f"hdg:{pos.heading:.0f}° spd:{pos.speed:.1f}m/s "
            f"bat:{self.battery_pct:.0f}% mode:{self.flight_mode} "
            f"baro:{self.altitude_baro:.1f}m gps:{'OK' if self.gps_fix else 'NO'}\n"
            f"[CAMERA | {self.drone_id}] {self.camera_summary}"
        )


class WingmanOrder(BaseModel):
    """Lead Pilot → Wingman. Schema-validated command. Never free-form NL only."""
    order_id: Optional[str] = None  # SLM may omit; Lead NLU assigns if missing
    mission_context: str     # NL brief: WHY this order was issued
    intent: FlightIntent     # the actual structured command
    priority: Literal["routine", "urgent", "emergency"]
    lead_position: Optional[DronePosition] = None
    confidence: Literal["high", "medium", "low"]
    clarification_question: Optional[str] = None

    def to_prompt_block(self) -> str:
        intent_str = self.intent.model_dump_json(exclude_none=True)
        return (
            f"[ORDER from LEAD | id:{self.order_id} priority:{self.priority.upper()}]\n"
            f"Context: {self.mission_context}\n"
            f"Command: {intent_str}\n"
            f"Lead confidence: {self.confidence}"
            + (f"\nLead asks: {self.clarification_question}" if self.clarification_question else "")
        )


class StatusReport(BaseModel):
    """Wingman → Lead. Reports execution status and current situation."""
    order_id: str
    status: Literal[
        "acknowledged", "executing", "completed", "failed", "needs_clarification"
    ]
    drone_position: DronePosition
    battery_pct: float = Field(ge=0.0, le=100.0)
    obstacle_detected: bool = False
    obstacle_description: Optional[str] = None
    situation_summary: str   # NL summary for lead's context window
    clarification_question: Optional[str] = None
    confidence: Literal["high", "medium", "low"]

    def to_prompt_block(self) -> str:
        return (
            f"[WINGMAN REPORT | order:{self.order_id} status:{self.status.upper()}]\n"
            f"{self.situation_summary}"
            + (f"\nWingman asks: {self.clarification_question}" if self.clarification_question else "")
            + (f"\nObstacle: {self.obstacle_description}" if self.obstacle_detected else "")
        )


class LeadOutput(BaseModel):
    """Full JSON output from Lead SLM per inference cycle."""
    my_intent: Optional[FlightIntent] = None
    wingman_order: Optional[WingmanOrder] = None
    confidence: Literal["high", "medium", "low"]
    situation_report: str    # NL summary for GCS display (the "radio chatter")
    clarification_question: Optional[str] = None


class WingmanOutput(BaseModel):
    """Full JSON output from Wingman SLM per inference cycle."""
    intent: Optional[FlightIntent] = None
    confidence: Literal["high", "medium", "low"]
    situation_summary: str
    clarification_question: Optional[str] = None


# ─────────────────────────────────────────────
# Safety event (from Part 5 safety monitor)
# ─────────────────────────────────────────────

@dataclass
class SafetyEvent:
    drone_id: str
    event_type: str   # "low_battery", "gps_lost", "geofence", "obstacle"
    severity: str     # "warning", "critical"
    message: str


# ─────────────────────────────────────────────
# Compact value expansion (TypeFly-style token reduction)
# SLM outputs abbreviated values; these maps expand them before Pydantic.
# Backward-compatible: full strings (e.g. "high") pass through unchanged.
# ─────────────────────────────────────────────

_CONF_EXPAND: dict[str, str] = {"H": "high", "M": "medium", "L": "low"}
_DIR_EXPAND: dict[str, str] = {
    "N": "north", "S": "south", "E": "east", "W": "west",
    "NE": "northeast", "NW": "northwest", "SE": "southeast", "SW": "southwest",
    "FWD": "forward", "BCK": "backward", "L": "left", "R": "right",
    "UP": "up", "DN": "down",
}
_PRI_EXPAND: dict[str, str] = {"R": "routine", "U": "urgent", "E": "emergency"}


def expand_compact_values(data):
    """
    Expand abbreviated SLM output values to full strings before Pydantic validation.
    Applied recursively to handle nested FlightIntent inside WingmanOrder, and lists.
    """
    if isinstance(data, list):
        return [expand_compact_values(item) for item in data]
    if not isinstance(data, dict):
        return data
    out = {}
    for k, v in data.items():
        if isinstance(v, dict):
            out[k] = expand_compact_values(v)
        elif isinstance(v, list):
            out[k] = expand_compact_values(v)
        elif k == "confidence" and isinstance(v, str):
            out[k] = _CONF_EXPAND.get(v, v)
        elif k == "direction" and isinstance(v, str):
            out[k] = _DIR_EXPAND.get(v, v)
        elif k == "priority" and isinstance(v, str):
            out[k] = _PRI_EXPAND.get(v, v)
        else:
            out[k] = v
    return out


# ─────────────────────────────────────────────
# Serialisation helpers
# ─────────────────────────────────────────────

def parse_lead_output(raw_json: str) -> Optional[LeadOutput]:
    """Parse and validate Lead SLM output. Returns None on failure."""
    try:
        data = expand_compact_values(json.loads(raw_json))
        return LeadOutput(**data)
    except Exception:
        return None

def parse_wingman_output(raw_json: str) -> Optional[WingmanOutput]:
    """Parse and validate Wingman SLM output. Returns None on failure."""
    try:
        data = expand_compact_values(json.loads(raw_json))
        return WingmanOutput(**data)
    except Exception:
        return None
EOF
```

### 3.2 Ollama Client (ollama_client.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/ollama_client.py
"""
Thin wrapper around Ollama REST API.
Handles timeout, retry, and JSON extraction.
"""
import requests
import json
import time
import logging

logger = logging.getLogger(__name__)


class OllamaClient:
    def __init__(self, host: str = "localhost", port: int = 11434,
                 model: str = "qwen3.5:2b",
                 num_ctx: int = 8192, max_retries: int = 3,
                 timeout: float = 15.0):
        self.url = f"http://{host}:{port}/api/generate"
        self.model = model
        self.num_ctx = num_ctx
        self.max_retries = max_retries
        self.timeout = timeout

    def infer(self, prompt: str, system: str) -> tuple[str | None, float]:
        """
        Run inference. Returns (json_string_or_None, latency_seconds).
        Retries up to max_retries on failure.
        """
        payload = {
            "model": self.model,
            "prompt": prompt,
            "system": system,
            "stream": False,
            "format": "json",
            "options": {
                "num_ctx": self.num_ctx,
                "temperature": 0,
                "top_p": 1.0,
                "repeat_penalty": 1.0,
            }
        }

        for attempt in range(self.max_retries):
            t_start = time.perf_counter()
            try:
                response = requests.post(
                    self.url, json=payload, timeout=self.timeout)
                latency = time.perf_counter() - t_start

                if response.status_code == 200:
                    raw = response.json().get("response", "")
                    # Ollama with format=json should return valid JSON
                    # but sometimes wraps it — try to extract
                    raw = raw.strip()
                    if raw.startswith("{"):
                        return raw, latency
                    # Try to find JSON object in response
                    start = raw.find("{")
                    end = raw.rfind("}") + 1
                    if start >= 0 and end > start:
                        return raw[start:end], latency
                    logger.warning(f"No JSON found in response: {raw[:100]}")
                else:
                    logger.warning(f"Ollama returned {response.status_code}")

            except requests.Timeout:
                logger.warning(f"Ollama timeout (attempt {attempt+1}/{self.max_retries})")
            except Exception as e:
                logger.warning(f"Ollama error: {e} (attempt {attempt+1}/{self.max_retries})")

            if attempt < self.max_retries - 1:
                time.sleep(0.5)

        return None, 0.0
EOF
```

### 3.3 Confidence Gate (confidence_gate.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/confidence_gate.py
"""
Confidence gate policy for both Lead and Wingman pilots.
Implements the two-level cascade described in the architecture doc.
"""
from enum import Enum

class LeadAction(Enum):
    EXECUTE = "execute"
    EXECUTE_WITH_WARNING = "execute_with_warning"
    WITHHOLD_CLARIFY_HUMAN = "withhold_clarify_human"

class WingmanAction(Enum):
    EXECUTE = "execute"
    EXECUTE_WITH_WARNING = "execute_with_warning"
    CLARIFY_LEAD = "clarify_lead"   # wingman never contacts human directly

def gate_lead(confidence: str) -> LeadAction:
    """
    Lead Pilot gate:
      high   → execute my_intent + send wingman_order immediately
      medium → execute + warn GCS, send wingman_order with warning
      low    → withhold, request clarification from Human Commander
    """
    if confidence == "high":
        return LeadAction.EXECUTE
    elif confidence == "medium":
        return LeadAction.EXECUTE_WITH_WARNING
    else:  # low
        return LeadAction.WITHHOLD_CLARIFY_HUMAN

def gate_wingman(confidence: str) -> WingmanAction:
    """
    Wingman gate:
      high   → execute immediately, report back to lead
      medium → execute with assumption, flag in status report
      low    → withhold, send clarification request to Lead
    """
    if confidence == "high":
        return WingmanAction.EXECUTE
    elif confidence == "medium":
        return WingmanAction.EXECUTE_WITH_WARNING
    else:  # low
        return WingmanAction.CLARIFY_LEAD
EOF
```

### 3.4 Normaliser (normaliser.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/normaliser.py
"""
Normalises SLM output to canonical action names.
The SLM often uses variant spellings; this maps them all to schema values.
"""

ACTION_ALIASES: dict[str, str] = {
    # takeoff variants
    "take_off": "takeoff", "take off": "takeoff", "launch": "takeoff",
    "liftoff": "takeoff", "lift_off": "takeoff", "ascend": "takeoff",
    "go up": "takeoff", "fly up": "takeoff",
    # move variants
    "fly": "move", "go": "move", "navigate": "move", "travel": "move",
    "proceed": "move", "advance": "move", "translate": "move",
    # hover variants
    "stop": "hover", "halt": "hover", "stay": "hover", "wait": "hover",
    "hold": "hover", "pause": "hover", "maintain": "hover",
    # land variants
    "landing": "land", "touch down": "land", "touchdown": "land",
    "descend and land": "land", "set down": "land",
    # rtl variants
    "return": "rtl", "come back": "rtl", "go home": "rtl",
    "return to home": "rtl", "return to launch": "rtl", "rth": "rtl",
    # search variants
    "scan": "search", "survey": "search", "look": "search",
    "inspect": "search", "investigate": "search", "patrol": "search",
    "recon": "search", "reconnaissance": "search",
    # hold (wingman)
    "hold position": "hold", "hold_position": "hold",
    "stay in place": "hold", "remain": "hold",
    # follow
    "follow": "follow_lead", "trail": "follow_lead", "shadow": "follow_lead",
}

DIRECTION_ALIASES: dict[str, str] = {
    "n": "north", "s": "south", "e": "east", "w": "west",
    "ne": "northeast", "nw": "northwest", "se": "southeast", "sw": "southwest",
    "fwd": "forward", "back": "backward", "bwd": "backward",
    "ahead": "forward", "behind": "backward",
}

def normalise_action(action: str) -> str:
    """Map any action alias to its canonical schema value."""
    if action is None:
        return "hover"
    cleaned = action.strip().lower().replace("-", " ").replace("_", " ")
    # Direct canonical match
    canonical = {
        "takeoff", "move", "hover", "land", "rtl",
        "search", "search_stop", "search_resume", "search_expand",
        "hold", "follow_lead"
    }
    cleaned_underscore = cleaned.replace(" ", "_")
    if cleaned_underscore in canonical:
        return cleaned_underscore
    # Alias lookup
    return ACTION_ALIASES.get(cleaned, ACTION_ALIASES.get(cleaned_underscore, "hover"))

def normalise_direction(direction: str) -> str:
    if direction is None:
        return None
    cleaned = direction.strip().lower()
    return DIRECTION_ALIASES.get(cleaned, cleaned)

def normalise_parsed(data: dict) -> dict:
    """
    Apply all normalisations to a raw SLM output dict before Pydantic validation.
    Call this BEFORE passing to FlightIntent(**data).
    """
    if "action" in data:
        data["action"] = normalise_action(data["action"])
    if "direction" in data:
        data["direction"] = normalise_direction(data["direction"])
    # Normalise nested 'then' chain recursively
    if "then" in data and isinstance(data["then"], dict):
        data["then"] = normalise_parsed(data["then"])
    return data
EOF
```

### 3.5 Tool Registry (common/tool_registry.py)

```bash
# [PC-1]
touch ~/major_ws/src/major_project/major_project/common/tool_registry.py
touch ~/major_ws/src/major_project/major_project/common/context_manager.py
touch ~/major_ws/src/major_project/major_project/common/agent_memory.py
```

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/tool_registry.py
"""
Tool registry for the drone agent loop.

Each tool has a name, compact description (injected into system prompt),
parameter spec, and an execute() callable. Tools are split into:
  BaseToolRegistry  — flight + sensing + memory (shared by Lead and Wingman)
  LeadToolRegistry  — adds human comms + wingman messaging
  WingmanToolRegistry — adds lead comms (never contacts human directly)

Tool execute functions return a short result STRING that goes directly
back into the agent's context window as the tool result.
"""
from __future__ import annotations
import json
import time
import math
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional


@dataclass
class Tool:
    description: str          # shown in system prompt
    params: dict[str, str]    # param_name → "type: description"
    execute: Callable         # (params: dict) → str


class BaseToolRegistry:
    """
    Common tools for both Lead and Wingman agents.
    Subclasses inject a ROS2Interface (duck-typed) to access publishers.
    """

    def __init__(self, ros_iface):
        self.ros = ros_iface   # the agent node itself
        self.tools: dict[str, Tool] = {}
        self._register_base_tools()

    def _register_base_tools(self):
        self.tools.update({
            "takeoff": Tool(
                description="Arm drone and ascend to altitude metres (1–30)",
                params={"altitude": "float: target altitude in metres"},
                execute=self._takeoff),

            "move": Tool(
                description="Fly direction (N/S/E/W/NE/NW/SE/SW) for distance metres. Optional altitude change.",
                params={
                    "direction": "str: N S E W NE NW SE SW",
                    "distance": "float: metres (1–100)",
                    "altitude": "float: new altitude in metres (optional)"},
                execute=self._move),

            "hover": Tool(
                description="Hold current position indefinitely",
                params={},
                execute=self._hover),

            "search": Tool(
                description="Hold position and scan camera for duration_sec seconds (5–60). Returns all detections.",
                params={"duration_sec": "int: scan duration (5–60)"},
                execute=self._search),

            "land": Tool(
                description="Land drone at current position",
                params={},
                execute=self._land),

            "rtl": Tool(
                description="Return to launch point and land",
                params={},
                execute=self._rtl),

            "get_situation": Tool(
                description="Read full sensor state: position, altitude, battery, GPS, flight mode, camera",
                params={},
                execute=self._get_situation),

            "scan_camera": Tool(
                description="Get current camera detections with direction and distance estimates",
                params={},
                execute=self._scan_camera),

            "get_battery": Tool(
                description="Get current battery percentage for both drones",
                params={},
                execute=self._get_battery),

            "remember": Tool(
                description="Store a fact in long-term memory for later recall",
                params={"fact": "str: the fact to store"},
                execute=self._remember),

            "recall": Tool(
                description="Retrieve stored facts matching a keyword query",
                params={"query": "str: keyword to search memory"},
                execute=self._recall),

            "wait": Tool(
                description="Pause for seconds before next action (1–30)",
                params={"seconds": "int: seconds to wait"},
                execute=self._wait),

            "mission_complete": Tool(
                description="Declare mission accomplished. Ends the agent loop.",
                params={"report": "str: full mission completion summary"},
                execute=self._mission_complete),
        })

    # ── Flight tools ─────────────────────────────────────────────

    def _publish_intent(self, action_dict: dict):
        """Publish FlightIntent to approved_intent topic."""
        import json as _json
        from std_msgs.msg import String as _String
        msg = _String()
        msg.data = _json.dumps(action_dict)
        self.ros.pub_intent.publish(msg)

    def _takeoff(self, params: dict) -> str:
        altitude = float(params.get('altitude', 5.0))
        altitude = max(1.0, min(30.0, altitude))
        self._publish_intent({
            'action': 'takeoff', 'altitude': altitude, 'confidence': 'high'})
        return f"Takeoff initiated. Ascending to {altitude}m. Allow ~{int(altitude*2)+5}s."

    def _move(self, params: dict) -> str:
        dir_map = {
            'N': 'north', 'S': 'south', 'E': 'east', 'W': 'west',
            'NE': 'northeast', 'NW': 'northwest',
            'SE': 'southeast', 'SW': 'southwest',
            'FWD': 'forward', 'BCK': 'backward', 'L': 'left', 'R': 'right',
        }
        raw_dir = str(params.get('direction', 'N')).upper()
        direction = dir_map.get(raw_dir, raw_dir.lower())
        distance = float(params.get('distance', 10.0))
        distance = max(1.0, min(100.0, distance))
        altitude = params.get('altitude', None)

        intent = {'action': 'move', 'direction': direction,
                  'distance': distance, 'confidence': 'high'}
        if altitude is not None:
            intent['altitude'] = float(altitude)

        self._publish_intent(intent)
        eta = max(8, int(distance / 2.0) + 3)
        alt_note = f" Changing altitude to {altitude}m." if altitude else ""
        return (f"Moving {direction} {distance}m.{alt_note} "
                f"ETA ~{eta}s. Call wait({eta}) then get_situation().")

    def _hover(self, params: dict) -> str:
        self._publish_intent({'action': 'hover', 'confidence': 'high'})
        return "Hovering at current position."

    def _search(self, params: dict) -> str:
        duration = int(params.get('duration_sec', 15))
        duration = max(5, min(60, duration))
        # Issue hover to stop movement
        self._publish_intent({'action': 'hover', 'confidence': 'high'})
        time.sleep(1.0)
        # Accumulate camera detections over the scan duration
        observations = []
        end_time = time.time() + duration
        while time.time() < end_time:
            time.sleep(2.0)
            cam = getattr(self.ros, 'camera_summary', '')
            obs = getattr(self.ros, 'obstacle_vector', '')
            if cam and 'no ' not in cam.lower() and 'clear' not in cam.lower():
                entry = cam
                if obs:
                    entry += f" [{obs}]"
                if entry not in observations:
                    observations.append(entry)
        if observations:
            return (f"Search complete ({duration}s). "
                    f"Detected: {' | '.join(observations)}")
        return f"Search complete ({duration}s). Area clear — no detections."

    def _land(self, params: dict) -> str:
        self._publish_intent({'action': 'land', 'confidence': 'high'})
        return "Land command sent. Allow ~15s to touch down."

    def _rtl(self, params: dict) -> str:
        self._publish_intent({'action': 'rtl', 'confidence': 'high'})
        return "RTL initiated. Drone returning to launch point. Allow ~40s."

    # ── Sensing tools ─────────────────────────────────────────────

    def _get_situation(self, params: dict) -> str:
        with self.ros.lock:
            return self.ros.own_situation or "No situation data yet."

    def _scan_camera(self, params: dict) -> str:
        with self.ros.lock:
            cam = self.ros.camera_summary
            obs = self.ros.obstacle_vector
        if not cam:
            return "Camera not available."
        result = cam
        if obs:
            result += f"\nObstacle vectors: {obs}"
        return result

    def _get_battery(self, params: dict) -> str:
        with self.ros.lock:
            own = self.ros.battery_pct
            other = getattr(self.ros, 'other_battery_pct', None)
        result = f"Own drone battery: {own:.0f}%"
        if other is not None:
            result += f" | Other drone battery: {other:.0f}%"
        return result

    # ── Memory tools ──────────────────────────────────────────────

    def _remember(self, params: dict) -> str:
        fact = str(params.get('fact', '')).strip()
        if not fact:
            return "Error: no fact provided to remember."
        self.ros.agent_memory.remember(fact)
        return f"Remembered: '{fact[:80]}'"

    def _recall(self, params: dict) -> str:
        query = str(params.get('query', '')).strip()
        facts = self.ros.agent_memory.recall(query)
        if not facts:
            return "No memories found matching that query."
        return "Recalled: " + " | ".join(facts[:5])

    # ── Control tools ─────────────────────────────────────────────

    def _wait(self, params: dict) -> str:
        secs = int(params.get('seconds', 5))
        secs = max(1, min(30, secs))
        time.sleep(secs)
        return f"Waited {secs}s."

    def _mission_complete(self, params: dict) -> str:
        report = str(params.get('report', 'Mission accomplished.'))
        self.ros._mission_done = True
        self.ros._mission_report = report
        return f"MISSION COMPLETE: {report}"

    # ── Schema for system prompt ──────────────────────────────────

    def schema_block(self) -> str:
        """Compact tool list for injection into system prompt."""
        lines = []
        for name, tool in self.tools.items():
            param_str = ", ".join(
                f"{k}:{v.split(':')[0].strip()}"
                for k, v in tool.params.items()
            ) if tool.params else "—"
            lines.append(f"  {name}({param_str})  {tool.description}")
        return "\n".join(lines)

    def is_valid(self, tool_name: str) -> bool:
        return tool_name in self.tools

    def execute(self, tool_name: str, params: dict) -> str:
        if tool_name not in self.tools:
            valid = list(self.tools.keys())
            return f"Unknown tool '{tool_name}'. Valid tools: {valid}"
        try:
            return self.tools[tool_name].execute(params)
        except Exception as e:
            return f"Tool '{tool_name}' error: {str(e)[:120]}"


class LeadToolRegistry(BaseToolRegistry):
    """Lead Pilot tools — adds human comms + wingman coordination."""

    def __init__(self, ros_iface):
        super().__init__(ros_iface)
        self._register_lead_tools()

    def _register_lead_tools(self):
        self.tools.update({
            "message_wingman": Tool(
                description="Send a direct message to Wingman agent (non-blocking)",
                params={"message": "str: message text"},
                execute=self._message_wingman),

            "ask_human": Tool(
                description="Ask Ground Commander a question. BLOCKS until human responds (max 120s).",
                params={"question": "str: question for the human"},
                execute=self._ask_human),

            "notify_human": Tool(
                description="Send a status message to GCS. Non-blocking — human may or may not reply.",
                params={"message": "str: status message"},
                execute=self._notify_human),
        })

    def _message_wingman(self, params: dict) -> str:
        from std_msgs.msg import String as _String
        msg = _String()
        msg.data = str(params.get('message', ''))
        self.ros.pub_wingman_msg.publish(msg)
        return f"Sent to Wingman: '{params.get('message','')[:60]}'"

    def _ask_human(self, params: dict) -> str:
        from std_msgs.msg import String as _String
        question = str(params.get('question', 'Please advise.'))
        q_msg = _String()
        q_msg.data = question
        self.ros.pub_clarification.publish(q_msg)
        # Block waiting for voice response
        self.ros._human_response = None
        self.ros._waiting_for_human = True
        self.ros._human_event.clear()
        answered = self.ros._human_event.wait(timeout=120.0)
        self.ros._waiting_for_human = False
        if answered and self.ros._human_response:
            return f"Human responded: '{self.ros._human_response}'"
        return "No human response (120s timeout). Proceeding with best judgment."

    def _notify_human(self, params: dict) -> str:
        from std_msgs.msg import String as _String
        message = str(params.get('message', ''))
        msg = _String()
        msg.data = f"[LEAD] {message}"
        self.ros.pub_clarification.publish(msg)
        return f"GCS notified: '{message[:60]}'"


class WingmanToolRegistry(BaseToolRegistry):
    """Wingman tools — lead comms only, no human contact."""

    def __init__(self, ros_iface):
        super().__init__(ros_iface)
        self._register_wingman_tools()

    def _register_wingman_tools(self):
        self.tools.update({
            "message_lead": Tool(
                description="Send a message to Lead agent (non-blocking)",
                params={"message": "str: message text"},
                execute=self._message_lead),

            "ask_lead": Tool(
                description="Ask Lead agent a question. BLOCKS until Lead responds (max 60s).",
                params={"question": "str: question for Lead"},
                execute=self._ask_lead),

            "notify_lead": Tool(
                description="Send a status update to Lead agent (non-blocking)",
                params={"message": "str: status message"},
                execute=self._notify_lead),
        })

    def _message_lead(self, params: dict) -> str:
        from std_msgs.msg import String as _String
        msg = _String()
        msg.data = str(params.get('message', ''))
        self.ros.pub_lead_msg.publish(msg)
        return f"Sent to Lead: '{params.get('message','')[:60]}'"

    def _ask_lead(self, params: dict) -> str:
        import json as _json
        from std_msgs.msg import String as _String
        question = str(params.get('question', ''))
        payload = _json.dumps({"type": "query", "content": question})
        msg = _String()
        msg.data = payload
        self.ros.pub_lead_msg.publish(msg)
        # Wait for Lead to respond via /agent/lead_to_wingman
        self.ros._lead_response = None
        self.ros._waiting_for_lead = True
        self.ros._lead_event.clear()
        answered = self.ros._lead_event.wait(timeout=60.0)
        self.ros._waiting_for_lead = False
        if answered and self.ros._lead_response:
            return f"Lead responded: '{self.ros._lead_response}'"
        return "No Lead response (60s timeout). Proceeding with best judgment."

    def _notify_lead(self, params: dict) -> str:
        import json as _json
        from std_msgs.msg import String as _String
        message = str(params.get('message', ''))
        payload = _json.dumps({"type": "status", "content": message})
        msg = _String()
        msg.data = payload
        self.ros.pub_lead_msg.publish(msg)
        return f"Lead notified: '{message[:60]}'"
EOF
```

### 3.6 Context Manager (common/context_manager.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/context_manager.py
"""
Context Manager — keeps the agent's context window bounded.

The context fed to the SLM each inference cycle is:
  [GOAL]            current mission goal (~20 tok)
  [SITUATION]       latest sensor block (~80 tok)
  [MEMORY]          compressed old history + recalled facts (~100 tok)
  [INTER-AGENT]     recent messages from other agent (~50 tok)
  [RECENT ACTIONS]  last MAX_HISTORY tool calls + results (~600 tok)
  [NEXT ACTION]     output anchor line (~10 tok)
Total: ~860 tokens + 300 tok system prompt + 50 tok output = ~1210 tok (fits 2048)

When RECENT ACTIONS exceeds MAX_HISTORY entries, the oldest COMPRESS_BATCH
entries are compressed into a one-line summary and moved to [MEMORY].
"""

MAX_HISTORY   = 8   # max tool-call entries before compression
COMPRESS_BATCH = 4  # how many to compress at once


class ContextManager:

    def __init__(self):
        self.goal          = ""    # current mission goal
        self.situation     = ""    # latest situation string
        self.memory_block  = ""    # compressed history + recalled facts
        self.inter_agent   = []    # recent messages from other agent (max 4)
        self.history       = []    # list of {tool, params, result} dicts

    # ── Update methods ─────────────────────────────────────────────

    def set_goal(self, goal: str):
        self.goal = goal

    def update_situation(self, situation: str):
        self.situation = situation

    def add_inter_agent_message(self, source: str, content: str):
        self.inter_agent.append(f"[{source}] {content}")
        if len(self.inter_agent) > 4:
            self.inter_agent.pop(0)

    def add_memory_note(self, note: str):
        """Inject a recalled fact or important event into the memory block."""
        self.memory_block = f"{note}\n{self.memory_block}".strip()
        # Keep memory block bounded (~200 chars)
        if len(self.memory_block) > 400:
            self.memory_block = self.memory_block[:400] + "…"

    def add_tool_result(self, tool: str, params: dict, result: str):
        """Record a completed tool call. Compress if history is too long."""
        params_str = ", ".join(
            f"{k}={str(v)[:20]}" for k, v in params.items()
        ) if params else ""
        self.history.append({
            "tool": tool,
            "params_str": params_str,
            "result": result[:120]   # cap each result to 120 chars
        })
        if len(self.history) > MAX_HISTORY:
            self._compress_oldest()

    def clear_history(self):
        """Call when a mission ends and a new one starts."""
        self.history = []
        self.memory_block = ""
        self.inter_agent  = []
        self.goal = ""

    # ── Compression ────────────────────────────────────────────────

    def _compress_oldest(self):
        batch = self.history[:COMPRESS_BATCH]
        self.history = self.history[COMPRESS_BATCH:]
        parts = []
        for e in batch:
            r = e['result'][:40].replace('\n', ' ')
            if e['params_str']:
                parts.append(f"{e['tool']}({e['params_str'][:30]})→{r}")
            else:
                parts.append(f"{e['tool']}()→{r}")
        compressed = "Earlier: " + " | ".join(parts)
        # Prepend to memory block
        self.memory_block = (compressed + "\n" + self.memory_block).strip()
        if len(self.memory_block) > 600:
            self.memory_block = self.memory_block[:600] + "…"

    # ── Prompt building ────────────────────────────────────────────

    def build_prompt(self) -> str:
        """Assemble the full context prompt for the next SLM inference."""
        parts = []

        if self.goal:
            parts.append(f"[MISSION GOAL]\n{self.goal}")

        if self.situation:
            parts.append(f"[CURRENT SITUATION]\n{self.situation}")

        if self.memory_block:
            parts.append(f"[MEMORY]\n{self.memory_block}")

        if self.inter_agent:
            parts.append("[MESSAGES FROM OTHER AGENT]\n" +
                         "\n".join(self.inter_agent))

        if self.history:
            lines = []
            for e in self.history:
                call_str = (f"→ {e['tool']}({e['params_str']})"
                            if e['params_str']
                            else f"→ {e['tool']}()")
                lines.append(call_str)
                lines.append(f"← {e['result']}")
            parts.append("[RECENT ACTIONS]\n" + "\n".join(lines))

        parts.append("[NEXT ACTION] Output one tool call JSON:")
        return "\n\n".join(parts)
EOF
```

### 3.7 Agent Memory (common/agent_memory.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/agent_memory.py
"""
Agent Memory — SQLite-backed long-term remember/recall.

Provides two operations the SLM can invoke as tools:
  remember(fact)  → stores a timestamped string
  recall(query)   → returns matching facts (keyword search, newest first)

Persistent across node restarts. One DB file per drone role.
"""
import sqlite3
import threading
import time
import os


class AgentMemory:

    def __init__(self, db_name: str = "lead_agent_memory.db"):
        db_dir = os.path.expanduser("~/.ros")
        os.makedirs(db_dir, exist_ok=True)
        self.db_path = os.path.join(db_dir, db_name)
        self._lock = threading.Lock()
        self._init_db()

    def _init_db(self):
        with self._lock:
            conn = sqlite3.connect(self.db_path)
            conn.execute("""
                CREATE TABLE IF NOT EXISTS memory (
                    id        INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    fact      TEXT NOT NULL
                )
            """)
            conn.commit()
            conn.close()

    def remember(self, fact: str):
        with self._lock:
            conn = sqlite3.connect(self.db_path)
            conn.execute(
                "INSERT INTO memory (timestamp, fact) VALUES (?, ?)",
                (time.time(), fact.strip()))
            conn.commit()
            conn.close()

    def recall(self, query: str = "", limit: int = 6) -> list[str]:
        with self._lock:
            conn = sqlite3.connect(self.db_path)
            if query:
                rows = conn.execute(
                    "SELECT fact FROM memory WHERE fact LIKE ? "
                    "ORDER BY timestamp DESC LIMIT ?",
                    (f"%{query}%", limit)
                ).fetchall()
            else:
                rows = conn.execute(
                    "SELECT fact FROM memory ORDER BY timestamp DESC LIMIT ?",
                    (limit,)
                ).fetchall()
            conn.close()
        return [r[0] for r in rows]

    def get_recent(self, n: int = 5) -> list[str]:
        return self.recall(query="", limit=n)

    def clear(self):
        """Wipe the memory DB — use only for testing."""
        with self._lock:
            conn = sqlite3.connect(self.db_path)
            conn.execute("DELETE FROM memory")
            conn.commit()
            conn.close()
EOF
```

---

## Section 4: GCS Nodes

### 4.1 Speech-to-Text Node (stt_node.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/stt_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
import queue
import threading
from faster_whisper import WhisperModel


class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.pub = self.create_publisher(String, '/voice_commands', 10)

        # Load Whisper model
        self.get_logger().info("Loading Faster-Whisper tiny.en (int8)...")
        self.model = WhisperModel("tiny.en", device="cpu",
                                   compute_type="int8")
        self.get_logger().info("STT model loaded.")

        self.audio_queue = queue.Queue()
        self.sample_rate = 16000
        self.chunk_duration = 0.5   # seconds per chunk
        self.chunk_samples = int(self.sample_rate * self.chunk_duration)
        self.buffer = []
        self.silence_threshold = 0.01
        self.min_speech_duration = 1.0   # seconds
        self.silence_after_speech = 1.5  # seconds of silence to end utterance

        self.is_speaking = False
        self.silence_chunks = 0
        self.silence_chunks_needed = int(
            self.silence_after_speech / self.chunk_duration)

        # Start audio capture thread
        self.capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        self.get_logger().info("Listening for voice commands... (speak clearly)")

    def _audio_callback(self, indata, frames, time_info, status):
        self.audio_queue.put(indata.copy())

    def _capture_loop(self):
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32',
            blocksize=self.chunk_samples,
            callback=self._audio_callback
        ):
            while rclpy.ok():
                try:
                    chunk = self.audio_queue.get(timeout=1.0)
                    rms = np.sqrt(np.mean(chunk**2))

                    if rms > self.silence_threshold:
                        self.is_speaking = True
                        self.silence_chunks = 0
                        self.buffer.append(chunk)
                    elif self.is_speaking:
                        self.buffer.append(chunk)
                        self.silence_chunks += 1
                        if self.silence_chunks >= self.silence_chunks_needed:
                            # Utterance complete — transcribe
                            audio = np.concatenate(self.buffer, axis=0).flatten()
                            duration = len(audio) / self.sample_rate
                            if duration >= self.min_speech_duration:
                                self._transcribe(audio)
                            self.buffer = []
                            self.is_speaking = False
                            self.silence_chunks = 0
                except queue.Empty:
                    pass

    def _transcribe(self, audio: np.ndarray):
        try:
            segments, info = self.model.transcribe(
                audio, beam_size=5, language="en",
                condition_on_previous_text=False)
            text = " ".join(seg.text.strip() for seg in segments).strip()
            if text and len(text) > 2:
                self.get_logger().info(f"Transcribed: '{text}'")
                msg = String()
                msg.data = text
                self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 4.2 Clarification Speaker Node (clarification_speaker_node.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/clarification_speaker_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os


class ClarificationSpeakerNode(Node):
    def __init__(self):
        super().__init__('clarification_speaker_node')
        self.sub = self.create_subscription(
            String, '/clarification_request', self.on_clarification, 10)
        self.use_tts = os.path.exists('/usr/bin/espeak-ng')
        self.get_logger().info("Clarification Speaker ready.")

    def on_clarification(self, msg: String):
        question = msg.data
        # Print clearly to terminal
        print("\n" + "="*60)
        print("  CLARIFICATION NEEDED FROM GROUND COMMANDER:")
        print(f"  {question}")
        print("="*60 + "\n")
        # Optionally speak it
        if self.use_tts:
            try:
                subprocess.Popen(
                    ['espeak-ng', '-s', '150', question],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ClarificationSpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 4.3 Mission Monitor Node (mission_monitor_node.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/mission_monitor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.panel import Panel
from rich.layout import Layout
from rich.text import Text
import threading
import time
import json


class MissionMonitorNode(Node):
    def __init__(self):
        super().__init__('mission_monitor_node')

        self.drone0_situation = "No data"
        self.drone1_situation = "No data"
        self.last_command = "—"
        self.lead_status = "—"
        self.wingman_status = "—"
        self.mission_log = []
        self.lock = threading.Lock()

        self.sub_d0 = self.create_subscription(
            String, '/drone_0/situation', self.on_drone0, 10)
        self.sub_d1 = self.create_subscription(
            String, '/drone_1/situation', self.on_drone1, 10)
        self.sub_cmd = self.create_subscription(
            String, '/voice_commands', self.on_command, 10)
        self.sub_mission = self.create_subscription(
            String, '/mission_status', self.on_mission_status, 10)

        # Start display thread
        self.display_thread = threading.Thread(
            target=self._display_loop, daemon=True)
        self.display_thread.start()

    def on_drone0(self, msg):
        with self.lock:
            self.drone0_situation = msg.data

    def on_drone1(self, msg):
        with self.lock:
            self.drone1_situation = msg.data

    def on_command(self, msg):
        with self.lock:
            self.last_command = msg.data
            self.mission_log.append(f"[CMD] {msg.data}")
            self.mission_log = self.mission_log[-10:]

    def on_mission_status(self, msg):
        with self.lock:
            try:
                data = json.loads(msg.data)
                self.lead_status = data.get('lead', '—')
                self.wingman_status = data.get('wingman', '—')
            except Exception:
                self.lead_status = msg.data

    def _display_loop(self):
        console = Console()
        with Live(console=console, refresh_per_second=2) as live:
            while rclpy.ok():
                with self.lock:
                    d0 = self.drone0_situation
                    d1 = self.drone1_situation
                    cmd = self.last_command
                    lead_s = self.lead_status
                    wingman_s = self.wingman_status
                    log = list(self.mission_log)

                table = Table(show_header=True, header_style="bold cyan",
                              title="MISSION CONTROL STATION",
                              title_style="bold white on blue")
                table.add_column("Field", style="bold", width=20)
                table.add_column("DRONE-0 (LEAD)", width=35)
                table.add_column("DRONE-1 (WINGMAN)", width=35)
                table.add_row("Situation", d0, d1)
                table.add_row("SLM Status", lead_s, wingman_s)
                table.add_row("Last Command", cmd, "—")

                log_text = "\n".join(log[-5:]) if log else "No commands yet"
                panel = Panel(table, subtitle=f"Log: {log_text[:100]}")
                live.update(panel)
                time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 4.4 Emergency Stop Node (emergency_stop_node.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/emergency_stop_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import threading
import sys


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.pub_stop = self.create_publisher(Bool, '/emergency_stop', 10)
        # Also listen for voice emergency
        self.sub_cmd = self.create_subscription(
            String, '/voice_commands', self.on_voice, 10)

        print("\n[EMERGENCY STOP NODE]")
        print("  Type 'STOP' and press Enter to emergency-land ALL drones")
        print("  OR say 'emergency land' / 'abort all' via voice")
        print()

        self.keyboard_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()

    def _keyboard_loop(self):
        while True:
            try:
                line = input()
                if line.strip().upper() in ('STOP', 'ABORT', 'EMERGENCY', 'E'):
                    self._trigger_stop("keyboard")
            except EOFError:
                break

    def on_voice(self, msg: String):
        text = msg.data.lower()
        triggers = ['emergency', 'abort all', 'emergency land',
                    'stop all', 'all land', 'kill']
        if any(t in text for t in triggers):
            self._trigger_stop(f"voice: '{msg.data}'")

    def _trigger_stop(self, source: str):
        self.get_logger().error(f"EMERGENCY STOP triggered by {source}")
        msg = Bool()
        msg.data = True
        # Publish multiple times to ensure delivery
        for _ in range(5):
            self.pub_stop.publish(msg)
        print(f"\n!!! EMERGENCY STOP SENT (source: {source}) !!!\n")


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

---
## Section 5: Sensor & Detection Layer

### 5.1 Camera Detection Node (camera_detection_node.py)

The camera detection node subscribes to a USB/CSI camera, runs YOLOv8-nano inference at a configurable rate, and publishes two topics:

- `/camera_0/detections` — a human-readable text summary of all detected objects (class name, confidence, direction, and distance tier), used by the Lead Sensor Aggregator to populate the SLM situation block.
- `/camera_0/obstacle_vector` — a compact structured string of the form `label:direction:distance ...` (e.g., `person:ahead:very_close car:left:medium`) containing only the obstacle-class labels that the SLM should reason about for routing decisions.

Distance is estimated monocularly using bounding-box width as a proxy for range. The `_bbox_distance` helper maps the fraction of the frame occupied by the bounding box to one of four tiers: `very_close` (> 30% of frame, approximately 1–3 m), `close` (10–30%), `medium` (3–10%), and `far` (< 3%). Lateral position of the bounding-box centre determines direction (`left`, `ahead`, `right`) via `_bbox_direction`. When the camera or YOLOv8 model is unavailable, the node falls back to publishing a static "not available" message and an empty obstacle vector so downstream nodes degrade gracefully.

```python
"""
Camera Detection Node — Part 5 version.

Adds to Part 2 version:
- Bbox-size monocular distance estimation (near/medium/far heuristic)
- Obstacle direction from bbox center offset
- /camera_0/obstacle_vector topic: "label:direction:distance ..." for each detection
  Example: "person:left:close car:ahead:medium" → injected into SituationalAwareness

No depth sensor required. Resolution: coarse (3 tiers) but sufficient for the
SLM to reason about routing decisions at drone speeds. For precise avoidance,
a depth camera or stereo rig would be needed — this is explicitly noted as a
limitation in the research paper.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

try:
    import cv2
    import numpy as np
    from ultralytics import YOLO
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False

# Distance thresholds (fraction of frame width occupied by bbox)
DIST_THRESHOLDS = [
    (0.30, "very_close"),   # > 30% of frame → ~1-3m
    (0.10, "close"),         # 10–30% → ~3-8m
    (0.03, "medium"),        # 3–10% → ~8-20m
    (0.00, "far"),           # < 3% → > 20m
]

# Direction bins based on bbox center x-position in frame
def _bbox_direction(cx: float, frame_w: float) -> str:
    rel = (cx - frame_w / 2) / (frame_w / 2)   # -1.0 (left) to +1.0 (right)
    if rel < -0.35:
        return "left"
    elif rel > 0.35:
        return "right"
    return "ahead"

def _bbox_distance(bbox_w: float, frame_w: float) -> str:
    fraction = bbox_w / max(frame_w, 1)
    for threshold, label in DIST_THRESHOLDS:
        if fraction >= threshold:
            return label
    return "far"


class CameraDetectionNode(Node):

    def __init__(self):
        super().__init__('camera_detection_node')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('obstacle_labels',
                               ['person', 'car', 'truck', 'bicycle', 'bird', 'tree'])

        self.pub_detections = self.create_publisher(String, '/camera_0/detections', 10)
        self.pub_obstacle   = self.create_publisher(String, '/camera_0/obstacle_vector', 10)
        self._running = True

        if not VISION_AVAILABLE:
            self.get_logger().warning(
                "OpenCV or ultralytics not installed — camera in fallback mode.")
            self.create_timer(2.0, self._publish_fallback)
            return

        cam_idx    = self.get_parameter('camera_index').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        rate_hz    = self.get_parameter('publish_rate_hz').value
        self.obstacle_labels = set(self.get_parameter('obstacle_labels').value)

        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.get_logger().warning(
                f"Camera {cam_idx} not available — fallback mode.")
            self.cap = None
            self.create_timer(2.0, self._publish_fallback)
            return

        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLOv8 loaded: {model_path}")
        except Exception as e:
            self.get_logger().warning(f"YOLO load failed ({e}) — fallback mode.")
            self.model = None
            self.cap.release()
            self.create_timer(2.0, self._publish_fallback)
            return

        self._detect_thread = threading.Thread(
            target=self._detect_loop, args=(1.0 / rate_hz,), daemon=True)
        self._detect_thread.start()
        self.get_logger().info(
            f"Camera detection + distance heuristic running at {rate_hz}Hz")

    def _publish_fallback(self):
        fallback_msg = String()
        fallback_msg.data = "Camera not available. Sensor data only."
        self.pub_detections.publish(fallback_msg)
        # Publish empty obstacle vector
        ov_msg = String()
        ov_msg.data = ""
        self.pub_obstacle.publish(ov_msg)

    def _detect_loop(self, period: float):
        while rclpy.ok() and self._running:
            ret, frame = self.cap.read()
            if not ret:
                msg = String()
                msg.data = "Camera read error."
                self.pub_detections.publish(msg)
                time.sleep(period)
                continue

            h, w = frame.shape[:2]
            results = self.model(frame, conf=self.conf_threshold, verbose=False)

            detections = []      # human-readable summary
            obstacle_parts = []  # structured "label:direction:distance" list

            for result in results:
                for box in result.boxes:
                    cls_id   = int(box.cls[0])
                    cls_name = self.model.names.get(cls_id, str(cls_id))
                    conf     = float(box.conf[0])
                    xyxy     = box.xyxy[0].tolist()

                    bbox_w = xyxy[2] - xyxy[0]
                    cx     = (xyxy[0] + xyxy[2]) / 2.0

                    dist_label = _bbox_distance(bbox_w, w)
                    direction  = _bbox_direction(cx, w)

                    detections.append(f"{cls_name}({conf:.0%}) {direction} {dist_label}")

                    # Only include known obstacle types in the vector
                    if cls_name in self.obstacle_labels:
                        obstacle_parts.append(f"{cls_name}:{direction}:{dist_label}")

            # ── Publish text summary ──────────────────────────────
            det_summary = ", ".join(detections) if detections else "Clear — no detections"
            det_msg = String()
            det_msg.data = det_summary
            self.pub_detections.publish(det_msg)

            # ── Publish structured obstacle vector ────────────────
            ov_msg = String()
            ov_msg.data = " ".join(obstacle_parts)
            self.pub_obstacle.publish(ov_msg)

            time.sleep(period)

    def destroy_node(self):
        self._running = False
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 5.2 Lead Sensor Aggregator (lead_sensor_aggregator_node.py)

The Lead Sensor Aggregator runs on PC-1 and aggregates all Drone-0 (Lead) telemetry into a single `/drone_0/situation` text block published at 1 Hz. The Lead NLU node injects this block into every SLM prompt.

It subscribes to:
- `/fmu/out/vehicle_local_position` — NED position, velocity, GPS fix flag
- `/fmu/out/vehicle_status` — arming state and nav/flight mode
- `/fmu/out/battery_status` — battery remaining fraction
- `/camera_0/detections` — human-readable detection summary from the camera node
- `/camera_0/obstacle_vector` — structured obstacle distance/direction string
- `/mission/plan` — to mark the mission phase as `executing` when a new plan arrives
- `/mission/step_assessment` — to mark the phase as `complete` when the SLM declares `done`

Beyond raw telemetry, the node provides temporal enrichment: it records the home position on the first GPS fix and calculates distance from home (with cardinal direction) and mission elapsed time. This implements structured temporal context injection so the frozen SLM can reason about time and position relative to the mission start without requiring any fine-tuning, following the Memory-T1 ablation approach. The published situation string includes an `elapsed`, `dist_home`, and `phase` field in every update.

```python
"""
Lead Sensor Aggregator — Part 5 version.

Adds to Part 3 version:
- Temporal enrichment: mission elapsed time, distance from home, mission phase
- Obstacle vector: subscribes to /camera_0/obstacle_vector for distance-annotated detections
- Mission phase tracking (idle → executing → complete)

Temporal reasoning basis: Memory-T1 ablation shows RL training adds 15% but
requires training (not possible on frozen edge model). Structured context
injection ("T+4m32s, 120m NW of home") achieves equivalent grounding zero-shot.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus
import math
import time
import json

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

CARDINAL = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']


def bearing_to_cardinal(deg: float) -> str:
    idx = int((deg + 22.5) / 45.0) % 8
    return CARDINAL[idx]


class LeadSensorAggregatorNode(Node):

    def __init__(self):
        super().__init__('lead_sensor_aggregator_node')

        # ── PX4 telemetry subscriptions ───────────────────────────
        self.sub_pos = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.on_position, BEST_EFFORT_QOS)
        self.sub_status = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.on_status, BEST_EFFORT_QOS)
        self.sub_battery = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.on_battery, BEST_EFFORT_QOS)

        # ── Camera detections (text summary from camera_detection_node) ─
        self.sub_camera = self.create_subscription(
            String, '/camera_0/detections', self.on_camera, 10)
        # ── Obstacle vector (distance-annotated from Part 25) ─────
        self.sub_obstacle = self.create_subscription(
            String, '/camera_0/obstacle_vector', self.on_obstacle, 10)
        # ── Mission status (to update mission phase) ──────────────
        self.sub_mission = self.create_subscription(
            String, '/mission/plan', self.on_mission_start, 10)
        self.sub_step_complete = self.create_subscription(
            String, '/mission/step_assessment',
            lambda m: self._check_mission_done(m), 10)

        # ── Output ────────────────────────────────────────────────
        self.pub_situation = self.create_publisher(String, '/drone_0/situation', 10)

        # ── State ─────────────────────────────────────────────────
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                    'speed': 0.0, 'heading': 0.0, 'alt_baro': 0.0}
        self.battery_pct = 0.0
        self.flight_mode = "UNKNOWN"
        self.arming_state = "DISARMED"
        self.gps_fix = False
        self.camera_summary = "No camera data"
        self.obstacle_vector = ""

        # ── Temporal state ─────────────────────────────────────────
        self.home_x: float | None = None
        self.home_y: float | None = None
        self.mission_start_time: float | None = None
        self.mission_phase = "idle"   # idle | executing | complete

        # ── 1 Hz publish timer ────────────────────────────────────
        self.create_timer(1.0, self.publish_situation)
        self.get_logger().info("Lead sensor aggregator started (Part 5 — temporal enrichment)")

    # ── Telemetry callbacks ──────────────────────────────────────

    def on_position(self, msg: VehicleLocalPosition):
        speed = math.sqrt(msg.vx**2 + msg.vy**2)
        heading = math.degrees(math.atan2(msg.vy, msg.vx)) % 360
        self.pos = {
            'x': round(msg.x, 1),
            'y': round(msg.y, 1),
            'z': round(msg.z, 1),
            'alt_baro': round(-msg.z, 1),
            'speed': round(speed, 1),
            'heading': round(heading, 0),
        }
        self.gps_fix = msg.xy_global

        # Record home position on first GPS fix
        if self.home_x is None and self.gps_fix:
            self.home_x = msg.x
            self.home_y = msg.y
            self.get_logger().info(
                f"Home position recorded: ({self.home_x:.1f}, {self.home_y:.1f})")

    def on_status(self, msg: VehicleStatus):
        self.arming_state = "ARMED" if msg.arming_state == 2 else "DISARMED"
        nav_map = {
            14: "OFFBOARD", 2: "POSITION", 1: "MANUAL",
            3: "ALTITUDE", 17: "AUTO_TAKEOFF", 12: "LOITER", 0: "MANUAL"
        }
        self.flight_mode = nav_map.get(msg.nav_state, f"MODE_{msg.nav_state}")

    def on_battery(self, msg: BatteryStatus):
        self.battery_pct = round(msg.remaining * 100.0, 1)

    def on_camera(self, msg: String):
        self.camera_summary = msg.data

    def on_obstacle(self, msg: String):
        self.obstacle_vector = msg.data   # e.g. "person:left:close person:ahead:medium"

    def on_mission_start(self, msg: String):
        self.mission_phase = "executing"
        self.mission_start_time = time.time()

    def _check_mission_done(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get('decision') == 'done':
                self.mission_phase = "complete"
        except Exception:
            pass

    # ── Temporal helpers ─────────────────────────────────────────

    def _elapsed_str(self) -> str:
        if self.mission_start_time is None:
            return "—"
        secs = int(time.time() - self.mission_start_time)
        return f"{secs // 60}m{secs % 60}s"

    def _dist_from_home_str(self) -> str:
        if self.home_x is None:
            return "home unknown"
        dx = self.pos['x'] - self.home_x
        dy = self.pos['y'] - self.home_y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 3.0:
            return "at home"
        bearing = math.degrees(math.atan2(dy, dx)) % 360
        return f"{dist:.0f}m {bearing_to_cardinal(bearing)}"

    # ── Situational awareness publication ────────────────────────

    def publish_situation(self):
        alt = self.pos.get('alt_baro', 0.0)
        speed = self.pos.get('speed', 0.0)
        heading = self.pos.get('heading', 0.0)
        x, y = self.pos.get('x', 0.0), self.pos.get('y', 0.0)

        text = (
            f"pos:({x:.1f},{y:.1f}) alt:{alt:.1f}m hdg:{heading:.0f}° spd:{speed:.1f}m/s "
            f"bat:{self.battery_pct:.0f}% "
            f"mode:{self.flight_mode} {self.arming_state} "
            f"gps:{'OK' if self.gps_fix else 'NO'}\n"
            f"camera:{self.camera_summary}"
        )
        if self.obstacle_vector:
            text += f"\nobstacles:{self.obstacle_vector}"

        # ── Temporal enrichment ──────────────────────────────────
        text += (
            f"\ntemporal:elapsed={self._elapsed_str()} "
            f"dist_home={self._dist_from_home_str()} "
            f"phase={self.mission_phase}"
        )

        msg = String()
        msg.data = text
        self.pub_situation.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeadSensorAggregatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 5.3 Wingman Sensor Aggregator (wingman_sensor_aggregator_node.py)

The Wingman Sensor Aggregator mirrors the Lead's aggregator but subscribes to Drone-1 telemetry via the `/px4_1/` namespace and publishes to `/drone_1/situation`. It runs on PC-2 as part of the wingman stack. The published situation string is consumed by the Wingman NLU node and is also forwarded over WiFi DDS to PC-1 where the Lead NLU incorporates it into the full situational awareness block.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus
import math

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class WingmanSensorAggregatorNode(Node):
    def __init__(self):
        super().__init__('wingman_sensor_aggregator_node')

        ns = '/px4_1'  # Drone-1 namespace

        self.sub_pos = self.create_subscription(
            VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position',
            self.on_position, BEST_EFFORT_QOS)
        self.sub_status = self.create_subscription(
            VehicleStatus, f'{ns}/fmu/out/vehicle_status',
            self.on_status, BEST_EFFORT_QOS)
        self.sub_battery = self.create_subscription(
            BatteryStatus, f'{ns}/fmu/out/battery_status',
            self.on_battery, BEST_EFFORT_QOS)
        self.sub_camera = self.create_subscription(
            String, '/camera_1/detections', self.on_camera, 10)

        self.pub_situation = self.create_publisher(
            String, '/drone_1/situation', 10)

        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'speed': 0.0, 'heading': 0.0}
        self.battery_pct = 0.0
        self.flight_mode = "UNKNOWN"
        self.arming_state = "DISARMED"
        self.gps_fix = False
        self.camera_summary = "No camera data"

        self.timer = self.create_timer(1.0, self.publish_situation)
        self.get_logger().info("Wingman sensor aggregator started (Drone-1)")

    def on_position(self, msg: VehicleLocalPosition):
        speed = math.sqrt(msg.vx**2 + msg.vy**2)
        heading = math.degrees(math.atan2(msg.vy, msg.vx)) % 360
        self.pos = {
            'x': round(msg.x, 1), 'y': round(msg.y, 1), 'z': round(msg.z, 1),
            'alt': round(-msg.z, 1), 'speed': round(speed, 1),
            'heading': round(heading, 0),
        }
        self.gps_fix = msg.xy_global

    def on_status(self, msg: VehicleStatus):
        self.arming_state = "ARMED" if msg.arming_state == 2 else "DISARMED"
        nav_map = {14: "OFFBOARD", 2: "POSITION", 12: "LOITER", 1: "MANUAL"}
        self.flight_mode = nav_map.get(msg.nav_state, f"MODE_{msg.nav_state}")

    def on_battery(self, msg: BatteryStatus):
        self.battery_pct = round(msg.remaining * 100.0, 1)

    def on_camera(self, msg: String):
        self.camera_summary = msg.data

    def publish_situation(self):
        text = (
            f"pos:({self.pos.get('x',0):.1f},{self.pos.get('y',0):.1f},"
            f"{self.pos.get('z',0):.1f}m) "
            f"alt:{self.pos.get('alt',0):.1f}m "
            f"hdg:{self.pos.get('heading',0):.0f}° "
            f"spd:{self.pos.get('speed',0):.1f}m/s "
            f"bat:{self.battery_pct:.0f}% "
            f"mode:{self.flight_mode} {self.arming_state}\n"
            f"camera:{self.camera_summary}"
        )
        msg = String()
        msg.data = text
        self.pub_situation.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WingmanSensorAggregatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Section 6: Safety Monitor

### 6.1 Safety Monitor Node (safety_monitor_node.py)

The Safety Monitor is an independent hard-rule node that operates entirely outside the SLM inference path. It monitors battery level and vehicle status for both drones simultaneously and takes life-critical actions directly — the SLM is never consulted for safety triggers. This hybrid architecture (SLM handles mission reasoning; hard-rule monitor handles critical thresholds) follows the REAL system (arXiv:2311.01403) proactive safety-trigger pattern.

Two configurable thresholds govern battery behaviour:

- `battery_warn_pct` (default 20 %): publishes a `battery_warning` event to `/safety/event` and speaks a warning to `/clarification_request` so the operator is informed. No flight action is taken.
- `battery_rtl_pct` (default 15 %): publishes a `battery_rtl` critical event, speaks an RTL notification, and issues a `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` `VehicleCommand` directly to the affected drone's PX4 input topic. The command is sent three times for belt-and-suspenders reliability even though `RELIABLE_QOS` is used on the publisher.

Per-drone state flags (`_warned`, `_rtl_sent`, `_gps_warned`) ensure each threshold triggers only once per drone per flight, preventing message storms. The node subscribes to both `/fmu/out/battery_status` (Drone-0) and `/px4_1/fmu/out/battery_status` (Drone-1) and publishes VehicleCommands to the corresponding `/fmu/in/vehicle_command` or `/px4_1/fmu/in/vehicle_command` topics.

```python
"""
Safety Monitor Node — hard-rule autonomous safety triggers.

Monitors battery and GPS for both drones independently of the SLM.
Issues RTL VehicleCommands directly to PX4 when critical thresholds are crossed.
SLM is NOT consulted for safety-critical actions — this is intentional.

Thresholds (configurable via YAML):
  battery_warn_pct  = 20%  → publish warning to /safety/event
  battery_rtl_pct   = 15%  → force RTL + publish critical event
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import BatteryStatus, VehicleStatus, VehicleCommand
import threading
import json

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor_node')

        # Parameters
        self.declare_parameter('battery_warn_pct', 20.0)
        self.declare_parameter('battery_rtl_pct', 15.0)
        self.warn_pct = self.get_parameter('battery_warn_pct').value
        self.rtl_pct  = self.get_parameter('battery_rtl_pct').value

        # Per-drone state flags (prevent repeated triggers)
        self._warned  = {'LEAD': False, 'WINGMAN': False}
        self._rtl_sent = {'LEAD': False, 'WINGMAN': False}
        self._gps_warned = {'LEAD': False, 'WINGMAN': False}
        self._lock = threading.Lock()

        # ── Battery subscribers ────────────────────────────────────
        self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            lambda m: self._on_battery(m, 'LEAD'),
            BEST_EFFORT_QOS)

        self.create_subscription(
            BatteryStatus,
            '/px4_1/fmu/out/battery_status',
            lambda m: self._on_battery(m, 'WINGMAN'),
            BEST_EFFORT_QOS)

        # ── GPS/status subscribers ─────────────────────────────────
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            lambda m: self._on_status(m, 'LEAD'),
            BEST_EFFORT_QOS)

        self.create_subscription(
            VehicleStatus,
            '/px4_1/fmu/out/vehicle_status',
            lambda m: self._on_status(m, 'WINGMAN'),
            BEST_EFFORT_QOS)

        # ── VehicleCommand publishers (direct to PX4 — bypasses NLU) ─
        self._cmd_lead = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', RELIABLE_QOS)
        self._cmd_wingman = self.create_publisher(
            VehicleCommand, '/px4_1/fmu/in/vehicle_command', RELIABLE_QOS)

        # ── Safety event publisher (for GCS and memory node) ──────
        self._pub_event = self.create_publisher(String, '/safety/event', 10)
        # ── GCS clarification (speaks warning to operator) ─────────
        self._pub_clarify = self.create_publisher(String, '/clarification_request', 10)

        self.get_logger().info(
            f"Safety monitor active — warn:{self.warn_pct}% RTL:{self.rtl_pct}%")

    # ── Battery handler ──────────────────────────────────────────

    def _on_battery(self, msg: BatteryStatus, drone_id: str):
        pct = msg.remaining * 100.0   # BatteryStatus.remaining is 0.0–1.0

        with self._lock:
            if pct <= self.rtl_pct and not self._rtl_sent[drone_id]:
                self._rtl_sent[drone_id] = True
                self._trigger_rtl(drone_id, pct)

            elif pct <= self.warn_pct and not self._warned[drone_id]:
                self._warned[drone_id] = True
                self._publish_event(
                    "battery_warning", drone_id, "warning",
                    f"{drone_id} battery at {pct:.0f}% — approaching RTL threshold",
                    pct)
                warn_msg = String()
                warn_msg.data = (
                    f"[SAFETY] {drone_id} battery {pct:.0f}%. "
                    f"RTL will trigger automatically at {self.rtl_pct:.0f}%.")
                self._pub_clarify.publish(warn_msg)

    # ── GPS/status handler ───────────────────────────────────────

    def _on_status(self, msg: VehicleStatus, drone_id: str):
        # VehicleStatus.vehicle_type: if GPS-denied, nav_state falls out of OFFBOARD
        # We detect loss of position estimate via arming_state / failure_detector
        # Simple heuristic: if drone was armed and now arming_state goes to disarmed
        # mid-mission, something went wrong. For demo: detect GPS fix from VehicleLocalPosition
        # instead (handled in sensor aggregator). Here we just check for pre-arm failure.
        pass  # GPS monitoring extended in lead_sensor_aggregator (Part 24)

    # ── RTL trigger (hard safety action) ───────────────────────

    def _trigger_rtl(self, drone_id: str, pct: float):
        self.get_logger().error(
            f"SAFETY: {drone_id} battery critical ({pct:.0f}%) — forcing RTL")

        cmd_pub = self._cmd_lead if drone_id == 'LEAD' else self._cmd_wingman
        self._send_vehicle_command(cmd_pub, VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

        self._publish_event(
            "battery_rtl", drone_id, "critical",
            f"{drone_id} battery {pct:.0f}% — RTL initiated automatically",
            pct)

        critical_msg = String()
        critical_msg.data = (
            f"[SAFETY CRITICAL] {drone_id} battery {pct:.0f}%. "
            f"RTL initiated automatically. No human action required.")
        self._pub_clarify.publish(critical_msg)

    # ── Helpers ──────────────────────────────────────────────────

    def _send_vehicle_command(self, pub, command: int,
                               param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # Publish 3 times — RELIABLE_QOS but belt-and-suspenders for safety
        for _ in range(3):
            pub.publish(msg)

    def _publish_event(self, event_type: str, drone_id: str,
                        severity: str, message: str, value: float = 0.0):
        event = {
            "event_type": event_type,
            "drone_id": drone_id,
            "severity": severity,
            "message": message,
            "value": round(value, 1)
        }
        msg = String()
        msg.data = json.dumps(event)
        self._pub_event.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---
## Section 7: Flight Execution Layer

### 7.1 Lead PX4 Commander (lead_px4_commander_node.py)

The Lead PX4 Commander is the lowest-level flight control node for Drone-0. It subscribes to `/lead/approved_intent`, which carries a `FlightIntent` JSON payload produced by the lead intelligence layer (NLU node or agent loop). On each received intent it translates the high-level action into PX4-native commands: it switches the vehicle into `OFFBOARD` mode, arms it, and drives the trajectory setpoint publisher at 10 Hz to keep the mode alive. All position setpoints are expressed in the NED (North-East-Down) local frame used by PX4 — altitude is therefore stored as a negative Z value.

The `move` action supports an optional `altitude` field. When present, the drone climbs or descends to the new altitude while translating; when absent, the current Z is preserved.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleLocalPosition
)
import json
import math
import threading

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class LeadPX4CommanderNode(Node):

    DIRECTION_OFFSETS = {
        'north':     ( 1.0,  0.0),
        'south':     (-1.0,  0.0),
        'east':      ( 0.0,  1.0),
        'west':      ( 0.0, -1.0),
        'northeast': ( 0.707,  0.707),
        'northwest': ( 0.707, -0.707),
        'southeast': (-0.707,  0.707),
        'southwest': (-0.707, -0.707),
        'forward':   ( 1.0,  0.0),
        'backward':  (-1.0,  0.0),
        'left':      ( 0.0, -1.0),
        'right':     ( 0.0,  1.0),
    }

    def __init__(self):
        super().__init__('lead_px4_commander_node')

        # Publishers (Drone-0 namespace = default /fmu/)
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', BEST_EFFORT_QOS)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', BEST_EFFORT_QOS)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', RELIABLE_QOS)

        # Current position (updated from telemetry)
        self.cur_x = 0.0; self.cur_y = 0.0; self.cur_z = 0.0

        # Target setpoint (published at 10 Hz regardless of NLU speed)
        self.target_x = 0.0; self.target_y = 0.0; self.target_z = -0.5
        self.target_yaw = 0.0
        self.armed = False
        self.offboard_active = False
        self.lock = threading.Lock()

        # Subscribe to position telemetry
        self.sub_pos = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.on_position, BEST_EFFORT_QOS)

        # Subscribe to approved intents from the lead intelligence layer
        self.sub_intent = self.create_subscription(
            String, '/lead/approved_intent', self.on_intent, 10)

        # Publish execution errors back for self-correction context
        self.pub_exec_feedback = self.create_publisher(
            String, '/lead/execution_feedback', 10)

        # Subscribe to emergency stop
        from std_msgs.msg import Bool
        self.sub_stop = self.create_subscription(
            Bool, '/emergency_stop', self.on_emergency_stop, 10)

        # 10 Hz keepalive timer (PX4 offboard requires >2 Hz)
        self.timer = self.create_timer(0.1, self.publish_setpoint)

        self.get_logger().info("Lead PX4 Commander ready (Drone-0)")

    def on_position(self, msg: VehicleLocalPosition):
        with self.lock:
            self.cur_x = msg.x
            self.cur_y = msg.y
            self.cur_z = msg.z

    def on_emergency_stop(self, msg):
        if msg.data:
            self.get_logger().error("EMERGENCY STOP — sending land command")
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def on_intent(self, msg: String):
        """Parse FlightIntent JSON and update target setpoint."""
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().error("Failed to parse intent JSON")
            fb = String()
            fb.data = "Intent JSON parse error — check SLM output format."
            self.pub_exec_feedback.publish(fb)
            return

        action = data.get('action', 'hover')
        altitude = data.get('altitude', 5.0) or 5.0
        distance = data.get('distance', 10.0) or 10.0
        direction = data.get('direction', 'north') or 'north'

        with self.lock:
            x, y, z = self.cur_x, self.cur_y, self.cur_z

        if action == 'takeoff':
            target_z = -altitude  # NED: negative = up
            self._arm_and_enable_offboard()
            with self.lock:
                self.target_x = x
                self.target_y = y
                self.target_z = target_z
            self.get_logger().info(f"Takeoff → alt {altitude}m")

        elif action == 'move':
            dx, dy = self.DIRECTION_OFFSETS.get(direction, (1.0, 0.0))
            # Support optional altitude change in move
            new_alt = data.get('altitude', None)
            with self.lock:
                self.target_x = x + dx * distance
                self.target_y = y + dy * distance
                self.target_z = -float(new_alt) if new_alt is not None else z
            self.get_logger().info(
                f"Move {direction} {distance}m → ({self.target_x:.1f},{self.target_y:.1f})")

        elif action == 'hover' or action == 'hold':
            with self.lock:
                self.target_x = x
                self.target_y = y
                self.target_z = z
            self.get_logger().info("Hover at current position")

        elif action == 'land':
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Land command sent")

        elif action == 'rtl':
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.get_logger().info("RTL command sent")

        elif action in ('search', 'search_stop', 'search_resume', 'search_expand'):
            dx, dy = self.DIRECTION_OFFSETS.get(direction, (1.0, 0.0))
            with self.lock:
                self.target_x = x + dx * distance
                self.target_y = y + dy * distance
                self.target_z = z
            self.get_logger().info(f"Search toward {direction}")

        else:
            self.get_logger().warning(f"Unknown action '{action}' — no setpoint sent")
            fb = String()
            fb.data = f"Unknown action '{action}'. Valid: takeoff, move, hover, land, rtl, search, hold."
            self.pub_exec_feedback.publish(fb)

    def _arm_and_enable_offboard(self):
        """Send arm + enable offboard mode commands."""
        # Set offboard mode
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, param2=6.0)
        # Arm
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0)
        self.armed = True
        self.offboard_active = True

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(msg)

    def publish_setpoint(self):
        """10 Hz loop: publish offboard keepalive + trajectory setpoint."""
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        ocm = OffboardControlMode()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.timestamp = now_us
        self.pub_offboard.publish(ocm)

        with self.lock:
            tx, ty, tz, yaw = self.target_x, self.target_y, self.target_z, self.target_yaw

        sp = TrajectorySetpoint()
        sp.position = [tx, ty, tz]
        sp.yaw = yaw
        sp.timestamp = now_us
        self.pub_setpoint.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = LeadPX4CommanderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 7.2 Lead Intent Bridge (lead_intent_bridge_node.py)

The Lead Intent Bridge handles chained `FlightIntent` commands. When the intelligence layer outputs an intent with a `then` field — for example `{"action": "takeoff", ..., "then": {"action": "move", ...}}` — the bridge dispatches each subsequent step automatically after a configurable delay. The `lead_px4_commander` sees and executes the first step immediately (it subscribes to the same `/lead/approved_intent` topic); the bridge takes responsibility for steps 2, 3, and so on.

A private marker key (`__bridge_dispatched__`) is injected into each bridge-dispatched message to prevent the bridge from re-processing its own publications and entering an infinite loop. The commander ignores this extra key because it only reads known fields via `data.get(...)`.

```python
"""
Lead intent bridge: dispatches chained FlightIntents (the 'then' field) sequentially.
Subscribes to /lead/approved_intent.
Republishes subsequent chain steps to /lead/approved_intent after a configurable delay.
The lead_px4_commander always executes the first step immediately (it subscribes to same topic).
This node handles steps 2, 3, ... in the chain.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time

_BRIDGE_MARKER = "__bridge_dispatched__"  # prevents echo-loop


class LeadIntentBridgeNode(Node):
    def __init__(self):
        super().__init__('lead_intent_bridge_node')
        self.declare_parameter('chain_step_delay_sec', 6.0)
        self.step_delay = self.get_parameter('chain_step_delay_sec').value

        self.sub = self.create_subscription(
            String, '/lead/approved_intent', self.on_intent, 10)
        self.pub = self.create_publisher(
            String, '/lead/approved_intent', 10)

        self._chain_thread = None
        self.get_logger().info(
            f"Lead intent bridge ready (chain step delay: {self.step_delay}s)")

    def on_intent(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        # Skip messages we dispatched ourselves (avoids infinite loop on then-chains)
        if data.get(_BRIDGE_MARKER):
            return

        then = data.get('then')
        if then is None:
            return  # no chain — nothing to do

        self.get_logger().info(
            f"Chained intent: {data.get('action')} → {then.get('action')} "
            f"(dispatching step-2 in {self.step_delay}s)")

        if self._chain_thread and self._chain_thread.is_alive():
            self.get_logger().warning(
                "Previous chain interrupted by new command — replacing")

        self._chain_thread = threading.Thread(
            target=self._execute_chain, args=(then,), daemon=True)
        self._chain_thread.start()

    def _execute_chain(self, intent_data: dict):
        """Walk the 'then' linked list, dispatching each step after a delay."""
        current = intent_data
        step = 2
        while current and rclpy.ok():
            time.sleep(self.step_delay)

            # Strip 'then' from the step we're dispatching (bridge handles the rest)
            dispatch = {k: v for k, v in current.items() if k != 'then'}
            dispatch[_BRIDGE_MARKER] = True   # mark so we don't re-process it

            msg = String()
            msg.data = json.dumps(dispatch)
            self.pub.publish(msg)
            self.get_logger().info(
                f"Bridge dispatched chain step {step}: {current.get('action')}")

            current = current.get('then')
            step += 1


def main(args=None):
    rclpy.init(args=args)
    node = LeadIntentBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 7.3 Wingman PX4 Commander (wingman_px4_commander_node.py)

The Wingman PX4 Commander mirrors the Lead commander in structure but controls Drone-1. All PX4 topics use the `/px4_1/fmu/` namespace, and `target_system` in vehicle commands is set to `2` (MAVLink system ID for the second SITL instance). The node runs on PC-2; its setpoint publications cross the DDS WiFi bridge back to Drone-1's SITL on PC-1.

The `move` action supports the same optional `altitude` field as the Lead commander: when provided, the drone simultaneously translates and changes altitude in a single manoeuvre.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleLocalPosition
)
import json
import threading

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class WingmanPX4CommanderNode(Node):

    DIRECTION_OFFSETS = {
        'north': (1.0, 0.0), 'south': (-1.0, 0.0),
        'east': (0.0, 1.0), 'west': (0.0, -1.0),
        'northeast': (0.707, 0.707), 'northwest': (0.707, -0.707),
        'southeast': (-0.707, 0.707), 'southwest': (-0.707, -0.707),
        'forward': (1.0, 0.0), 'backward': (-1.0, 0.0),
        'left': (0.0, -1.0), 'right': (0.0, 1.0),
    }

    def __init__(self):
        super().__init__('wingman_px4_commander_node')

        # All topics use /px4_1/ namespace for Drone-1
        ns = '/px4_1'

        self.pub_offboard = self.create_publisher(
            OffboardControlMode,
            f'{ns}/fmu/in/offboard_control_mode', BEST_EFFORT_QOS)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint,
            f'{ns}/fmu/in/trajectory_setpoint', BEST_EFFORT_QOS)
        self.pub_cmd = self.create_publisher(
            VehicleCommand,
            f'{ns}/fmu/in/vehicle_command', RELIABLE_QOS)

        self.cur_x = 0.0; self.cur_y = 0.0; self.cur_z = 0.0
        self.target_x = 5.0; self.target_y = 0.0; self.target_z = -0.5
        self.target_yaw = 0.0
        self.lock = threading.Lock()

        self.sub_pos = self.create_subscription(
            VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position',
            self.on_position, BEST_EFFORT_QOS)

        self.sub_intent = self.create_subscription(
            String, '/wingman/approved_intent', self.on_intent, 10)

        self.sub_stop = self.create_subscription(
            Bool, '/emergency_stop', self.on_emergency_stop, 10)

        # Publish execution errors back to the wingman intelligence layer
        self.pub_exec_feedback = self.create_publisher(
            String, '/wingman/execution_feedback', 10)

        # 10 Hz keepalive
        self.timer = self.create_timer(0.1, self.publish_setpoint)
        self.get_logger().info("Wingman PX4 Commander ready (Drone-1 /px4_1/)")

    def on_position(self, msg: VehicleLocalPosition):
        with self.lock:
            self.cur_x = msg.x
            self.cur_y = msg.y
            self.cur_z = msg.z

    def on_emergency_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().error("WINGMAN EMERGENCY STOP — landing now")
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def on_intent(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().error("Failed to parse intent JSON")
            fb = String()
            fb.data = "Intent JSON parse error — check SLM output format."
            self.pub_exec_feedback.publish(fb)
            return

        action = data.get('action', 'hover')
        altitude = data.get('altitude', 5.0) or 5.0
        distance = data.get('distance', 10.0) or 10.0
        direction = data.get('direction', 'north') or 'north'

        with self.lock:
            x, y, z = self.cur_x, self.cur_y, self.cur_z

        if action == 'takeoff':
            self._arm_and_enable_offboard()
            with self.lock:
                self.target_x = x
                self.target_y = y
                self.target_z = -altitude
            self.get_logger().info(f"Wingman takeoff → {altitude}m")

        elif action == 'move':
            dx, dy = self.DIRECTION_OFFSETS.get(direction, (1.0, 0.0))
            # Support optional altitude change in move
            new_alt = data.get('altitude', None)
            with self.lock:
                self.target_x = x + dx * distance
                self.target_y = y + dy * distance
                self.target_z = -float(new_alt) if new_alt is not None else z
            self.get_logger().info(f"Wingman move {direction} {distance}m")

        elif action in ('hover', 'hold', 'search_stop'):
            with self.lock:
                self.target_x = x
                self.target_y = y
                self.target_z = z

        elif action == 'land':
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        elif action == 'rtl':
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

        elif action in ('search', 'search_resume', 'search_expand'):
            dx, dy = self.DIRECTION_OFFSETS.get(direction, (1.0, 0.0))
            with self.lock:
                self.target_x = x + dx * distance
                self.target_y = y + dy * distance
                self.target_z = z

        elif action == 'follow_lead':
            # Placeholder — in full impl, subscribe to /drone_0/situation for lead pos
            self.get_logger().info("follow_lead: holding current position (stub)")

        else:
            self.get_logger().warning(f"Unknown action '{action}' — no setpoint sent")
            fb = String()
            fb.data = f"Unknown action '{action}'. Valid: takeoff, move, hover, land, rtl, search, hold, follow_lead."
            self.pub_exec_feedback.publish(fb)

    def _arm_and_enable_offboard(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, param2=6.0)
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0)

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 2      # MAVLink system ID for Drone-1 (SITL instance -i 1)
        # PX4 SITL instance 0 = system ID 1, instance 1 = system ID 2, etc.
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(msg)

    def publish_setpoint(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        ocm = OffboardControlMode()
        ocm.position = True
        ocm.timestamp = now_us
        self.pub_offboard.publish(ocm)

        with self.lock:
            tx, ty, tz, yaw = self.target_x, self.target_y, self.target_z, self.target_yaw

        sp = TrajectorySetpoint()
        sp.position = [tx, ty, tz]
        sp.yaw = yaw
        sp.timestamp = now_us
        self.pub_setpoint.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = WingmanPX4CommanderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---
## Section 8: Lead Pilot Agent

The lead agent is an always-active brain for Drone-0. It runs a continuous
think-act-observe loop: build context → SLM infer → parse tool call →
execute → update context → repeat. It replaces the old NLU + mission
executor + mission memory pattern with a single self-directing agent.

### 8.1 Create prompt directory

```bash
# [PC-1]
mkdir -p ~/major_ws/src/major_project/major_project/lead_pilot/prompts
```

### 8.2 System Prompt (lead_agent_system.txt)

```bash
cat << 'PROMPT_EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/prompts/lead_agent_system.txt
You are LEAD PILOT — an autonomous drone agent controlling Drone-0.
Your Wingman controls Drone-1. Human Ground Commander gives you mission goals.

You run in a think-act loop. At each step output EXACTLY ONE tool call:
{"tool": "<name>", "params": {"key": value, ...}}
No params needed? Use: {"tool": "<name>", "params": {}}

TOOLS:
takeoff(altitude:float)  Move to altitude metres (1–30)
move(direction:str, distance:float)  Fly N/S/E/W/NE/NW/SE/SW distance metres
move(direction:str, distance:float, altitude:float)  Fly and change altitude
hover()  Hold position
search(duration_sec:int)  Scan in place 5–60s, returns detections
land()  Land now
rtl()  Return to launch
get_situation()  Full sensor readout: pos/battery/alt/camera
scan_camera()  Camera detections with distances
get_battery()  Battery for both drones
remember(fact:str)  Store a fact for later
recall(query:str)  Retrieve stored facts
message_wingman(message:str)  Tell Wingman something (non-blocking)
ask_human(question:str)  Ask Ground Commander — WAITS for answer
notify_human(message:str)  Status to GCS — no wait
wait(seconds:int)  Pause 1–30s then continue
mission_complete(report:str)  End mission, full report

AGENT RULES:
- Start every mission with get_situation() to read current state
- After move/takeoff call wait(N) then get_situation() to confirm arrival
- Battery ≤ 20%: notify_human immediately, plan RTL soon
- Battery ≤ 15%: call rtl() immediately (safety_monitor also does this)
- Wingman never contacts human — you are the sole human interface
- Use message_wingman before major manoeuvres so Wingman knows context
- Use ask_human ONLY for: safety, scope expansion, genuine uncertainty
- Use notify_human for routine status (no human reply needed)
- Use remember() for findings, positions of interest, mission decisions
- Begin response with { and end with } — no prose, no markdown

COMPACT DIRECTIONS: N S E W NE NW SE SW

EXAMPLES:
Mission starts → {"tool":"get_situation","params":{}}
Need to take off → {"tool":"takeoff","params":{"altitude":10}}
Move north → {"tool":"move","params":{"direction":"N","distance":50}}
Wait for move → {"tool":"wait","params":{"seconds":25}}
Scan area → {"tool":"search","params":{"duration_sec":20}}
Found football → {"tool":"remember","params":{"fact":"football at pos(50,0) N sector"}}
Tell wingman → {"tool":"message_wingman","params":{"message":"Football found N50m. Cover E sector."}}
Done → {"tool":"mission_complete","params":{"report":"Football found 50m north. Wingman covered east. Both RTL."}}
PROMPT_EOF
```

### 8.3 Lead Agent Node (lead_agent_node.py)

```bash
touch ~/major_ws/src/major_project/major_project/lead_pilot/lead_agent_node.py

cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_agent_node.py
"""
Lead Agent Node — the always-active brain for Drone-0.

Replaces lead_nlu_node + mission_executor + mission_memory from Parts 3–5.

Architecture:
  Voice command / text goal → sets self.ctx.goal → starts agent loop
  Agent loop: build_context → SLM infer → parse tool → execute → repeat
  ask_human() blocks the loop thread until voice response arrives
  message_wingman() publishes to /agent/lead_to_wingman (non-blocking)
  Wingman messages arrive on /agent/wingman_to_lead and land in context
  Safety monitor runs in a separate node — no change needed here

The loop runs in a daemon thread at the rate the SLM can sustain (~0.2–0.5 Hz).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import re
import threading
import time

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from major_project.common.ollama_client import OllamaClient
from major_project.common.tool_registry import LeadToolRegistry
from major_project.common.context_manager import ContextManager
from major_project.common.agent_memory import AgentMemory


def _load_prompt(filename: str) -> str:
    path = os.path.join(os.path.dirname(__file__), 'prompts', filename)
    with open(path) as f:
        return f.read()


class LeadAgentNode(Node):

    def __init__(self):
        super().__init__('lead_agent_node')

        # ── Parameters ─────────────────────────────────────────────
        self.declare_parameter('ollama_host', 'localhost')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('model', 'qwen3.5:2b')
        self.declare_parameter('num_ctx', 8192)
        self.declare_parameter('loop_pause_sec', 0.5)

        host    = self.get_parameter('ollama_host').value
        port    = self.get_parameter('ollama_port').value
        model   = self.get_parameter('model').value
        num_ctx = self.get_parameter('num_ctx').value
        self.loop_pause = self.get_parameter('loop_pause_sec').value

        # ── Ollama + prompts ────────────────────────────────────────
        self.ollama = OllamaClient(host=host, port=port,
                                    model=model, num_ctx=num_ctx)
        self.system_prompt = _load_prompt('lead_agent_system.txt')

        # ── Shared sensor state (written by ROS callbacks, read by tools) ──
        self.lock = threading.Lock()
        self.own_situation   = ""
        self.camera_summary  = ""
        self.obstacle_vector = ""
        self.battery_pct     = 100.0
        self.other_battery_pct: float | None = None

        # ── Human interaction state ─────────────────────────────────
        self._waiting_for_human = False
        self._human_response: str | None = None
        self._human_event = threading.Event()

        # ── Wingman interaction state ───────────────────────────────
        self._wingman_query_pending = False

        # ── Agent state ─────────────────────────────────────────────
        self._agent_running = False
        self._mission_done  = False
        self._mission_report = ""
        self.ctx = ContextManager()
        self.agent_memory = AgentMemory(db_name="lead_agent_memory.db")
        self.tools = LeadToolRegistry(self)

        # ── Publishers ──────────────────────────────────────────────
        self.pub_intent       = self.create_publisher(String, '/lead/approved_intent', 10)
        self.pub_wingman_msg  = self.create_publisher(String, '/agent/lead_to_wingman', 10)
        self.pub_wingman_order = self.create_publisher(String, '/wingman/order', 10)
        self.pub_clarification = self.create_publisher(String, '/clarification_request', 10)
        self.pub_mission_status = self.create_publisher(String, '/mission_status', 10)

        # ── Subscriptions ───────────────────────────────────────────
        self.create_subscription(
            String, '/drone_0/situation', self._on_situation, 10)
        self.create_subscription(
            String, '/camera_0/detections', self._on_camera, 10)
        self.create_subscription(
            String, '/camera_0/obstacle_vector', self._on_obstacle, 10)
        self.create_subscription(
            String, '/voice_commands', self._on_voice, 10)
        self.create_subscription(
            String, '/agent/wingman_to_lead', self._on_wingman_message, 10)
        self.create_subscription(
            String, '/safety/event', self._on_safety_event, 10)

        self.get_logger().info(
            f"Lead Agent ready (Ollama {host}:{port} model:{model})")
        self.get_logger().info(
            "Waiting for mission goal via voice or /voice_commands topic...")

    # ── ROS callbacks ────────────────────────────────────────────

    def _on_situation(self, msg: String):
        with self.lock:
            self.own_situation = msg.data
            # Parse battery from "bat:90% ..." in the situation string
            m = re.search(r'bat:(\d+(?:\.\d+)?)', msg.data)
            if m:
                self.battery_pct = float(m.group(1))
        self.ctx.update_situation(msg.data)

    def _on_camera(self, msg: String):
        with self.lock:
            self.camera_summary = msg.data

    def _on_obstacle(self, msg: String):
        with self.lock:
            self.obstacle_vector = msg.data

    def _on_voice(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"Voice input: '{text}'")

        # If agent is waiting for human answer — this IS the answer
        if self._waiting_for_human:
            self._human_response = text
            self._human_event.set()
            return

        # If agent is waiting for lead response (wingman scenario — not applicable here)
        # Otherwise: new mission goal
        self._assign_goal(text)

    def _on_wingman_message(self, msg: String):
        """Receive messages/queries from Wingman agent."""
        try:
            data = json.loads(msg.data)
            msg_type = data.get('type', 'message')
            content  = data.get('content', msg.data)
        except Exception:
            msg_type = 'message'
            content  = msg.data

        self.get_logger().info(
            f"Wingman [{msg_type}]: {content[:60]}")
        self.ctx.add_inter_agent_message("WINGMAN", content)

        # If wingman asks a question, schedule a response in next loop iteration
        if msg_type == 'query':
            self._wingman_query_pending = True

    def _on_safety_event(self, msg: String):
        """Safety events go into context so agent is aware of them."""
        try:
            data = json.loads(msg.data)
            note = data.get('message', msg.data)
        except Exception:
            note = msg.data
        self.ctx.add_memory_note(f"[SAFETY] {note}")
        self.get_logger().warn(f"Safety event in context: {note}")

    # ── Goal assignment ──────────────────────────────────────────

    def _assign_goal(self, goal: str):
        self.get_logger().info(f"New mission goal: '{goal}'")
        self.ctx.clear_history()
        self.ctx.set_goal(goal)
        self._mission_done  = False
        self._mission_report = ""

        if not self._agent_running:
            self._agent_running = True
            thread = threading.Thread(
                target=self._agent_loop, daemon=True)
            thread.start()
        # If already running: new goal replaces old one, loop continues

    # ── Agent loop ───────────────────────────────────────────────

    def _agent_loop(self):
        """
        The main think-act-observe loop.
        Runs until mission_complete tool is called or rclpy shuts down.
        """
        self.get_logger().info("Agent loop started.")
        self._publish_status("Agent loop active. Processing mission goal.")

        while rclpy.ok() and not self._mission_done:
            # ── Build context prompt ────────────────────────────
            prompt = self.ctx.build_prompt()

            # ── Infer next tool call ────────────────────────────
            tool_name, params = self._infer_tool_call(prompt)

            if tool_name is None:
                self.get_logger().warning(
                    "Could not parse a valid tool call — skipping cycle.")
                time.sleep(self.loop_pause)
                continue

            self.get_logger().info(
                f"Agent → {tool_name}({json.dumps(params)[:80]})")

            # ── Execute tool ────────────────────────────────────
            result = self.tools.execute(tool_name, params)

            self.get_logger().info(f"Result ← {result[:100]}")

            # ── Update context ──────────────────────────────────
            self.ctx.add_tool_result(tool_name, params, result)
            self._publish_status(f"{tool_name}: {result[:80]}")

            # ── Handle wingman query (inject as priority next cycle) ─
            if self._wingman_query_pending:
                self._wingman_query_pending = False
                self.ctx.add_memory_note("[NOTE] Wingman has a pending query. Consider messaging_wingman() with an answer.")

            # ── Brief pause between inference cycles ────────────
            if not self._mission_done:
                time.sleep(self.loop_pause)

        if self._mission_done:
            self.get_logger().info(
                f"Mission complete: {self._mission_report[:100]}")
            self._publish_status(f"MISSION COMPLETE: {self._mission_report}")
            # Announce to GCS
            done_msg = String()
            done_msg.data = f"[MISSION COMPLETE] {self._mission_report}"
            self.pub_clarification.publish(done_msg)

        self._agent_running = False
        self.get_logger().info("Agent loop ended. Awaiting next goal.")

    # ── Tool call inference ──────────────────────────────────────

    def _infer_tool_call(self, prompt: str) -> tuple[str | None, dict]:
        """
        Call the SLM and parse a tool call from the response.
        Retries up to 3 times with error context on parse failure.
        Returns (tool_name, params) or (None, {}) on failure.
        """
        error_ctx = ""
        for attempt in range(3):
            full_prompt = prompt
            if error_ctx:
                full_prompt += (
                    f"\n\n[CORRECTION NEEDED] {error_ctx}"
                    f"\nOutput a valid JSON tool call only. Begin with {{ end with }}")

            raw, latency = self.ollama.infer(full_prompt, self.system_prompt)
            self.get_logger().debug(f"Inference latency: {latency*1000:.0f}ms")

            if raw is None:
                error_ctx = "Inference returned no output."
                continue

            # Try to parse
            try:
                # Handle cases where SLM wraps JSON in markdown
                raw_clean = raw.strip()
                start = raw_clean.find('{')
                end   = raw_clean.rfind('}') + 1
                if start >= 0 and end > start:
                    raw_clean = raw_clean[start:end]

                data = json.loads(raw_clean)
                tool_name = data.get('tool', '')
                params    = data.get('params', {})

                if not isinstance(params, dict):
                    params = {}

                if self.tools.is_valid(tool_name):
                    if attempt > 0:
                        self.get_logger().info(
                            f"Tool parse succeeded on retry {attempt + 1}")
                    return tool_name, params

                valid = list(self.tools.tools.keys())
                error_ctx = (
                    f"Unknown tool '{tool_name}'. "
                    f"You must use one of: {valid}")

            except json.JSONDecodeError as e:
                error_ctx = f"JSON parse error: {str(e)[:80]}. Output only valid JSON."
            except Exception as e:
                error_ctx = f"Parse error: {str(e)[:80]}"

        return None, {}

    # ── Helpers ──────────────────────────────────────────────────

    def _publish_status(self, text: str):
        msg = String()
        msg.data = json.dumps({"lead": text, "wingman": "—"})
        self.pub_mission_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeadAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

## Section 9: Wingman Pilot Agent

The wingman agent mirrors the lead's architecture but uses a smaller context
window (num_ctx=8192), communicates with Lead instead of human, and never
contacts the Ground Commander directly.

### 9.1 Create prompt directory

```bash
# [PC-2] (also sync from PC-1 via rsync)
mkdir -p ~/major_ws/src/major_project/major_project/wingman_pilot/prompts
```

### 9.2 System Prompt (wingman_agent_system.txt)

```bash
cat << 'PROMPT_EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/prompts/wingman_agent_system.txt
You are WINGMAN PILOT — autonomous drone agent controlling Drone-1.
Lead Pilot controls Drone-0 and is your superior.
You NEVER contact the human directly. All human comms go through Lead.

Think-act loop: output EXACTLY ONE tool call per step:
{"tool": "<name>", "params": {"key": value}}

TOOLS:
takeoff(altitude:float)  Ascend to altitude metres
move(direction:str, distance:float)  Fly N/S/E/W/NE/NW/SE/SW
move(direction:str, distance:float, altitude:float)  Fly and change altitude
hover()  Hold position
search(duration_sec:int)  Scan in place 5–60s
land()  Land
rtl()  Return to launch
get_situation()  Full sensor readout
scan_camera()  Camera detections with distances
get_battery()  Own battery percentage
remember(fact:str)  Store a fact
recall(query:str)  Retrieve stored facts
message_lead(message:str)  Tell Lead something (non-blocking)
ask_lead(question:str)  Ask Lead a question — WAITS for answer
notify_lead(message:str)  Status update to Lead (no wait)
wait(seconds:int)  Pause 1–30s
mission_complete(report:str)  Report completion to Lead

WINGMAN RULES:
- When Lead sends you a task: get_situation() first, then execute
- After every move: wait(N) then get_situation() to confirm position
- Found something important: notify_lead() immediately
- Uncertain about an order: ask_lead() before acting
- NEVER call ask_human or notify_human — they do not exist for you
- Battery ≤ 20%: notify_lead("Battery 20%, planning RTL")
- Battery ≤ 15%: call rtl() immediately
- Begin with { end with }

EXAMPLES:
Lead said "cover the east sector" →
  {"tool":"get_situation","params":{}}
  → read state, then:
  {"tool":"move","params":{"direction":"E","distance":40}}
  {"tool":"wait","params":{"seconds":22}}
  {"tool":"search","params":{"duration_sec":20}}
  {"tool":"notify_lead","params":{"message":"East sector covered. Car detected SE 30m."}}
  {"tool":"remember","params":{"fact":"car at SE 30m from home"}}
  {"tool":"rtl","params":{}}
  {"tool":"mission_complete","params":{"report":"East sector covered. Car found SE 30m. RTL initiated."}}
PROMPT_EOF
```

### 9.3 Wingman Agent Node (wingman_agent_node.py)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_agent_node.py
"""
Wingman Agent Node — always-active brain for Drone-1.

Replaces wingman_nlu_node from Parts 4–5.

Key differences from Lead Agent:
  - No ask_human / notify_human tools
  - Uses WingmanToolRegistry (ask_lead, notify_lead, message_lead)
  - Gets mission goals from Lead via /agent/lead_to_wingman
  - Reports completion to Lead, not to human
  - Smaller context (num_ctx=8192 to leave headroom for Lead)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import re
import threading
import time

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from major_project.common.ollama_client import OllamaClient
from major_project.common.tool_registry import WingmanToolRegistry
from major_project.common.context_manager import ContextManager
from major_project.common.agent_memory import AgentMemory


def _load_prompt(filename: str) -> str:
    path = os.path.join(os.path.dirname(__file__), 'prompts', filename)
    with open(path) as f:
        return f.read()


class WingmanAgentNode(Node):

    def __init__(self):
        super().__init__('wingman_agent_node')

        self.declare_parameter('ollama_host', 'localhost')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('model', 'qwen3.5:2b')
        self.declare_parameter('num_ctx', 8192)
        self.declare_parameter('loop_pause_sec', 0.5)

        host    = self.get_parameter('ollama_host').value
        port    = self.get_parameter('ollama_port').value
        model   = self.get_parameter('model').value
        num_ctx = self.get_parameter('num_ctx').value
        self.loop_pause = self.get_parameter('loop_pause_sec').value

        self.ollama = OllamaClient(host=host, port=port,
                                    model=model, num_ctx=num_ctx)
        self.system_prompt = _load_prompt('wingman_agent_system.txt')

        # ── Shared sensor state ─────────────────────────────────
        self.lock = threading.Lock()
        self.own_situation   = ""
        self.camera_summary  = ""
        self.obstacle_vector = ""
        self.battery_pct     = 100.0

        # ── Lead interaction state ──────────────────────────────
        self._waiting_for_lead = False
        self._lead_response: str | None = None
        self._lead_event = threading.Event()

        # ── Agent state ─────────────────────────────────────────
        self._agent_running = False
        self._mission_done  = False
        self._mission_report = ""
        self.ctx = ContextManager()
        self.agent_memory = AgentMemory(db_name="wingman_agent_memory.db")
        self.tools = WingmanToolRegistry(self)

        # ── Publishers ──────────────────────────────────────────
        self.pub_intent  = self.create_publisher(String, '/wingman/approved_intent', 10)
        self.pub_lead_msg = self.create_publisher(String, '/agent/wingman_to_lead', 10)
        self.pub_status  = self.create_publisher(String, '/wingman/status_report_text', 10)

        # ── Subscriptions ────────────────────────────────────────
        self.create_subscription(
            String, '/drone_1/situation', self._on_situation, 10)
        self.create_subscription(
            String, '/camera_1/detections', self._on_camera, 10)
        self.create_subscription(
            String, '/camera_1/obstacle_vector', self._on_obstacle, 10)
        # Orders from Lead agent OR from Lead NLU (backward-compat)
        self.create_subscription(
            String, '/agent/lead_to_wingman', self._on_lead_message, 10)
        self.create_subscription(
            String, '/wingman/order', self._on_legacy_order, 10)
        self.create_subscription(
            String, '/safety/event', self._on_safety_event, 10)

        self.get_logger().info(
            f"Wingman Agent ready (Ollama {host}:{port} model:{model})")

    # ── ROS callbacks ────────────────────────────────────────────

    def _on_situation(self, msg: String):
        with self.lock:
            self.own_situation = msg.data
            m = re.search(r'bat:(\d+(?:\.\d+)?)', msg.data)
            if m:
                self.battery_pct = float(m.group(1))
        self.ctx.update_situation(msg.data)

    def _on_camera(self, msg: String):
        with self.lock:
            self.camera_summary = msg.data

    def _on_obstacle(self, msg: String):
        with self.lock:
            self.obstacle_vector = msg.data

    def _on_lead_message(self, msg: String):
        """Receive message or task from Lead agent."""
        content = msg.data
        self.get_logger().info(f"Lead message: '{content[:80]}'")

        # Check if this is a response to a pending ask_lead
        if self._waiting_for_lead:
            self._lead_response = content
            self._lead_event.set()
            return

        self.ctx.add_inter_agent_message("LEAD", content)
        # Treat any Lead message as potentially a new task/goal
        self._assign_goal(content)

    def _on_legacy_order(self, msg: String):
        """
        Backward compatibility: accept WingmanOrder JSON from lead_nlu_node
        if Parts 3–5 are mixed with Part 6.
        Converts WingmanOrder into a natural language goal for the agent.
        """
        try:
            data = json.loads(msg.data)
            context = data.get('mission_context', '')
            intent  = data.get('intent', {})
            action  = intent.get('action', 'hover')
            goal_text = f"{context}. Action: {action}"
            if intent.get('direction'):
                goal_text += f" {intent['direction']}"
            if intent.get('distance'):
                goal_text += f" {intent['distance']}m"
        except Exception:
            goal_text = msg.data[:120]

        self.get_logger().info(f"Legacy order converted to goal: '{goal_text}'")
        self._assign_goal(goal_text)

    def _on_safety_event(self, msg: String):
        try:
            data = json.loads(msg.data)
            note = data.get('message', msg.data)
        except Exception:
            note = msg.data
        self.ctx.add_memory_note(f"[SAFETY] {note}")

    # ── Goal assignment ──────────────────────────────────────────

    def _assign_goal(self, goal: str):
        self.ctx.clear_history()
        self.ctx.set_goal(goal)
        self._mission_done  = False
        self._mission_report = ""

        if not self._agent_running:
            self._agent_running = True
            threading.Thread(
                target=self._agent_loop, daemon=True).start()

    # ── Agent loop ───────────────────────────────────────────────

    def _agent_loop(self):
        self.get_logger().info("Wingman agent loop started.")

        while rclpy.ok() and not self._mission_done:
            prompt = self.ctx.build_prompt()
            tool_name, params = self._infer_tool_call(prompt)

            if tool_name is None:
                time.sleep(self.loop_pause)
                continue

            self.get_logger().info(
                f"Wingman → {tool_name}({json.dumps(params)[:60]})")
            result = self.tools.execute(tool_name, params)
            self.get_logger().info(f"Result ← {result[:80]}")

            self.ctx.add_tool_result(tool_name, params, result)

            # Publish status to Lead
            status_msg = String()
            status_msg.data = f"{tool_name}: {result[:80]}"
            self.pub_status.publish(status_msg)

            if not self._mission_done:
                time.sleep(self.loop_pause)

        if self._mission_done:
            # Report completion to Lead (not human)
            done_payload = json.dumps({
                "type": "status",
                "content": f"MISSION COMPLETE: {self._mission_report}"
            })
            done_msg = String()
            done_msg.data = done_payload
            self.pub_lead_msg.publish(done_msg)
            self.get_logger().info(
                f"Wingman mission done: {self._mission_report[:80]}")

        self._agent_running = False
        self.get_logger().info("Wingman agent loop ended.")

    # ── Tool call inference ──────────────────────────────────────

    def _infer_tool_call(self, prompt: str) -> tuple[str | None, dict]:
        error_ctx = ""
        for attempt in range(3):
            full_prompt = prompt
            if error_ctx:
                full_prompt += (
                    f"\n\n[CORRECTION NEEDED] {error_ctx}"
                    f"\nOutput a valid JSON tool call only.")

            raw, latency = self.ollama.infer(full_prompt, self.system_prompt)
            if raw is None:
                error_ctx = "No output."
                continue
            try:
                raw_clean = raw.strip()
                start = raw_clean.find('{')
                end   = raw_clean.rfind('}') + 1
                if start >= 0 and end > start:
                    raw_clean = raw_clean[start:end]
                data = json.loads(raw_clean)
                tool_name = data.get('tool', '')
                params    = data.get('params', {})
                if not isinstance(params, dict):
                    params = {}
                if self.tools.is_valid(tool_name):
                    return tool_name, params
                error_ctx = (f"Unknown tool '{tool_name}'. "
                             f"Valid: {list(self.tools.tools.keys())}")
            except json.JSONDecodeError as e:
                error_ctx = f"JSON error: {str(e)[:60]}"
            except Exception as e:
                error_ctx = f"Error: {str(e)[:60]}"
        return None, {}


def main(args=None):
    rclpy.init(args=args)
    node = WingmanAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

---
## Section 10: Configuration, Launch & Deployment

---

### 10.1 setup.py (final)

This is the complete final `setup.py` incorporating all entry points from Parts 3–6. Part 6 adds `lead_agent` and `wingman_agent`; Parts 3–5 supply all preceding entries.

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'major_project'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Devrajsinh Gohil',
    maintainer_email='202511004@dau.ac.in',
    description='Rank-based multi-SLM drone pilot system — full agentic loop version',
    license='MIT',
    entry_points={
        'console_scripts': [
            # GCS nodes
            'stt_node = major_project.gcs.stt_node:main',
            'clarification_speaker = major_project.gcs.clarification_speaker_node:main',
            'mission_monitor = major_project.gcs.mission_monitor_node:main',
            'emergency_stop = major_project.gcs.emergency_stop_node:main',
            'camera_detection = major_project.gcs.camera_detection_node:main',
            # Lead pilot nodes
            'lead_sensor_aggregator = major_project.lead_pilot.lead_sensor_aggregator_node:main',
            'lead_px4_commander = major_project.lead_pilot.lead_px4_commander_node:main',
            'lead_intent_bridge = major_project.lead_pilot.lead_intent_bridge_node:main',
            'safety_monitor = major_project.lead_pilot.safety_monitor_node:main',
            # Wingman pilot nodes
            'wingman_sensor_aggregator = major_project.wingman_pilot.wingman_sensor_aggregator_node:main',
            'wingman_px4_commander = major_project.wingman_pilot.wingman_px4_commander_node:main',
            # Part 6: agent loop nodes (replace NLU + mission_executor + mission_memory)
            'lead_agent = major_project.lead_pilot.lead_agent_node:main',
            'wingman_agent = major_project.wingman_pilot.wingman_agent_node:main',
        ],
    },
)
```

> **Note on removed entries:** `lead_nlu`, `wingman_nlu`, `mission_executor`, and `mission_memory` are intentionally absent. Part 6's agent loop nodes (`lead_agent`, `wingman_agent`) fully replace them. The old nodes remain on disk for reference but are not registered as executable entry points.

---

### 10.2 Lead Pilot Configuration (lead_config.yaml)

Complete final `lead_config.yaml` combining the initial Part 3 entries, the Part 5 additions (safety monitor, camera detection), and the Part 6 agent node parameters.

```yaml
# ── Sensor layer ──────────────────────────────────────────────────────────────
lead_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0

camera_detection_node:
  ros__parameters:
    camera_index: 0
    model_path: "yolov8n.pt"
    confidence_threshold: 0.4
    publish_rate_hz: 2.0
    obstacle_labels: ["person", "car", "truck", "bicycle", "bird"]

# ── Safety layer ──────────────────────────────────────────────────────────────
safety_monitor_node:
  ros__parameters:
    battery_warn_pct: 20.0   # warn at 20% — published to /safety/event
    battery_rtl_pct: 15.0    # force RTL at 15% — bypasses SLM entirely

# ── Intelligence layer (Part 6 — agent loop) ──────────────────────────────────
lead_agent_node:
  ros__parameters:
    ollama_host: "localhost"
    ollama_port: 11434
    model: "qwen3.5:2b"
    num_ctx: 8192
    loop_pause_sec: 0.5    # pause between inference cycles

# ── Execution layer ────────────────────────────────────────────────────────────
lead_px4_commander_node:
  ros__parameters:
    drone_namespace: ""    # Drone-0 uses default /fmu/ namespace
```

---

### 10.3 Wingman Pilot Configuration (wingman_config.yaml)

Complete final `wingman_config.yaml` combining Part 4 sensor aggregator and commander entries with the Part 6 wingman agent parameters.

```yaml
# ── Sensor layer ──────────────────────────────────────────────────────────────
wingman_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0

# ── Intelligence layer (Part 6 — wingman agent loop) ──────────────────────────
wingman_agent_node:
  ros__parameters:
    ollama_host: "localhost"   # PC-2's own Ollama instance
    ollama_port: 11434
    model: "qwen3.5:2b"
    num_ctx: 8192              # smaller than Lead to leave headroom on PC-2
    loop_pause_sec: 0.5

# ── Execution layer ────────────────────────────────────────────────────────────
wingman_px4_commander_node:
  ros__parameters:
    drone_namespace: "px4_1"
```

---

### 10.4 Lead Pilot Launch File (lead_pilot.launch.py)

Complete final launch file for PC-1. Starts all ten nodes in the Part 6 architecture. `lead_nlu`, `mission_executor`, and `mission_memory` nodes from Parts 3–5 are **not** included — they are replaced by `lead_agent`.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('major_project'),
        'config', 'lead_config.yaml')

    return LaunchDescription([
        # ── Sensor layer ─────────────────────────────────────────────────────
        Node(package='major_project',
             executable='lead_sensor_aggregator',
             name='lead_sensor_aggregator_node',
             output='screen',
             parameters=[cfg]),

        Node(package='major_project',
             executable='camera_detection',
             name='camera_detection_node',
             output='screen',
             parameters=[cfg]),

        # ── Safety layer (hard rules, no SLM) ────────────────────────────────
        Node(package='major_project',
             executable='safety_monitor',
             name='safety_monitor_node',
             output='screen',
             parameters=[cfg]),

        # ── Intelligence layer — Part 6 agent loop ────────────────────────────
        Node(package='major_project',
             executable='lead_agent',
             name='lead_agent_node',
             output='screen',
             parameters=[cfg]),

        # ── Execution layer ───────────────────────────────────────────────────
        Node(package='major_project',
             executable='lead_px4_commander',
             name='lead_px4_commander_node',
             output='screen',
             parameters=[cfg]),

        Node(package='major_project',
             executable='lead_intent_bridge',
             name='lead_intent_bridge_node',
             output='screen'),

        # ── GCS layer ─────────────────────────────────────────────────────────
        Node(package='major_project',
             executable='stt_node',
             name='stt_node',
             output='screen'),

        Node(package='major_project',
             executable='clarification_speaker',
             name='clarification_speaker_node',
             output='screen'),

        Node(package='major_project',
             executable='mission_monitor',
             name='mission_monitor_node',
             output='screen'),

        Node(package='major_project',
             executable='emergency_stop',
             name='emergency_stop_node',
             output='screen'),
    ])
```

---

### 10.5 Wingman Pilot Launch File (wingman_pilot.launch.py)

Complete final launch file for PC-2. Starts three nodes. `wingman_nlu` from Parts 4–5 is **not** included — it is replaced by `wingman_agent`.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('major_project'),
        'config', 'wingman_config.yaml')

    return LaunchDescription([
        # ── Sensor layer ─────────────────────────────────────────────────────
        Node(package='major_project',
             executable='wingman_sensor_aggregator',
             name='wingman_sensor_aggregator_node',
             output='screen',
             parameters=[cfg]),

        # ── Intelligence layer — Part 6 wingman agent ─────────────────────────
        Node(package='major_project',
             executable='wingman_agent',
             name='wingman_agent_node',
             output='screen',
             parameters=[cfg]),

        # ── Execution layer ───────────────────────────────────────────────────
        Node(package='major_project',
             executable='wingman_px4_commander',
             name='wingman_px4_commander_node',
             output='screen',
             parameters=[cfg]),
    ])
```

---

### 10.6 Build & Verify

Run all build steps on PC-1 first, then sync and rebuild on PC-2.

**PC-1 — build:**

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash
```

**Verify new entry points are registered:**

```bash
ros2 pkg executables major_project | grep agent
# Expected output:
#   lead_agent
#   wingman_agent
```

**Python smoke test — import all new Part 6 modules and validate core logic:**

```bash
python3 - << 'EOF'
import sys, os
sys.path.insert(0, os.path.expanduser('~/major_ws/src/major_project'))

from major_project.common.tool_registry import LeadToolRegistry, WingmanToolRegistry
from major_project.common.context_manager import ContextManager
from major_project.common.agent_memory import AgentMemory

# Test context manager
ctx = ContextManager()
ctx.set_goal("find a football on the north field")
ctx.update_situation("bat:90% alt:0m mode:MANUAL gps:OK")
ctx.add_tool_result("get_situation", {}, "bat:90% alt:0m mode:MANUAL")
ctx.add_tool_result("takeoff", {"altitude": 10}, "Takeoff initiated. Ascending to 10m.")
ctx.add_tool_result("wait", {"seconds": 15}, "Waited 15s.")
prompt = ctx.build_prompt()
assert "[MISSION GOAL]" in prompt
assert "find a football" in prompt
assert "RECENT ACTIONS" in prompt
print("ContextManager OK")

# Test memory
mem = AgentMemory(db_name="test_agent_memory.db")
mem.clear()
mem.remember("football spotted at N50m E0m")
mem.remember("car parked at E30m")
results = mem.recall("football")
assert len(results) == 1
assert "football" in results[0]
print("AgentMemory OK")

# Test compression
for i in range(12):
    ctx.add_tool_result(f"tool_{i}", {"x": i}, f"result_{i}")
assert len(ctx.history) <= 8, f"History too long: {len(ctx.history)}"
assert "Earlier" in ctx.memory_block, "Compression not working"
print("Context compression OK")

print("\nAll Part 6 module tests passed!")
EOF
```

**PC-1 — sync to PC-2:**

```bash
rsync -av --progress \
  ~/major_ws/src/major_project/ \
  dev@<PC2_IP>:~/major_ws/src/major_project/
```

**PC-2 — build:**

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash
```

---

### 10.7 Deployment Sequence

Start processes in this exact order. Each step gates the next — do not proceed if a step fails.

**PC-1 Terminal 1 — PX4 SITL Drone-0:**

```bash
cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_iris
```

**PC-1 Terminal 2 — PX4 SITL Drone-1:**

```bash
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=iris \
  ./build/px4_sitl_default/bin/px4 -i 1 -d ./build/px4_sitl_default/etc
```

**PC-1 Terminal 3 — DDS Bridge Agent:**

```bash
MicroXRCEAgent udp4 -p 8888
```

**PC-1 Terminal 4 — Full lead stack with agent loop:**

```bash
source ~/.bashrc
ros2 launch major_project lead_pilot.launch.py
```

Wait until all ten nodes report ready in the terminal output before starting PC-2.

**PC-2 Terminal 1 — Wingman agent stack:**

```bash
source ~/.bashrc
ros2 launch major_project wingman_pilot.launch.py
```

**PC-1 Terminal 5 — Monitor agent reasoning in real time (optional but recommended):**

```bash
ros2 topic echo /mission_status
```

**PC-1 Terminal 6 — Monitor GCS speaker output:**

```bash
ros2 topic echo /clarification_request
```

**PC-1 Terminal 7 — Monitor lead-to-wingman messages:**

```bash
ros2 topic echo /agent/lead_to_wingman
```

**PC-1 Terminal 8 — Monitor wingman-to-lead messages:**

```bash
ros2 topic echo /agent/wingman_to_lead
```

---

### 10.8 End-to-End Tests

#### Test 1 — Simple autonomous mission

Inject a mission goal:

```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String \
  '{data: "survey the north sector, both drones, come back when done"}'
```

**Expected agent reasoning trace (from `/mission_status` topic):**

```
get_situation: bat:92% alt:0m mode:MANUAL gps:OK camera:clear
takeoff: Takeoff initiated. Ascending to 10m. Allow ~25s.
wait: Waited 12s.
get_situation: bat:90% alt:9.8m mode:OFFBOARD camera:clear
message_wingman: Sent to Wingman: 'Mission: survey north sector. You take NE sector, I take N.'
move: Moving north 50m. ETA ~28s. Call wait(28)...
wait: Waited 28s.
get_situation: bat:87% alt:10.1m pos:(49.8,0.2) camera:clear
search: Search complete (20s). Area clear — no detections.
notify_human: GCS notified: 'North sector surveyed. Area clear. Returning home.'
rtl: RTL initiated. Allow ~40s.
wait: Waited 40s.
mission_complete: MISSION COMPLETE: Surveyed N sector 50m. No obstacles. Wingman covered NE. Both RTL.
```

---

#### Test 2 — Human permission escalation

```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String \
  '{data: "investigate the east perimeter and check if there are any people"}'
```

Then simulate a person detection mid-flight:

```bash
ros2 topic pub --rate 2 /camera_0/obstacle_vector std_msgs/msg/String \
  '{data: "person:ahead:very_close"}'
```

**Expected — agent calls ask_human:**

```
search: Search complete (15s). Detected: person(91%) ahead close [person:ahead:very_close]
ask_human: [blocking] "Person detected very close ahead at E sector boundary. Should I hover and observe, avoid and continue, or return home?"
```

GCS speaker says the question aloud. You respond:

```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String \
  '{data: "hover and report exact position then come back"}'
```

**Agent continues:**

```
hover: Hovering at current position.
get_situation: bat:74% alt:10m pos:(0,45) camera:person(91%) ahead close
remember: Remembered: 'person detected at east perimeter pos(0,45) at 10m alt'
notify_human: GCS notified: 'Person confirmed at E perimeter ~45m east. Hovering to observe.'
wait: Waited 10s.
rtl: RTL initiated.
mission_complete: Person found at E perimeter (45m E). Position logged in memory. RTL complete.
```

---

#### Test 3 — Agent-to-agent collaboration

```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String \
  '{data: "split up: you take north sector, tell wingman to take south sector, meet back in 60 seconds"}'
```

**Expected Lead agent:**

```
get_situation: ...
message_wingman: 'Your mission: survey south sector 40m. RTL after 60s. I am taking north.'
move: Moving N 40m.
...
```

**Expected Wingman agent (received Lead message):**

```
get_situation: ...
move: Moving S 40m.
search: Search complete (15s). ...
notify_lead: 'South sector clear. RTL initiated.'
rtl: RTL initiated.
mission_complete: South sector surveyed. Clear. RTL.
```

---

#### Test 4 — Safety monitor during active mission

Let the agent run a long mission, then check that battery RTL fires independently of the SLM:

```bash
# Simulate critical battery on Drone-1
ros2 param set /safety_monitor_node battery_rtl_pct 95.0
# Safety monitor will immediately force RTL on Drone-1 + publish safety event
ros2 topic echo /safety/event
# Agent sees the safety event in its next context update
```

**Expected:** `/safety/event` publishes a `battery_rtl` event with `severity: critical`. The lead agent node picks it up on the next context cycle via its `_on_safety_event` callback and injects `[SAFETY] ...` into the context window. The safety RTL fires regardless of what the SLM is currently doing — the two paths are independent.

---

#### Test 5 — Context compression under sustained operation

Run a long multi-step mission and verify the agent's context window never overflows:

```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String \
  '{data: "do a full perimeter sweep, 100m radius, all four cardinal directions, come back"}'
```

**What to watch:** After 8 tool calls have accumulated in the agent's history, the `ContextManager` automatically compresses the oldest 4 entries into a one-line `Earlier: ...` summary and moves it to `[MEMORY]`. The `[RECENT ACTIONS]` section in the prompt never exceeds 8 entries. Monitor with:

```bash
ros2 topic echo /mission_status
# Each line shows one tool call result — watch the sequence grow then compress
```

**Expected:** The agent completes all four legs of the sweep without running out of context or losing track of the goal. The final `mission_complete` report correctly summarises all sectors covered.

---

## Architecture Summary

```
VOICE/TEXT GOAL
      │
      ▼
┌─────────────────────────────────────────┐
│           LEAD AGENT NODE               │
│  ┌──────────────────────────────────┐   │
│  │  Context Manager (bounded 2048)  │   │  ← ContextManager
│  │  [GOAL][SITUATION][MEMORY]       │   │
│  │  [WINGMAN MSGS][RECENT ACTIONS]  │   │
│  └──────────────────┬───────────────┘   │
│                     │ build_prompt()    │
│              OllamaClient              │  ← qwen3.5:2b
│                     │ tool call JSON   │
│          LeadToolRegistry.execute()    │  ← ToolRegistry
│          ┌──────────┼──────────────┐  │
│       flight     memory         comms │
│       tools      tools          tools │
│      takeoff    remember     ask_human│
│      move       recall    notify_human│
│      search              msg_wingman  │
│      rtl ...                          │
│                     │                 │
│        ctx.add_tool_result()          │  ← context grows, compresses
│                     │                 │
│              AgentMemory.db           │  ← SQLite persist
└─────────────────────────────────────┬─┘
                                       │
          /lead/approved_intent        │  /agent/lead_to_wingman
                │                     │          │
                ▼                     │          ▼
    LeadPX4CommanderNode     ┌─────────────────────────┐
    (unchanged from Part 3)  │    WINGMAN AGENT NODE    │
                             │  (same loop, smaller     │
    SafetyMonitorNode        │   tool set, ask_lead     │
    (unchanged from Part 5)  │   instead of ask_human)  │
                             └─────────────────────────┘
```

**The agents are now:**
- Always thinking (loop runs until `mission_complete` is called)
- Tool-driven (each decision is one discrete tool call)
- Context-aware (bounded window with compression)
- Memory-persistent (SQLite survives restarts)
- Internally communicating (Lead ↔ Wingman via /agent/* topics)
- Human-escalating (ask_human blocks the loop until answered)
- Safety-independent (safety_monitor fires RTL without consulting SLM)
