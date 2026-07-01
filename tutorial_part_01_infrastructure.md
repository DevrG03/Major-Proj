# Tutorial Part 01 — Infrastructure Setup

> **Project:** Autonomous Drone Swarm with LLM Agents  
> **Stack:** Ubuntu 26.04 LTS · ROS2 Lyrical · Gazebo Jetty (gz-sim 10.x) · PX4 v1.15.0 · Python 3.12+

---

## Overview

This tutorial walks you through building the complete infrastructure for a two-drone autonomous swarm system. By the end you will have:

- ROS2 Lyrical and Gazebo Jetty installed and verified
- PX4 SITL compiled with the GCC 15 compatibility workaround
- `px4_msgs` built against ROS2 Lyrical
- MicroXRCE-DDS Agent running to bridge PX4 ↔ ROS2
- Ollama serving `qwen2.5-coder:3b` locally
- Both drone SITL instances launching with correct namespaces
- A latency benchmark confirming the LLM is fast enough for real-time use

> [!IMPORTANT]
> All commands below assume you are running **Ubuntu 26.04 LTS** on the GCS / simulation PC (PC-1). Run every block in a fresh terminal unless instructed otherwise.

---

## Section 1.1 — System Requirements and Overview

### Hardware Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 8-core x86_64 | 12-core+ (AMD Ryzen 7 / Intel i7) |
| RAM | 16 GB | 32 GB |
| GPU | Any (CPU-only Ollama works) | NVIDIA RTX 3060+ for faster inference |
| Storage | 50 GB free | 100 GB SSD |
| Network | Gigabit LAN / WiFi 5 | WiFi 6 (for PC-1 ↔ PC-2 agent split) |

### Software Stack

```
Ubuntu 26.04 LTS (Noble+1)
├── ROS2 Lyrical (ROS 2 release for Ubuntu 26.04)
├── Gazebo Jetty (gz-sim 10.x)
│   └── ros-gz bridge
├── PX4 Autopilot v1.15.0 (SITL)
│   └── MicroXRCE-DDS Agent (DDS ↔ uORB bridge)
├── px4_msgs (ROS2 message definitions for PX4)
├── Ollama (local LLM server)
│   └── qwen2.5-coder:3b model
└── Python 3.12+ (system default on Ubuntu 26.04)
```

> [!NOTE]
> **GCC 15 Compatibility Warning:** Ubuntu 26.04 ships GCC 15 as default. PX4 v1.15.0 does **not** compile cleanly with GCC 15 due to stricter `int`/`bool` conversion rules. The workaround (Section 1.4) installs and pins GCC 12 for the PX4 build only — your system default remains GCC 15.

---

## Section 1.2 — Install ROS2 Lyrical

### 1.2.1 Set locale

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2.2 Add ROS2 apt repository

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.2.3 Install ROS2 Lyrical desktop

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-lyrical-desktop
```

### 1.2.4 Install development tools and colcon

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  ros-lyrical-rmw-cyclonedds-cpp \
  ros-lyrical-ros-gz \
  build-essential \
  git \
  wget \
  curl

sudo rosdep init || true
rosdep update
```

### 1.2.5 Source ROS2 in every terminal

```bash
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### Verification

```bash
ros2 --version
# Expected output: ros2 cli version: <version> (lyrical)

printenv RMW_IMPLEMENTATION
# Expected: rmw_cyclonedds_cpp
```

---

## Section 1.3 — Install Gazebo Jetty and ROS2-Gazebo Bridge

### 1.3.1 Add Gazebo apt repository

```bash
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
```

### 1.3.2 Install Gazebo Jetty

```bash
sudo apt install -y gz-jetty
```

### 1.3.3 Install ROS2-Gazebo bridge

```bash
sudo apt install -y ros-lyrical-ros-gz
```

### Verification

```bash
gz sim --version
# Expected: Gazebo Sim, version 10.x.x

ros2 pkg list | grep ros_gz
# Expected: ros_gz_bridge, ros_gz_sim, etc.
```

---

## Section 1.4 — Build PX4 v1.15.0 with GCC 15 Workaround (SITL)

> [!WARNING]
> **GCC 15 Workaround Required.** You MUST set `CC=gcc-12` and `CXX=g++-12` before building PX4 or the build will fail with errors like `error: conversion from 'int' to 'bool'`. Do not skip this step.

### 1.4.1 Install GCC 12

```bash
sudo apt install -y gcc-12 g++-12
gcc-12 --version
# Expected: gcc-12 (Ubuntu ...) 12.x.x
```

### 1.4.2 Install PX4 build dependencies

```bash
sudo apt install -y \
  astyle \
  cmake \
  cppcheck \
  doxygen \
  file \
  g++ \
  gcc \
  gdb \
  git \
  lcov \
  libacl1-dev \
  libssl-dev \
  libxml2-utils \
  libxml2-dev \
  make \
  ninja-build \
  python3-dev \
  python3-pip \
  python3-setuptools \
  python3-wheel \
  rsync \
  shellcheck \
  unzip \
  zip

pip3 install --user \
  kconfiglib \
  jinja2 \
  jsonschema \
  packaging \
  toml \
  pyros-genmsg \
  setuptools
```

### 1.4.3 Clone PX4 v1.15.0

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.15.0 --depth 1
cd PX4-Autopilot
git submodule update --init --recursive
```

> [!NOTE]
> The submodule update can take 5-15 minutes depending on your connection. This downloads Gazebo models, SITL wrappers, and uORB definitions.

### 1.4.4 Run PX4 Ubuntu setup script

```bash
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx
```

### 1.4.5 Build PX4 SITL with GCC 12

```bash
cd ~/PX4-Autopilot

# Pin to GCC 12 for this build only
export CC=gcc-12
export CXX=g++-12

make px4_sitl_default
```

> [!NOTE]
> Build takes 10-20 minutes on first run. Subsequent builds are incremental and much faster.

### Verification

```bash
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
# Expected: file exists

~/PX4-Autopilot/build/px4_sitl_default/bin/px4 --version
# Expected: px4 v1.15.0
```

---

## Section 1.5 — Build and Install px4_msgs for ROS2 Lyrical

`px4_msgs` provides all the ROS2 message types (`.msg` files) that match PX4's uORB topics. You must build the version that matches your PX4 version.

### 1.5.1 Create the px4_msgs workspace

```bash
mkdir -p ~/px4_msgs_ws/src
cd ~/px4_msgs_ws/src

git clone https://github.com/PX4/px4_msgs.git --branch release/1.15 --depth 1
```

### 1.5.2 Build px4_msgs

```bash
cd ~/px4_msgs_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 1.5.3 Source px4_msgs in every terminal

```bash
echo "source ~/px4_msgs_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verification

```bash
source ~/px4_msgs_ws/install/setup.bash
ros2 interface list | grep px4_msgs | head -10
# Expected: px4_msgs/msg/VehicleCommand, px4_msgs/msg/VehicleLocalPosition, etc.
```

---

## Section 1.6 — Install MicroXRCE-DDS Agent

The MicroXRCE-DDS Agent bridges PX4's internal uORB messaging to the DDS/ROS2 network. It must be running whenever you want PX4 SITL topics to be visible in ROS2.

### 1.6.1 Clone and build the agent

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git --depth 1
cd Micro-XRCE-DDS-Agent

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

### 1.6.2 Verify the agent binary

```bash
MicroXRCEAgent --version
# Expected: eProsima Micro XRCE-DDS Agent v2.x.x
```

### 1.6.3 Create a convenience alias

```bash
echo "alias dds_agent='MicroXRCEAgent udp4 -p 8888'" >> ~/.bashrc
source ~/.bashrc
```

### Verification

```bash
# Start agent briefly (will show "waiting for connections")
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 2
echo "Agent running: PID $AGENT_PID"
kill $AGENT_PID
```

---

## Section 1.7 — Install Ollama and Pull qwen2.5-coder:3b

Ollama serves local LLM models via a simple HTTP API. The `qwen2.5-coder:3b` model is small enough (~2 GB) to run on CPU while being capable enough for structured JSON intent generation.

### 1.7.1 Install Ollama

```bash
curl -fsSL https://ollama.com/install.sh | sh
```

### 1.7.2 Start Ollama service

```bash
ollama serve &
sleep 3
echo "Ollama server started"
```

> [!NOTE]
> On a systemd system you can also run: `sudo systemctl enable --now ollama`

### 1.7.3 Pull the model

```bash
ollama pull qwen2.5-coder:3b
```

> [!NOTE]
> Download is approximately 2 GB. This may take several minutes on a slow connection.

### 1.7.4 Add Ollama to systemd (recommended for persistence)

```bash
sudo tee /etc/systemd/system/ollama.service > /dev/null << 'SYSTEMD_EOF'
[Unit]
Description=Ollama LLM Server
After=network-online.target

[Service]
ExecStart=/usr/local/bin/ollama serve
Restart=always
RestartSec=3
Environment=OLLAMA_HOST=0.0.0.0:11434

[Install]
WantedBy=multi-user.target
SYSTEMD_EOF

sudo systemctl daemon-reload
sudo systemctl enable --now ollama
```

### Verification

```bash
# Check Ollama API is responding
curl -s http://localhost:11434/api/tags | python3 -m json.tool | grep "qwen2.5-coder"
# Expected: "name": "qwen2.5-coder:3b"

# Quick inference test
curl -s http://localhost:11434/api/generate \
  -d '{"model":"qwen2.5-coder:3b","prompt":"Say: READY","stream":false}' \
  | python3 -c "import sys,json; print(json.load(sys.stdin)['response'])"
# Expected: READY (or similar short response)
```

---

## Section 1.8 — Verify Single-Drone SITL

This section launches one PX4 SITL instance with Gazebo Jetty and confirms ROS2 can see its topics.

### Terminal A — Start MicroXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

Leave this terminal open. You should see it waiting for connections.

### Terminal B — Launch Drone-0 SITL

```bash
cd ~/PX4-Autopilot
export CC=gcc-12 CXX=g++-12

PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500_mono_cam \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_UXRCE_DDS_KEY=1 \
./build/px4_sitl_default/bin/px4 -i 0 -d
```

Wait for Gazebo to open and for the PX4 console to show readiness. You will see DDS agent log lines indicating connection from PX4.

### Terminal C — Verify ROS2 Topics

```bash
source /opt/ros/lyrical/setup.bash
source ~/px4_msgs_ws/install/setup.bash

# List all topics — should include /fmu/out/... topics
ros2 topic list | grep fmu | head -20
```

Expected output includes:
```
/fmu/in/vehicle_command
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status
/fmu/out/sensor_combined
```

### Verification

```bash
# Check specific telemetry topic is publishing
ros2 topic hz /fmu/out/vehicle_local_position --window 5
# Expected: average rate: ~50.000 Hz

# Check message type
ros2 topic info /fmu/out/vehicle_local_position
# Expected: Type: px4_msgs/msg/VehicleLocalPosition
```

Kill all terminals (Ctrl+C) before proceeding.

---

## Section 1.9 — Verify Two-Drone SITL

> [!IMPORTANT]
> Both drones share the **same DDS agent** on port 8888. The `PX4_UXRCE_DDS_KEY` differentiates them. Drone-0 publishes on `/fmu/...` and Drone-1 publishes on `/px4_1/fmu/...`.

### Terminal A — DDS Agent (leave running throughout)

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal B — Drone-0 (Lead) — starts Gazebo + spawns camera drone

```bash
cd ~/PX4-Autopilot
CC=gcc-12 CXX=g++-12 make px4_sitl gz_x500_mono_cam
```

> [!NOTE]
> `make px4_sitl gz_x500_mono_cam` does an **incremental build** (fast, ~5s if already built) then starts Gazebo Jetty with PX4's own tuned world (`default.sdf` — correct 1ms physics step, proper sensor plugin order). This avoids the accelerometer timeout and motor aliasing errors caused by the raw binary approach.
>
> Wait for `INFO [commander] Ready for takeoff!` before starting Drone-1.

### Terminal C — Drone-1 (Wingman) — joins existing Gazebo, wait 10s after Drone-0

```bash
cd ~/PX4-Autopilot

PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500_mono_cam \
PX4_GZ_MODEL_POSE="5,0,0,0,0,0" \
PX4_GZ_STANDALONE=1 \
PX4_UXRCE_DDS_KEY=2 \
CC=gcc-12 CXX=g++-12 \
./build/px4_sitl_default/bin/px4 -i 1 -d
```

> [!IMPORTANT]
> `PX4_GZ_STANDALONE=1` tells PX4 to **connect to the already-running Gazebo** from Terminal B. Without it, PX4 will try to start a second Gazebo instance and fail.

Drone-1 is spawned 5 metres along the X-axis from Drone-0.

### Terminal D — Verify Both Namespaces

```bash
source /opt/ros/lyrical/setup.bash
source ~/px4_msgs_ws/install/setup.bash

echo "=== Drone-0 (Lead) topics ==="
ros2 topic list | grep "^/fmu" | head -10

echo ""
echo "=== Drone-1 (Wingman) topics ==="
ros2 topic list | grep "^/px4_1/fmu" | head -10
```

Expected output:
```
=== Drone-0 (Lead) topics ===
/fmu/in/vehicle_command
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status

=== Drone-1 (Wingman) topics ===
/px4_1/fmu/in/vehicle_command
/px4_1/fmu/out/vehicle_local_position
/px4_1/fmu/out/vehicle_status
```

### Full Two-Drone Verification

```bash
# Both drones publishing position at ~50 Hz
ros2 topic hz /fmu/out/vehicle_local_position --window 5 &
ros2 topic hz /px4_1/fmu/out/vehicle_local_position --window 5 &
wait
# Both should show: average rate: ~50.000 Hz
```

---

## Section 1.10 — Ollama Latency Benchmark

This benchmark measures end-to-end LLM response time to confirm the model is fast enough for real-time drone command generation (target: < 2 seconds for a structured JSON response).

### Create the benchmark script

```bash
cat << 'EOF' > ~/ollama_benchmark.py
#!/usr/bin/env python3
"""
Ollama qwen2.5-coder:3b latency benchmark for drone swarm project.
Tests structured JSON intent generation latency.
Target: < 2000 ms per response for real-time use.
"""

import json
import time
import statistics
import urllib.request
import urllib.error

OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL = "qwen2.5-coder:3b"
NUM_RUNS = 5

SYSTEM_PROMPT = """You are a drone swarm tactical coordinator.
Given a voice command, output ONLY valid JSON with this exact schema:
{
  "intent": "MOVE|SEARCH|RETURN|HOLD|EMERGENCY",
  "target": {"x": float, "y": float, "z": float},
  "speed": float,
  "formation": "line|wedge|stack|dispersed"
}"""

TEST_PROMPTS = [
    "Move to grid alpha at 20 metres altitude",
    "Search sector bravo in wedge formation",
    "Return to base immediately",
    "Hold current position",
    "Emergency stop all drones",
]


def query_ollama(prompt: str) -> tuple[str, float]:
    """Query Ollama and return (response_text, elapsed_ms)."""
    payload = json.dumps({
        "model": MODEL,
        "system": SYSTEM_PROMPT,
        "prompt": prompt,
        "stream": False,
        "options": {
            "temperature": 0.1,
            "num_predict": 128,
        }
    }).encode("utf-8")

    req = urllib.request.Request(
        OLLAMA_URL,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    t0 = time.perf_counter()
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            data = json.loads(resp.read())
        elapsed_ms = (time.perf_counter() - t0) * 1000
        return data.get("response", ""), elapsed_ms
    except urllib.error.URLError as e:
        raise RuntimeError(f"Ollama request failed: {e}") from e


def main():
    print(f"{'='*60}")
    print(f"Ollama Latency Benchmark -- {MODEL}")
    print(f"Target: < 2000 ms per response")
    print(f"Runs per prompt: {NUM_RUNS}")
    print(f"{'='*60}\n")

    all_latencies = []

    for i, prompt in enumerate(TEST_PROMPTS, 1):
        print(f"[{i}/{len(TEST_PROMPTS)}] Prompt: \"{prompt}\"")
        run_latencies = []

        for run in range(NUM_RUNS):
            try:
                response, ms = query_ollama(prompt)
                run_latencies.append(ms)
                status = "PASS" if ms < 2000 else "FAIL"
                print(f"  Run {run+1}: {ms:7.1f} ms  [{status}]  ->  {response[:60].strip()}...")
            except RuntimeError as e:
                print(f"  Run {run+1}: ERROR -- {e}")

        if run_latencies:
            avg = statistics.mean(run_latencies)
            p95 = sorted(run_latencies)[int(len(run_latencies) * 0.95)]
            print(f"  -> avg: {avg:.1f} ms  |  p95: {p95:.1f} ms\n")
            all_latencies.extend(run_latencies)

    print(f"{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    if all_latencies:
        overall_avg = statistics.mean(all_latencies)
        overall_p95 = sorted(all_latencies)[int(len(all_latencies) * 0.95)]
        overall_min = min(all_latencies)
        overall_max = max(all_latencies)
        print(f"  Total samples : {len(all_latencies)}")
        print(f"  Min latency   : {overall_min:.1f} ms")
        print(f"  Avg latency   : {overall_avg:.1f} ms")
        print(f"  P95 latency   : {overall_p95:.1f} ms")
        print(f"  Max latency   : {overall_max:.1f} ms")
        verdict = "PASS" if overall_avg < 2000 else "FAIL -- consider GPU acceleration or smaller model"
        print(f"\n  Verdict: {verdict}")
    else:
        print("  No successful samples collected. Is Ollama running?")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
EOF

chmod +x ~/ollama_benchmark.py
```

### Run the benchmark

```bash
# Ensure Ollama is running
ollama serve &>/dev/null &
sleep 2

python3 ~/ollama_benchmark.py
```

### Expected output (CPU-only, typical laptop)

```
============================================================
Ollama Latency Benchmark -- qwen2.5-coder:3b
Target: < 2000 ms per response
Runs per prompt: 5
============================================================

[1/5] Prompt: "Move to grid alpha at 20 metres altitude"
  Run 1:   843.2 ms  [PASS]  ->  {"intent": "MOVE", "target": {"x": 0, "y": 0, "z": 20}...
  ...
  -> avg: 910.4 ms  |  p95: 1050.2 ms

SUMMARY
============================================================
  Total samples : 25
  Min latency   : 720.1 ms
  Avg latency   : 950.3 ms
  P95 latency   : 1280.4 ms
  Max latency   : 1502.0 ms

  Verdict: PASS
============================================================
```

> [!TIP]
> If average latency exceeds 2000 ms on CPU, either:
> 1. Install CUDA and run `ollama serve` with GPU (`nvidia-smi` to check), or
> 2. Switch to `qwen2.5-coder:1.5b` (smaller, faster, slightly less accurate)

---

## Infrastructure Verification Checklist

Run this complete checklist before proceeding to Part 02:

```bash
echo "=== Infrastructure Verification ==="

echo -n "[1] ROS2 Lyrical:         "
ros2 --version 2>/dev/null | grep -q lyrical && echo "OK" || echo "FAIL"

echo -n "[2] Gazebo Jetty:         "
gz sim --version 2>/dev/null | grep -q "10\." && echo "OK" || echo "FAIL"

echo -n "[3] PX4 binary:           "
test -f ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo "OK" || echo "FAIL"

echo -n "[4] px4_msgs installed:   "
source ~/px4_msgs_ws/install/setup.bash 2>/dev/null
ros2 interface list 2>/dev/null | grep -q px4_msgs && echo "OK" || echo "FAIL"

echo -n "[5] MicroXRCE-DDS Agent:  "
which MicroXRCEAgent &>/dev/null && echo "OK" || echo "FAIL"

echo -n "[6] Ollama running:       "
curl -s http://localhost:11434/api/tags &>/dev/null && echo "OK" || echo "FAIL"

echo -n "[7] qwen2.5-coder:3b:    "
curl -s http://localhost:11434/api/tags | grep -q "qwen2.5-coder" && echo "OK" || echo "FAIL"

echo -n "[8] GCC 12 available:     "
gcc-12 --version &>/dev/null && echo "OK" || echo "FAIL"

echo -n "[9] Python 3.12+:         "
python3 --version 2>/dev/null | grep -qE "3\.(12|13|14)" && echo "OK" || echo "FAIL"

echo ""
echo "=== Done. All items should show OK before proceeding. ==="
```

---

## Next Steps

Proceed to **Tutorial Part 02 — ROS2 Package Scaffold** to create the `major_project` ROS2 package with all nodes, entry points, and the PC-2 agent setup.
