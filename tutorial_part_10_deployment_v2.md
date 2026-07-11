# Part 10 — Configuration, Launch Files & Deployment (V2 - LangGraph)

> **Series position:** This is the final part of the drone swarm tutorial, bringing together all the V2 LangGraph components into a single deployable system.

---

## 10.1 `lead_config.yaml` — Complete Final Config

**File:** `config/lead_config.yaml`

This single YAML file supplies `ros__parameters` to every node launched on **PC-1**.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/config/lead_config.yaml
# ── Sensor layer ──────────────────────────────────────────────────────────────
lead_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0
    min_separation_m: 5.0          # proximity warning threshold

camera_detection_node:
  ros__parameters:
    image_topic: "/camera/image_raw"
    use_usb_camera: false
    camera_index: 0
    model_path: "yolov8n.pt"
    confidence_threshold: 0.4
    publish_rate_hz: 2.0
    obstacle_labels: ["person", "car", "truck", "bicycle", "bird"]

# ── Safety layer ──────────────────────────────────────────────────────────────
safety_monitor_node:
  ros__parameters:
    battery_warn_pct: 20.0
    battery_rtl_pct: 15.0
    min_separation_m: 5.0
    proximity_warn_interval_sec: 5.0

# ── Intelligence layer (V2 LangGraph) ─────────────────────────────────────────
lead_agent_node:
  ros__parameters:
    ollama_host: "localhost"
    ollama_port: 11434
    model: "qwen3.5:2b"
    num_ctx: 8192

# ── Execution layer ───────────────────────────────────────────────────────────
lead_px4_commander_node:
  ros__parameters:
    drone_namespace: ""     # Drone-0 uses default /fmu/ namespace

lead_intent_bridge_node:
  ros__parameters:
    chain_step_delay_sec: 6.0
EOF
```

---

## 10.2 `wingman_config.yaml` — Complete Final Config

**File:** `config/wingman_config.yaml`

This file is deployed and loaded on **PC-2** only. 

```bash
cat << 'EOF' > ~/major_ws/src/major_project/config/wingman_config.yaml
# ── Sensor layer ──────────────────────────────────────────────────────────────
wingman_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0

wingman_camera_detection_node:
  ros__parameters:
    image_topic: "/px4_1/camera/image_raw"
    use_usb_camera: false
    camera_index: 1
    model_path: "yolov8n.pt"
    confidence_threshold: 0.4
    publish_rate_hz: 2.0
    obstacle_labels: ["person", "car", "truck", "bicycle", "bird"]

# ── Intelligence layer (V2 LangGraph) ─────────────────────────────────────────
wingman_agent_node:
  ros__parameters:
    ollama_host: "localhost"
    ollama_port: 11434
    model: "qwen3.5:2b"
    num_ctx: 8192

# ── Execution layer ───────────────────────────────────────────────────────────
wingman_px4_commander_node:
  ros__parameters:
    drone_namespace: "px4_1"
EOF
```

---

## 10.3 Final Build Steps (PC-1)

With the V2 LangGraph upgrade, we have new Python package requirements.

```bash
# Install LangGraph dependencies
pip3 install langgraph langchain-core langchain-ollama pydantic

cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash
```

Verify the entry points exist:
```bash
ros2 pkg executables major_project
```
You should see `major_project lead_agent` and `major_project wingman_agent` among the output.

---

## 10.4 Deployment Sequence (Strict Order)

Use `tmux` with 5 panes on PC-1 + 1 terminal on PC-2.

### STEP 1 — PC-1: Launch Drone-0 SITL
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4010 PX4_GZ_MODEL=x500_mono_cam \
PX4_GZ_WORLD=baylands \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_UXRCE_DDS_KEY=1 \
./build/px4_sitl_default/bin/px4 -i 0 -d
```
**✅ Wait for:** `[commander] Ready for takeoff!`

### STEP 2 — PC-1: Launch Drone-1 SITL
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4010 PX4_GZ_MODEL=x500_mono_cam \
PX4_GZ_WORLD=baylands \
PX4_GZ_MODEL_POSE="5,0,0,0,0,0" PX4_UXRCE_DDS_KEY=2 \
./build/px4_sitl_default/bin/px4 -i 1 -d
```
**✅ Wait for:** `[commander] Ready for takeoff!`

### STEP 3 — PC-1: Start DDS Agent
```bash
source ~/.bashrc
MicroXRCEAgent udp4 -p 8888
```
**✅ Wait for:** Two `Session established with client key` logs.

### STEP 4 — PC-1: Launch Lead Stack
```bash
source ~/major_ws/install/setup.bash
ros2 launch major_project lead_pilot.launch.py
```
**✅ Wait for:** Lead Agent to self-start its LangGraph into the `STANDBY` state.

### STEP 5 — PC-2: Launch Wingman Stack
First, ensure PC-2 has the V2 dependencies:
```bash
pip3 install langgraph langchain-core langchain-ollama pydantic
```
Then launch:
```bash
source ~/major_ws/install/setup.bash
ros2 launch major_project wingman_pilot.launch.py
```

### STEP 6 — PC-1: Give Voice Command
```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String "data: 'take off and fly north 50 metres'"
```
Watch the Lead Agent's LangGraph `Planner` node generate the checklist and pass it to the `Executor` node!
