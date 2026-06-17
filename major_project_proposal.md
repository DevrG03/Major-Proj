# Project Proposal & 6-Week Development Plan

## Rank-Based Multi-SLM Drone Pilot System with Confidence-Gated Hierarchical Command Propagation

**Student:** Devrajsinh Gohil (202511004@dau.ac.in)
**Instructor:** Prof. Dr. Tapas Kumar Maiti
**Institution:** Dhirubhai Ambani University
**Program:** M.Tech Major Project
**Duration:** 6 Weeks
**Extends:** Minor Project — Confidence-Gated Intent Parsing for Voice-Controlled UAVs

---

## Table of Contents

1. [Abstract](#abstract)
2. [Problem Statement](#problem-statement)
3. [Research Gap](#research-gap)
4. [Objectives](#objectives)
5. [Novel Contributions](#novel-contributions)
6. [System Architecture Overview](#system-architecture-overview)
7. [Technology Stack](#technology-stack)
8. [Evaluation Metrics](#evaluation-metrics)
9. [Expected Outcomes](#expected-outcomes)
10. [6-Week Development Plan](#6-week-development-plan)
11. [Risk Register](#risk-register)
12. [References](#references)

---

## Abstract

This project extends a single-drone confidence-gated SLM voice control system (minor project) to a two-drone formation where each drone is piloted by a Small Language Model (SLM) operating under a rank-based command hierarchy. The Lead Pilot SLM receives natural language mission orders from a human Ground Commander, decomposes them into structured flight primitives, controls its own drone (Drone-0), and issues schema-validated orders to the Wingman SLM. The Wingman SLM pilots Drone-1, executes orders, and reports situational awareness back to the Lead. A two-level confidence-gating cascade ensures that ambiguous commands trigger clarification at the appropriate hierarchy level — the Wingman asks the Lead, and the Lead asks the human — without any command ever reaching the flight stack in an unscreened state. All language intelligence runs offline via Ollama on CPU-only hardware. The system is validated in PX4 SITL + Gazebo Garden distributed across two Ubuntu PCs over WiFi, establishing empirical benchmarks for latency, confidence calibration, and false execution rate in a multi-agent SLM drone context.

---

## Problem Statement

Voice-controlled UAV systems using Small Language Models (SLMs) have demonstrated promise for single-drone control, but extending them to multi-drone formations introduces three unsolved problems:

**Problem 1 — Coordination without semantic safety:** In a multi-drone system, structurally valid commands issued to individual drones may be collectively unsafe or contradictory (e.g., two drones ordered to the same waypoint). No existing edge-deployable SLM system applies confidence-gated semantic screening to inter-drone coordination commands.

**Problem 2 — Inter-agent communication protocol:** Natural language is the intuitive choice for SLM-to-SLM communication (mirroring how real pilots use radio), but empirical benchmarks show that free-form NL between LLM agents produces unreliable coordination even for frontier models. A minimum viable structured protocol for sub-1B SLM inter-agent communication has not been defined.

**Problem 3 — Latency viability:** The end-to-end latency of an SLM-based NLU pipeline (STT → Ollama inference → Pydantic validation → ROS2 publish) in a distributed WiFi environment has not been benchmarked against PX4 offboard mode constraints (>2 Hz setpoint update requirement). This is the fundamental feasibility question for any real-time SLM drone control system.

---

## Research Gap

A systematic review of literature (deep research, 2026-06-03, 110 agents, 27 sources) confirms:

| Gap | Evidence |
|---|---|
| No paper defines a minimum inter-agent message schema for sub-1B SLM drone coordination | arXiv:2602.21670 uses GPT-4o; no 3B-scale equivalent |
| No empirical latency benchmark for Ollama SLM in ROS2 WiFi loop vs PX4 offboard constraints | Explicitly noted as a gap in arXiv:2506.07509 |
| No confidence-gated hierarchical propagation (gate at every level of the chain) | Existing confidence gates are single-level (human→drone) |
| No Air Force-style rank hierarchy with SLM agents (Lead + Wingman with clarification cascade) | Closest work uses GPT-4o with PDDL planners, not edge SLMs |

---

## Objectives

**Primary Objectives:**
1. Design and implement a two-drone ROS2 system where each drone is controlled by an independent Qwen2.5-Coder:3b SLM pilot (Lead and Wingman) operating offline via Ollama
2. Implement a rank-based hierarchical command architecture: Human Commander → Lead SLM → Wingman SLM → PX4 flight stack
3. Extend the minor project's single-level confidence gate to a two-level cascade with clarification propagation through the hierarchy
4. Validate the system in three SITL scenarios covering clear, ambiguous, and emergency command classes

**Secondary Objectives:**
5. Empirically benchmark end-to-end latency of the SLM pipeline in a distributed WiFi ROS2 environment
6. Define and evaluate a minimum Pydantic schema for inter-SLM drone coordination (WingmanOrder + StatusReport)
7. Characterise confidence calibration accuracy for both Lead and Wingman SLMs independently

---

## Novel Contributions

### Contribution 1: Two-Level Confidence-Gated Hierarchical Command Propagation
The minor project introduced confidence gating at the human→drone interface. This project introduces the same gate at the lead→wingman interface, creating a cascade:
- Wingman's LOW confidence → clarification request to Lead (not to human)
- Lead's LOW confidence on wingman order → Lead requests clarification from human before forwarding
- This is the first system to propagate confidence-gated clarification through a multi-SLM command hierarchy

### Contribution 2: Minimum Inter-SLM Message Schema for Drone Coordination
We define, implement, and evaluate `WingmanOrder` and `StatusReport` Pydantic schemas — the minimum structured message specification required for reliable Lead-to-Wingman coordination at the 3B parameter scale. This fills an explicit gap in literature.

### Contribution 3: Empirical Latency Characterisation in Distributed SLM Drone Control
We benchmark the per-stage latency of: STT → Ollama inference → Pydantic validation → ROS2 publish → PX4 setpoint in a 2-PC WiFi distributed setup, providing the first data on whether 3B-scale edge SLMs are viable for real-time multi-drone coordination under PX4 offboard timing constraints.

### Contribution 4: Sensor-Grounded SLM Situational Awareness Without Vision Hardware
We implement and evaluate a sensor aggregation pipeline (Camera→YOLO text summary + GPS + IMU/EKF2 + Barometer) that grounds both SLMs in real-time environmental state without requiring LiDAR or a vision-language model — enabling deployment on CPU-only edge hardware.

---

## System Architecture Overview

```
┌────────────────────────────────────────────────────────┐
│              HUMAN GROUND COMMANDER                    │
│         Voice input + GCS terminal display             │
└──────────────────────┬─────────────────────────────────┘
                       │ /voice_commands
                       ▼
╔══════════════════════════════════════╗
║           PC-1 — LEAD PILOT         ║
║                                     ║
║  STT Node (Faster-Whisper)          ║
║  Lead NLU Node (Qwen2.5-Coder:3b)  ║
║  Lead Sensor Aggregator             ║
║  Lead PX4 Commander                 ║
║  PX4 SITL Drone-0 + Gazebo Server  ║
║  XRCE-DDS Agent (serves both)       ║
║  GCS Monitor Node                   ║
╚══════════╤═══════════════════════════╝
           │ /wingman/order (WingmanOrder)     WiFi
           │◄─────────────────────────────────────►
           │ /wingman/status_report (StatusReport)
           ▼
╔══════════════════════════════════════╗
║          PC-2 — WINGMAN             ║
║                                     ║
║  Wingman NLU Node (Qwen2.5-Coder:3b)║
║  Wingman Sensor Aggregator          ║
║  Wingman PX4 Commander              ║
║  PX4 SITL Drone-1                   ║
╚══════════════════════════════════════╝
```

**Command flow:** Voice → STT → Lead NLU → [Drone-0 intent + Wingman order] → Wingman NLU → Drone-1 intent → PX4

**Confidence cascade:**
- Lead LOW → human clarification request (speaker node)
- Wingman LOW → lead clarification request (Lead NLU re-routes to human if needed)

---

## Technology Stack

| Component | Technology | Version |
|---|---|---|
| OS | Ubuntu | 22.04 LTS |
| Middleware | ROS2 | Humble Hawksbill |
| DDS | CycloneDDS | latest |
| Flight stack | PX4 Autopilot | v1.17.0-alpha1 |
| Simulator | Gazebo | Garden |
| SLM inference | Ollama | 0.21.2+ |
| SLM model | Qwen2.5-Coder | 3b (int8 quantised) |
| STT | Faster-Whisper | 1.2.1 |
| Schema validation | Pydantic | v2 |
| Object detection | Ultralytics YOLO | v8 (nano, CPU) |
| Language | Python | 3.10 |
| DDS-PX4 bridge | MicroXRCEAgent | latest |

---

## Evaluation Metrics

### Primary Metrics (replicate minor project methodology)

| Metric | Definition | Target |
|---|---|---|
| False Execution Rate (FER) | % of ambiguous/OOS commands executed without clarification | < 25% (minor: 19%) |
| Confidence Accuracy | % of commands where SLM confidence matches expected | > 70% |
| HIGH Confidence Accuracy | % of clear commands rated HIGH | > 90% |
| LOW Confidence Accuracy | % of OOS commands rated LOW | > 60% |
| Schema Compliance Rate | % of SLM outputs that pass Pydantic validation | > 95% |

### New Metrics (multi-agent specific)

| Metric | Definition | Target |
|---|---|---|
| Order Transmission Latency | Time from Lead issuing WingmanOrder to Wingman receiving it | < 100ms |
| Wingman Confidence Accuracy | Same as above but for Wingman NLU node | > 65% |
| Clarification Cascade Rate | % of LOW confidence events that trigger correct clarification routing | > 80% |
| End-to-End Pipeline Latency | STT → Ollama → Pydantic → ROS2 publish | Measure + report |
| Offboard Feasibility Score | Whether pipeline latency is compatible with >2 Hz PX4 offboard | Pass/Fail |
| Mission Success Rate | 3 SITL scenarios completed as expected | 3/3 |

### Benchmark Dataset
200 annotated commands (reuse minor project dataset structure):
- 70 Clear → expected confidence: HIGH
- 90 Ambiguous → expected confidence: MEDIUM/LOW
- 40 Out-of-scope → expected confidence: LOW

Additional 60 multi-agent specific commands:
- 20 Formation commands (clear): "search in parallel formation"
- 20 Coordination ambiguous: "spread out and cover more area"
- 20 Conflict/emergency: "both land immediately", "wingman abort"

---

## Expected Outcomes

### Deliverables
1. Full ROS2 package: `major_project` (Python, open-source)
   - `lead_pilot/` — Lead NLU, sensor aggregator, PX4 commander
   - `wingman_pilot/` — Wingman NLU, sensor aggregator, PX4 commander
   - `common/` — Pydantic schemas, normaliser, confidence gate
   - `gcs/` — STT node, clarification speaker, mission monitor
   - Launch files for both PCs
2. Benchmark dataset: 260 annotated UAV commands (200 reused + 60 new)
3. Evaluation results: FER, confidence accuracy, latency benchmarks
4. Academic paper draft (conference-ready)
5. SITL demo video: 3 scenarios

### Academic Contribution
A conference paper (target: IEEE ICRA 2027 or similar) reporting:
- First empirical evaluation of confidence-gated hierarchical SLM command propagation in multi-drone systems
- Minimum inter-SLM message schema definition and evaluation
- Latency characterisation of distributed edge-SLM drone control over WiFi

---

## 6-Week Development Plan

> **Reading this plan:** Each day has atomic tasks. A task marked `[CRITICAL PATH]` blocks subsequent work. Tasks marked `[RISK]` have mitigation steps in the Risk Register. Check off tasks as you complete them. Each week ends with a go/no-go checkpoint.

---

### WEEK 1: Infrastructure, Networking & Latency Benchmark
**Goal:** Both PCs running ROS2 + PX4 SITL + Gazebo, cross-PC topic verified, Ollama latency measured.
**Critical path item:** Ollama WiFi latency benchmark — if this fails, the entire architecture must be reconsidered.

---

#### Day 1 (Mon) — PC-1 Base Setup

**Morning (3h): Ubuntu + ROS2 on PC-1**
- [ ] Verify Ubuntu 22.04 LTS is installed on PC-1 (`lsb_release -a`)
- [ ] Update system: `sudo apt update && sudo apt upgrade -y`
- [ ] Install ROS2 Humble:
  ```bash
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
  ```
- [ ] Source ROS2 and add to `.bashrc`:
  ```bash
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```
- [ ] Verify: `ros2 topic list` returns without error

**Afternoon (3h): CycloneDDS + environment variables**
- [ ] Install CycloneDDS:
  ```bash
  sudo apt install ros-humble-rmw-cyclonedds-cpp -y
  ```
- [ ] Set DDS and domain in `.bashrc`:
  ```bash
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
  source ~/.bashrc
  ```
- [ ] Create workspace:
  ```bash
  mkdir -p ~/major_ws/src
  cd ~/major_ws
  colcon build
  source install/setup.bash
  echo "source ~/major_ws/install/setup.bash" >> ~/.bashrc
  ```
- [ ] Install Python dependencies:
  ```bash
  pip install pydantic==2.* faster-whisper ultralytics pyserial
  ```
- [ ] Verify Pydantic v2: `python3 -c "import pydantic; print(pydantic.__version__)"`

**Evening checkpoint:**
- [ ] `ros2 topic list` works
- [ ] `python3 -c "import pydantic, faster_whisper, ultralytics"` — no errors

---

#### Day 2 (Tue) — PX4 + Gazebo Garden on PC-1

**Morning (3h): PX4 build**
- [ ] Install PX4 dependencies:
  ```bash
  sudo apt install git cmake ninja-build python3-pip -y
  pip3 install kconfiglib jinja2 jsonschema pyros-genmsg packaging toml
  ```
- [ ] Clone PX4:
  ```bash
  cd ~
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  cd PX4-Autopilot
  git checkout v1.14.0  # use stable tag
  bash Tools/setup/ubuntu.sh
  ```
- [ ] Build PX4 SITL:
  ```bash
  cd ~/PX4-Autopilot
  make px4_sitl gz_x500
  ```
  *(First build takes 10-20 min)*
- [ ] Verify single-drone SITL launches: `make px4_sitl gz_x500` — Gazebo opens with one drone

**Afternoon (2h): MicroXRCEAgent**
- [ ] Build MicroXRCE-DDS agent:
  ```bash
  cd ~
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent
  mkdir build && cd build
  cmake .. && make -j$(nproc)
  sudo make install
  ```
- [ ] Verify: `MicroXRCEAgent --version`

**Afternoon (1h): px4_msgs ROS2 package**
- [ ] Clone and build px4_msgs:
  ```bash
  cd ~/major_ws/src
  git clone https://github.com/PX4/px4_msgs.git
  cd ~/major_ws
  colcon build --packages-select px4_msgs
  source install/setup.bash
  ```

**Evening checkpoint:**
- [ ] Single drone SITL launches in Gazebo
- [ ] MicroXRCEAgent binary exists
- [ ] `ros2 interface list | grep px4_msgs` returns messages

---

#### Day 3 (Wed) — Single Drone ROS2 Bridge Verification

**Morning (3h): Single drone + DDS bridge**
- [ ] Terminal 1 — Launch single SITL:
  ```bash
  cd ~/PX4-Autopilot
  make px4_sitl gz_x500
  ```
- [ ] Terminal 2 — Launch XRCE-DDS agent:
  ```bash
  MicroXRCEAgent udp4 -p 8888
  ```
- [ ] Terminal 3 — Verify ROS2 topics appear:
  ```bash
  ros2 topic list | grep fmu
  # Should show /fmu/out/vehicle_local_position, /fmu/out/vehicle_status, etc.
  ```
- [ ] Echo a topic to confirm data flowing:
  ```bash
  ros2 topic echo /fmu/out/vehicle_status
  ```
- [ ] Check publish frequency:
  ```bash
  ros2 topic hz /fmu/out/vehicle_local_position
  # Should be ~10-50 Hz
  ```

**Afternoon (3h): px4_ros_com and offboard control test**
- [ ] Clone px4_ros_com (Python offboard example):
  ```bash
  cd ~/major_ws/src
  git clone https://github.com/PX4/px4_ros_com.git
  cd ~/major_ws
  colcon build --packages-select px4_ros_com
  ```
- [ ] Run the offboard control example:
  ```bash
  ros2 run px4_ros_com offboard_control
  ```
- [ ] Verify drone arms and takes off in Gazebo
- [ ] Understand the setpoint message structure: `ros2 interface show px4_msgs/msg/TrajectorySetpoint`

**Evening checkpoint:**
- [ ] `/fmu/out/vehicle_status` publishes at >10 Hz
- [ ] Offboard example arms and lifts drone in Gazebo

---

#### Day 4 (Thu) — 2-Drone SITL on Single PC

**Morning (3h): Multi-vehicle SITL**
- [ ] Study PX4 multi-vehicle docs: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation
- [ ] Launch Drone-0 (Terminal 1):
  ```bash
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
    ./build/px4_sitl_default/bin/px4 -i 0 -d
  ```
- [ ] Launch Drone-1 (Terminal 2, after Drone-0 is up):
  ```bash
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="2,0,0,0,0,0" \
    ./build/px4_sitl_default/bin/px4 -i 1 -d
  ```
- [ ] Terminal 3 — XRCE-DDS agent:
  ```bash
  MicroXRCEAgent udp4 -p 8888
  ```
- [ ] Verify both namespaced topics appear:
  ```bash
  ros2 topic list | grep fmu
  # Expect: /fmu/out/... (drone 0) AND /px4_1/fmu/out/... (drone 1)
  ```

**Afternoon (2h): Verify independent control**
- [ ] Write a quick Python test that takes off Drone-0 only:
  ```python
  # test_drone0_takeoff.py
  import rclpy
  from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
  # publish to /fmu/in/... (drone 0 namespace)
  ```
- [ ] Confirm Drone-1 remains on ground while Drone-0 takes off
- [ ] Land Drone-0, take off Drone-1 (using /px4_1/fmu/in/...)
- [ ] Confirm namespacing cleanly isolates both drones

**Evening checkpoint:**
- [ ] Two drones visible in Gazebo, independently controllable
- [ ] `/fmu/out/vehicle_local_position` and `/px4_1/fmu/out/vehicle_local_position` both publishing

---

#### Day 5 (Fri) — PC-2 Setup + Cross-PC ROS2

**Morning (3h): PC-2 mirrors PC-1 setup**
- [ ] Repeat Day 1 steps on PC-2 (Ubuntu 22.04, ROS2 Humble, CycloneDDS)
- [ ] Use SAME `ROS_DOMAIN_ID=42` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- [ ] Install PX4 + MicroXRCEAgent on PC-2 (same steps as Day 2)
- [ ] Do NOT start Gazebo on PC-2 yet

**Afternoon (2h): Cross-PC topic test**
- [ ] PC-1 Terminal: publish a test topic:
  ```bash
  ros2 topic pub /cross_pc_test std_msgs/msg/String "data: 'hello from PC1'"
  ```
- [ ] PC-2 Terminal: verify receipt:
  ```bash
  ros2 topic echo /cross_pc_test
  ```
- [ ] If multicast fails, configure unicast peers:
  ```bash
  # Create /etc/cyclonedds/cyclonedds.xml on both PCs:
  # <CycloneDDS><Domain><General>
  #   <Interfaces><NetworkInterface name="wlan0"/></Interfaces>
  # </General></Domain></CycloneDDS>
  export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
  ```
- [ ] Achieve bidirectional topic exchange between PCs

**Afternoon (1h): Latency pre-test (basic)**
- [ ] Measure basic ROS2 cross-PC latency:
  ```bash
  # PC-1: ros2 run demo_nodes_cpp talker
  # PC-2: ros2 run demo_nodes_cpp listener
  # Use ros2 topic delay /chatter to measure
  ```

**Evening checkpoint:**
- [ ] PC-2 can echo topics published from PC-1
- [ ] PC-1 can echo topics published from PC-2
- [ ] Round-trip latency measured (expect <10ms on same WiFi)

---

#### Day 6 (Sat) — [CRITICAL PATH] Ollama Latency Benchmark

**This is the most important day of Week 1. If Ollama cannot respond fast enough over WiFi in a ROS2 loop, the architecture must change (e.g., pre-computed action lookup, or async non-blocking inference).**

**Morning (3h): Ollama setup on both PCs**
- [ ] Install Ollama on PC-1:
  ```bash
  curl -fsSL https://ollama.ai/install.sh | sh
  ollama pull qwen2.5-coder:3b
  ```
- [ ] Install Ollama on PC-2 (same steps)
- [ ] Verify both can run model locally:
  ```bash
  ollama run qwen2.5-coder:3b "say hello"
  ```

**Afternoon (4h): Latency benchmark script**
- [ ] Create `benchmark/latency_test.py`:
  ```python
  """
  Measures end-to-end latency of the SLM pipeline stage by stage.
  Stages: [1] Ollama inference, [2] Pydantic validation, [3] ROS2 publish
  Runs N=50 trials, records mean, median, p95, p99.
  """
  import time, statistics, requests, json, rclpy
  from pydantic import BaseModel
  from typing import Literal, Optional

  SYSTEM_PROMPT = """Output JSON with action, confidence, clarification_question."""
  TEST_COMMANDS = [
      "take off to 5 meters",           # clear
      "go that way",                    # ambiguous
      "what is the weather today",      # out of scope
  ]
  N_TRIALS = 50

  def measure_ollama_latency(prompt, host="localhost"):
      url = f"http://{host}:11434/api/generate"
      payload = {"model": "qwen2.5-coder:3b", "prompt": prompt,
                 "system": SYSTEM_PROMPT, "stream": False, "format": "json"}
      t0 = time.perf_counter()
      r = requests.post(url, json=payload)
      t1 = time.perf_counter()
      return t1 - t0, r.json().get("response", "")

  # Run: local inference, remote inference (over WiFi to other PC)
  # For each: record latency per trial
  # Print: mean, median, p95, p99, min, max
  # Check: is mean < 500ms? (needed for ~2 Hz)
  ```
- [ ] Run benchmark: local inference on PC-1 (50 trials per command)
- [ ] Run benchmark: PC-2 calling PC-1's Ollama over WiFi (50 trials per command)
- [ ] Record all numbers in `benchmark/latency_results.csv`

**Checkpoint: Latency Decision Gate**
- [ ] If mean inference latency < 500ms → proceed with current architecture
- [ ] If mean latency 500ms–2000ms → implement async non-blocking NLU (SLM runs in background, ROS2 publishes last-known setpoint at >2Hz independently)
- [ ] If mean latency > 2000ms → escalate to Prof., consider using 1.5b model or cached response lookup

**Evening checkpoint:**
- [ ] Latency numbers recorded for all 3 command types, local and remote
- [ ] Architecture decision made based on results
- [ ] `benchmark/latency_results.csv` saved

---

#### Day 7 (Sun) — Buffer + Week 1 Review

- [ ] Fix any environment issues from Days 1–6
- [ ] Write `docs/week1_report.md`: infrastructure status, latency numbers, DDS config, go/no-go decision
- [ ] Plan Week 2 task order based on latency findings
- [ ] If latency requires async design, sketch the async NLU node architecture today

**Week 1 Go/No-Go Checkpoint:**
- [ ] Both PCs run ROS2 Humble with CycloneDDS
- [ ] 2-drone SITL runs on PC-1 with independent namespace topics
- [ ] Cross-PC ROS2 topics are bidirectional
- [ ] Ollama latency benchmarked and architecture decision made
- [ ] No blockers unresolved

---

### WEEK 2: Port Minor Project + Lead Sensor Stack
**Goal:** Lead Pilot controls Drone-0 in simulation with full sensor context, end-to-end: voice → intent → flight action.

---

#### Day 8 (Mon) — ROS2 Package Skeleton + Schema Definitions

**Morning (3h): Package structure**
- [ ] Create ROS2 package:
  ```bash
  cd ~/major_ws/src
  ros2 pkg create major_project --build-type ament_python \
    --dependencies rclpy px4_msgs std_msgs sensor_msgs
  ```
- [ ] Create directory structure:
  ```
  major_project/
  ├── major_project/
  │   ├── common/
  │   │   ├── __init__.py
  │   │   ├── schemas.py          # All Pydantic schemas
  │   │   ├── normaliser.py       # alias normalisation (from minor project)
  │   │   └── confidence_gate.py  # Gate logic
  │   ├── lead_pilot/
  │   │   ├── lead_nlu_node.py
  │   │   ├── lead_px4_commander_node.py
  │   │   ├── lead_sensor_aggregator_node.py
  │   │   └── lead_intent_bridge_node.py
  │   ├── wingman_pilot/
  │   │   ├── wingman_nlu_node.py
  │   │   ├── wingman_px4_commander_node.py
  │   │   └── wingman_sensor_aggregator_node.py
  │   └── gcs/
  │       ├── stt_node.py
  │       ├── clarification_speaker_node.py
  │       └── mission_monitor_node.py
  ├── launch/
  │   ├── lead_pilot.launch.py
  │   └── wingman_pilot.launch.py
  └── config/
      ├── lead_config.yaml
      └── wingman_config.yaml
  ```

**Afternoon (3h): Pydantic schemas**
- [ ] Write `common/schemas.py` with all schemas:
  - `FlightIntent` (ported from minor project, unchanged)
  - `WingmanOrder` (new)
  - `StatusReport` (new)
  - `SituationalAwareness` (new)
  - `LeadOutput` (what Lead SLM outputs per cycle)
  - `WingmanOutput` (what Wingman SLM outputs per cycle)
- [ ] Write unit tests for each schema in `tests/test_schemas.py`
- [ ] Run tests: `python3 -m pytest tests/test_schemas.py -v`

**Evening checkpoint:**
- [ ] Package structure created and builds: `colcon build --packages-select major_project`
- [ ] All schemas defined and unit tests pass

---

#### Day 9 (Tue) — Port Minor Project Core

**Morning (3h): Port normaliser + confidence gate**
- [ ] Copy and adapt `normaliser.py` from minor project:
  - Maps 50+ alias variants to canonical action names
  - Add new aliases: "form up", "follow lead", "hold position", "wingman abort"
- [ ] Port `confidence_gate.py`:
  - `gate_lead(confidence)` → EXECUTE / EXECUTE+WARN / WITHHOLD+CLARIFY
  - `gate_wingman(confidence)` → EXECUTE / EXECUTE+WARN / REQUEST_CLARIFICATION_FROM_LEAD
- [ ] Write unit tests for both

**Afternoon (3h): Port Lead NLU node**
- [ ] Create `lead_pilot/lead_nlu_node.py` starting from minor project's `nlu_node.py`
- [ ] Key changes from minor project:
  - System prompt extended with Lead Pilot role and WingmanOrder output field
  - Subscribes to `/wingman/status_report` in addition to `/voice_commands`
  - Builds situational awareness context block from `/drone_0/situation` and `/drone_1/situation`
  - Publishes to `/lead/approved_intent`, `/wingman/order`, `/clarification_request`
- [ ] Wire Ollama call (same pattern as minor project: `http://localhost:11434/api/generate`)
- [ ] Wire Pydantic validation using `LeadOutput` schema

**Evening checkpoint:**
- [ ] `lead_nlu_node.py` starts without errors
- [ ] Manual test: publish a string to `/voice_commands`, verify node calls Ollama and publishes to correct topics

---

#### Day 10 (Wed) — Lead PX4 Commander Node

**Morning (3h): Lead PX4 commander**
- [ ] Create `lead_pilot/lead_px4_commander_node.py`:
  - Subscribes to `/lead/approved_intent` (FlightIntent)
  - Translates to `TrajectorySetpoint` messages on `/fmu/in/trajectory_setpoint`
  - Publishes `OffboardControlMode` at 10 Hz (keepalive)
  - Sends `VehicleCommand` for arm/disarm, takeoff, land, RTL
  - Handles action types: `takeoff`, `move`, `hover`, `land`, `rtl`, `search`
- [ ] Port the action translation logic from minor project's `px4_commander_node.py`

**Afternoon (3h): Intent bridge + integration test**
- [ ] Create `lead_pilot/lead_intent_bridge_node.py`:
  - Subscribes to `/lead/approved_intent`
  - Handles `land` and `rtl` as direct MAVLink vehicle commands (IDs 21, 20)
- [ ] Write launch file `launch/lead_pilot.launch.py` that starts all lead nodes
- [ ] Integration test:
  1. Start 2-drone SITL + XRCE-DDS agent
  2. Launch lead_pilot.launch.py
  3. Manually publish to `/voice_commands`: `"take off to 5 meters"`
  4. Verify Drone-0 takes off to 5m in Gazebo

**Evening checkpoint:**
- [ ] Drone-0 responds to voice commands published to `/voice_commands`
- [ ] Land command works
- [ ] RTL command works

---

#### Day 11 (Thu) — STT Node + Clarification Speaker

**Morning (2h): STT node**
- [ ] Create `gcs/stt_node.py` (port from minor project):
  - Uses `faster_whisper` with `tiny.en` model, int8 quantisation
  - Captures mic audio at 16kHz mono
  - Publishes transcribed text to `/voice_commands`
- [ ] Test: speak "take off to 5 meters" → verify it publishes correctly

**Morning (1h): Clarification speaker**
- [ ] Create `gcs/clarification_speaker_node.py`:
  - Subscribes to `/clarification_request`
  - Prints to terminal with formatting
  - Optional: TTS via `espeak-ng`

**Afternoon (3h): GCS Mission Monitor**
- [ ] Create `gcs/mission_monitor_node.py`:
  - Subscribes to `/drone_0/situation`, `/drone_1/situation`, `/mission_status`
  - Displays a live terminal dashboard (using `rich` library):
    ```
    ┌─────────────────────────────────────────────────┐
    │ MISSION CONTROL STATION          [TIME: 00:04:32] │
    ├────────────────┬────────────────────────────────┤
    │ DRONE-0 (LEAD) │ DRONE-1 (WINGMAN)              │
    │ pos: (0,0,50)  │ pos: (5,0,50)                  │
    │ bat: 87%       │ bat: 84%                       │
    │ mode: OFFBOARD │ mode: HOLD                     │
    ├────────────────┴────────────────────────────────┤
    │ LAST COMMAND: "search eastern sector"           │
    │ LEAD STATUS: Moving to grid E-4                 │
    │ WINGMAN STATUS: Executing search pattern        │
    └─────────────────────────────────────────────────┘
    ```

**Evening checkpoint:**
- [ ] Full voice-to-flight pipeline works: speak → STT → Lead NLU → Drone-0 moves
- [ ] Clarification speaker outputs when Lead confidence is LOW
- [ ] Mission monitor displays live data

---

#### Day 12 (Fri) — Lead Sensor Aggregator

**Morning (3h): Sensor aggregator**
- [ ] Create `lead_pilot/lead_sensor_aggregator_node.py`:
  - Subscribes to:
    - `/fmu/out/vehicle_local_position` → x, y, z, vx, vy, vz
    - `/fmu/out/vehicle_status` → arming state, flight mode
    - `/fmu/out/battery_status` → battery percentage
    - `/camera_0/detections` → YOLO detection results
  - Formats `SituationalAwareness` text block every 1 second:
    ```
    [DRONE-0|LEAD] pos:(0.0,0.0,50.0) hdg:090 spd:2.1m/s bat:87% mode:OFFBOARD
    [CAMERA-0] Forward clear. No obstacles detected.
    [GPS] fix:True alt_baro:48.3m
    ```
  - Publishes to `/drone_0/situation` as `std_msgs/String`

**Afternoon (2h): Camera detection node**
- [ ] Create `gcs/camera_detection_node.py`:
  - Subscribes to `/camera_0/image_raw` (Gazebo camera plugin topic)
  - Runs YOLOv8 nano (CPU-only) on each frame
  - Publishes detection summary string to `/camera_0/detections`
  - Example: `"obstacle: person at 4m bearing 045"`, `"clear"`
- [ ] Add Gazebo camera plugin to drone SDF model for simulation

**Afternoon (1h): Wire SA into Lead NLU context**
- [ ] Update `lead_nlu_node.py` to subscribe to `/drone_0/situation`
- [ ] Inject SA block into Ollama prompt on each inference call
- [ ] Test: with Gazebo obstacle near drone, verify lead NLU prompt contains obstacle info

**Evening checkpoint:**
- [ ] SA block appears correctly in Lead NLU prompt
- [ ] Camera detection node publishes meaningful summaries
- [ ] End-to-end with SA context: speak → Lead NLU (with situation) → Drone-0 action

---

#### Day 13 (Sat) — Week 2 Integration Test

**Full pipeline test — Lead Pilot only:**
- [ ] Start SITL (2 drones in Gazebo)
- [ ] Start XRCE-DDS agent
- [ ] Launch `lead_pilot.launch.py` + `gcs` nodes
- [ ] Run 20 test commands (from benchmark dataset): 10 clear, 7 ambiguous, 3 OOS
- [ ] Record: confidence output, action taken, clarification triggered
- [ ] Calculate preliminary FER and confidence accuracy for Lead

**Week 2 Go/No-Go Checkpoint:**
- [ ] Lead Pilot controls Drone-0 end-to-end with voice
- [ ] Sensor aggregator provides context to Lead NLU
- [ ] Clarification cascade triggers on ambiguous/OOS commands for Lead
- [ ] GCS monitor displays live status

---

#### Day 14 (Sun) — Buffer + Schema Refinement

- [ ] Review all Pydantic schemas based on week 2 testing
- [ ] Fix any normaliser gaps discovered (new aliases found)
- [ ] Update system prompts based on observed Lead SLM failure modes
- [ ] Write `docs/week2_report.md`

---

### WEEK 3: Wingman Stack + Cross-PC Integration
**Goal:** Wingman SLM on PC-2 receives orders from Lead SLM on PC-1, executes on Drone-1, reports back.

---

#### Day 15 (Mon) — Wingman NLU Node

**Morning (3h): Wingman NLU node**
- [ ] Create `wingman_pilot/wingman_nlu_node.py`:
  - Subscribes to `/wingman/order` (WingmanOrder)
  - Builds Wingman system prompt with rank, rules, confidence gate policy
  - Injects WingmanOrder fields into prompt:
    - `mission_context` (Lead's NL brief)
    - `intent` (structured command)
    - `priority`
    - `lead_position`
  - Injects `/drone_1/situation` into prompt
  - Calls Ollama (local or PC-1 remote, based on Week 1 latency findings)
  - Validates output against `WingmanOutput` schema
  - Publishes to `/wingman/approved_intent` or `/wingman/clarification_to_lead`

**Afternoon (2h): Wingman confidence gate**
- [ ] Implement wingman-specific gate in `confidence_gate.py`:
  - HIGH → publish to `/wingman/approved_intent` immediately
  - MEDIUM → publish to `/wingman/approved_intent` with warning + update status_report
  - LOW → publish to `/wingman/clarification_to_lead` (NOT to human)
- [ ] Update Lead NLU to subscribe to `/wingman/clarification_to_lead` and handle escalation

**Afternoon (1h): Unit tests**
- [ ] Test: WingmanOrder arrives → WingmanOutput validated → correct routing
- [ ] Test: LOW confidence → clarification published to correct topic

**Evening checkpoint:**
- [ ] Wingman NLU node starts and processes a manually published WingmanOrder

---

#### Day 16 (Tue) — Wingman PX4 Commander + Sensor Aggregator

**Morning (3h): Wingman PX4 commander**
- [ ] Create `wingman_pilot/wingman_px4_commander_node.py`:
  - Same structure as lead commander but uses `/px4_1/fmu/in/...` topics
  - Subscribes to `/wingman/approved_intent`
  - Publishes setpoints on `/px4_1/fmu/in/trajectory_setpoint`
  - Publishes keepalive on `/px4_1/fmu/in/offboard_control_mode` at 10 Hz

**Morning (1h): Wingman sensor aggregator**
- [ ] Create `wingman_pilot/wingman_sensor_aggregator_node.py`:
  - Same as lead aggregator but uses `/px4_1/fmu/out/...` topics
  - Publishes to `/drone_1/situation`
  - Also publishes periodic `StatusReport` to `/wingman/status_report`

**Afternoon (3h): Wingman launch file + PC-2 deployment**
- [ ] Create `launch/wingman_pilot.launch.py`
- [ ] Copy `major_project` package to PC-2:
  ```bash
  rsync -av ~/major_ws/src/major_project/ user@PC2_IP:~/major_ws/src/major_project/
  ```
- [ ] Build on PC-2:
  ```bash
  cd ~/major_ws
  colcon build --packages-select major_project px4_msgs
  ```
- [ ] Start wingman stack on PC-2:
  ```bash
  ros2 launch major_project wingman_pilot.launch.py
  ```

**Evening checkpoint:**
- [ ] Wingman PX4 commander controls Drone-1 independently
- [ ] `/drone_1/situation` publishes from PC-2
- [ ] `/wingman/status_report` publishes from PC-2

---

#### Day 17 (Wed) — Cross-PC Integration: Full Lead→Wingman Chain

**Full day: End-to-end chain test**

**Morning (3h): Wire the cross-PC chain**
- [ ] PC-1: Launch lead stack (lead_pilot.launch.py + gcs nodes)
- [ ] PC-2: Launch wingman stack (wingman_pilot.launch.py)
- [ ] Verify on PC-1: `/wingman/status_report` echoes (from PC-2)
- [ ] Verify on PC-2: `/wingman/order` echoes (from PC-1)

**Afternoon (3h): Test the full chain**
- [ ] Test 1: Speak "take off to 10 meters and search northern area. Wingman cover the south."
  - Verify: Drone-0 takes off, Lead issues WingmanOrder for Drone-1
  - Verify: Drone-1 takes off and moves to southern area
  - Verify: Wingman publishes status report back to Lead
  - Verify: GCS monitor shows both drones moving
- [ ] Test 2: Speak "go that way" (ambiguous)
  - Verify: Lead sets confidence LOW, asks human for clarification
  - Verify: No WingmanOrder issued
- [ ] Test 3: Issue wingman-ambiguous order (e.g. "wingman investigate that")
  - Verify: Lead issues WingmanOrder, Wingman sets LOW confidence
  - Verify: Wingman publishes clarification_to_lead
  - Verify: Lead receives it, escalates to human clarification_request

**Evening checkpoint:**
- [ ] Full chain demonstrated: Voice → Lead → Wingman → Drone-1 moves
- [ ] Two-level clarification cascade demonstrated
- [ ] Both drones simultaneously visible and controlled in Gazebo

---

#### Day 18 (Thu) — Emergency Stop + Safety Systems

**Morning (2h): Emergency stop**
- [ ] Create `gcs/emergency_stop_node.py` on PC-1:
  - Keyboard shortcut (e.g., `Ctrl+E`) publishes `True` to `/emergency_stop`
  - Also accepts voice command "emergency land all" / "abort all"
  - On PC-1: sends land command to Drone-0 immediately
  - Broadcasts `/emergency_stop` to PC-2
- [ ] Create `wingman_pilot/emergency_subscriber_node.py` on PC-2:
  - Subscribes to `/emergency_stop`
  - On receipt: sends land MAVLink command to Drone-1 immediately
  - Does NOT wait for Wingman NLU

**Morning (1h): Battery failsafe**
- [ ] In both sensor aggregators: if battery < 20%, publish `WARN` to `/mission_status`
- [ ] In Lead NLU: if battery warning received, prepend `[BATTERY LOW - CONSIDER RTL]` to context

**Afternoon (2h): Watchdog timers**
- [ ] In Lead NLU: if `/wingman/status_report` not received for >10s, log warning and republish last order
- [ ] In Wingman NLU: if `/wingman/order` not received for >30s and drone is armed, hold position

**Evening checkpoint:**
- [ ] Emergency stop tested: Ctrl+E lands both drones within 2 seconds
- [ ] Battery warning appears in GCS monitor
- [ ] Watchdog timers tested by simulating lost connection

---

#### Day 19 (Fri) — Week 3 Integration Test + Config Files

**Morning (2h): Config files**
- [ ] Write `config/lead_config.yaml`:
  ```yaml
  lead_nlu:
    ollama_host: localhost
    ollama_port: 11434
    model: qwen2.5-coder:3b
    num_ctx: 2048
    inference_timeout: 10.0
    confidence_gate:
      medium_warn: true
      low_withhold: true
  px4_commander:
    namespace: ""           # drone 0
    setpoint_rate_hz: 10
    offboard_keepalive_hz: 10
  ```
- [ ] Write `config/wingman_config.yaml` (same structure, namespace: px4_1)

**Afternoon (3h): Full integration run**
- [ ] Run 30-command integration test across both drones
- [ ] Record all metrics: latency per stage, confidence accuracy, FER
- [ ] Document failure modes observed

**Week 3 Go/No-Go Checkpoint:**
- [ ] Full cross-PC Lead→Wingman chain working
- [ ] Two-level clarification cascade working
- [ ] Emergency stop working
- [ ] All major nodes running without crashes

---

#### Day 20 (Sat) — Stability Testing

- [ ] Run system for 30 continuous minutes without crash
- [ ] Run 50 automated test commands, record all results
- [ ] Fix any race conditions or topic timing issues discovered
- [ ] Profile CPU/RAM usage on both PCs

---

#### Day 21 (Sun) — Buffer + Week 3 Review

- [ ] Fix top-3 bugs from stability testing
- [ ] Write `docs/week3_report.md`
- [ ] Update Pydantic schemas if gaps found

---

### WEEK 4: SITL Validation Scenarios + Benchmark Dataset
**Goal:** Three documented SITL scenarios run successfully. Benchmark dataset evaluated on both SLMs.

---

#### Day 22 (Mon) — Scenario A: Formation Search (Clear Command)

**Scenario:** Human says "conduct a parallel search of the field. Drone-0 takes the north half, Drone-1 takes the south half."

**Expected:**
- Lead: HIGH confidence → approves own intent (move to north half, begin search)
- Lead: HIGH confidence on wingman order → issues WingmanOrder for south half
- Wingman: HIGH confidence → executes south-half search
- GCS monitor shows both drones in separate search patterns
- Both report completion status

**Tasks:**
- [ ] Define waypoints for N-half and S-half search patterns in Gazebo coordinate frame
- [ ] Add `search_grid` action type to both SLM prompts with waypoint list in parameters
- [ ] Run scenario 5 times, record: FER, confidence accuracy, task completion rate
- [ ] Save video recording of Gazebo session

---

#### Day 23 (Tue) — Scenario B: Clarification Cascade (Ambiguous)

**Scenario:** Human says "go investigate that anomaly" (no direction, no target specified).

**Expected flow:**
- Lead: LOW confidence → asks human "Which anomaly? Please specify direction or grid reference."
- Human responds: "northeast sector, bearing 045, 200m"
- Lead: HIGH confidence → approves action + issues WingmanOrder "Hold at current position while I investigate"
- Wingman: HIGH confidence → holds position, reports back

**Tasks:**
- [ ] Test the clarification loop end-to-end (voice → clarification → voice response → action)
- [ ] Verify clarification question is contextually appropriate (not generic)
- [ ] Test nested clarification: wingman also confused by follow-up order
- [ ] Run scenario 5 times, record clarification rate, cascade routing accuracy

---

#### Day 24 (Wed) — Scenario C: Emergency (Out-of-Scope / Safety)

**Scenario A:** Human says "go invisible" (physically impossible)
- Lead: LOW confidence → withholds, explains "That action is not within drone capabilities"
- No flight action taken on either drone

**Scenario B:** Human says "emergency land" 
- Lead: HIGH confidence (emergency keyword) → immediate land command to Drone-0
- Lead: issues emergency WingmanOrder: priority=emergency, action=land
- Wingman: HIGH confidence → immediate land
- GCS: shows both drones landing

**Tasks:**
- [ ] Test 5 out-of-scope commands (impossible, safety-violating, off-domain)
- [ ] Verify zero-execution on all 5
- [ ] Test emergency land scenario 5 times, measure time from voice to both drones landing

---

#### Day 25 (Thu) — Full Benchmark Dataset Evaluation

**Morning: Run 260 commands systematically**
- [ ] Write `benchmark/run_evaluation.py`:
  - Loads all 260 commands from `benchmark/dataset.json`
  - Publishes each command to `/voice_commands` with 15s inter-command gap
  - Records SLM outputs, confidence values, actions taken, clarifications triggered
  - Compares to expected confidence from dataset labels
  - Outputs `benchmark/results.csv`

**Afternoon: Analyse results**
- [ ] Calculate per-SLM metrics:
  - Lead FER, Lead confidence accuracy (HIGH/MEDIUM/LOW)
  - Wingman FER, Wingman confidence accuracy
  - Clarification cascade routing accuracy
  - Schema compliance rate (both SLMs)
- [ ] Write `benchmark/analysis.py` to generate tables and charts

**Evening checkpoint:**
- [ ] All 260 commands evaluated
- [ ] Results CSV generated
- [ ] Key metrics calculated

---

#### Day 26 (Fri) — Latency Benchmarking (Formal)

**Formal latency measurement — per pipeline stage:**
- [ ] Instrument each node with `time.perf_counter()` at entry and exit
- [ ] Publish timestamps to `/diagnostics/latency` topic
- [ ] Measure across N=100 clear commands:
  - Stage 1: STT (mic → /voice_commands)
  - Stage 2: Lead Ollama inference
  - Stage 3: Pydantic validation
  - Stage 4: ROS2 publish to /lead/approved_intent
  - Stage 5: Lead→Wingman order transmission (cross-PC)
  - Stage 6: Wingman Ollama inference
  - Stage 7: Wingman Pydantic validation
  - Stage 8: Wingman PX4 setpoint publish
  - Stage 9: PX4 actuator response (from topic timestamp)
- [ ] Record: mean, median, p95, p99 for each stage
- [ ] Determine total pipeline latency (Stage 1 → Stage 9)
- [ ] Compare to PX4 offboard requirement (>2 Hz = <500ms per cycle for NLU)

**Week 4 Go/No-Go Checkpoint:**
- [ ] 3 SITL scenarios documented and repeatable
- [ ] 260-command evaluation complete
- [ ] Formal latency measurements recorded per stage
- [ ] All key metrics calculated

---

#### Day 27–28 (Sat–Sun) — Buffer + Data Analysis

- [ ] Re-run any failed scenarios
- [ ] Investigate top-5 failure modes (wrong confidence, wrong action, schema fail)
- [ ] Write `docs/week4_report.md` with all metrics
- [ ] Begin paper outline

---

### WEEK 5: Ablation Studies + System Hardening
**Goal:** Validate novel contributions through ablation. Harden system for paper-quality results.

---

#### Day 29 (Mon) — Ablation 1: Without Wingman Confidence Gate

- [ ] Disable wingman confidence gate (pass all orders directly to PX4 commander)
- [ ] Re-run 90 ambiguous + 40 OOS commands on Wingman
- [ ] Record FER without gate vs with gate
- [ ] This ablation proves Contribution 2 (confidence gate at wingman level reduces FER)

---

#### Day 30 (Tue) — Ablation 2: Without Situational Awareness Context

- [ ] Disable sensor aggregator injection (remove SA block from SLM prompts)
- [ ] Re-run full 260-command evaluation
- [ ] Record confidence accuracy with vs without SA context
- [ ] This ablation proves SA context improves confidence calibration

---

#### Day 31 (Wed) — Ablation 3: Free-Form NL vs Structured Schema

- [ ] Implement alternate `wingman_nlu_free_nl.py` that passes lead's `situation_report` string (NL) instead of `WingmanOrder` (structured)
- [ ] Run 50 coordination commands with free-form NL inter-agent communication
- [ ] Compare: action accuracy, schema compliance, FER between structured vs NL
- [ ] This ablation proves Contribution 1 (schema-validated messages outperform free-form NL)

---

#### Day 32 (Thu) — System Hardening

- [ ] Fix all bugs discovered during ablation studies
- [ ] Add retry logic to Ollama calls (max 3 retries on timeout)
- [ ] Add graceful degradation: if Lead NLU times out, publish HOLD intent to Drone-0
- [ ] Add graceful degradation: if Wingman NLU times out, publish HOLD intent to Drone-1
- [ ] Test system under CPU load (simulate constrained hardware)

---

#### Day 33 (Fri) — Demo Preparation

- [ ] Record clean Gazebo demo video for each of 3 SITL scenarios
- [ ] Record GCS terminal display alongside Gazebo
- [ ] Prepare system for Prof. demo:
  - One-command startup scripts for PC-1 and PC-2
  - Clear README for replication
- [ ] Run full 260-command evaluation one final time for paper numbers

**Week 5 Go/No-Go Checkpoint:**
- [ ] 3 ablation studies complete with data
- [ ] Demo videos recorded
- [ ] System runs reliably for 60+ minutes
- [ ] Final evaluation numbers collected

---

#### Day 34–35 (Sat–Sun) — Buffer + Paper Outline

- [ ] Fix final issues from day 33 evaluation run
- [ ] Write full paper outline (sections, claims, figures, tables)
- [ ] Draft abstract

---

### WEEK 6: Paper Writing + Final Deliverables
**Goal:** Conference-ready paper draft. All code documented. Final demo delivered to Prof.

---

#### Day 36 (Mon) — Paper: Introduction + Related Work

- [ ] Write Section I: Introduction (problem, motivation, contribution summary)
- [ ] Write Section II: Related Work
  - LLM-based UAV control
  - Multi-agent LLM coordination
  - Uncertainty-aware robot control
  - Edge SLM deployment
- [ ] Add all citations from research report (arXiv:2605.03788, 2602.21670, 2506.07509, 2505.04364, etc.)

---

#### Day 37 (Tue) — Paper: System Architecture + Methodology

- [ ] Write Section III: System Architecture
  - Distributed 2-PC setup
  - ROS2 node graph
  - Network configuration
- [ ] Write Section IV: Confidence-Gating Mechanism
  - Extend minor project section
  - Two-level cascade policy (equation/formula)
  - WingmanOrder and StatusReport schemas
- [ ] Write Section V: Benchmark Dataset
  - 200 existing + 60 new commands
  - Category breakdown table

---

#### Day 38 (Wed) — Paper: Experimental Setup + Results

- [ ] Write Section VI: Experimental Setup
  - Hardware table (both PCs)
  - Software stack table
  - Models benchmarked
- [ ] Write Section VII: Results
  - Table: FER comparison (baseline vs single-gate vs two-level gate)
  - Table: Confidence accuracy (Lead vs Wingman, HIGH/MEDIUM/LOW)
  - Table: Latency per stage (mean, median, p95)
  - Table: Ablation results
  - Figure: System architecture diagram
  - Figure: SITL scenario sequence diagrams

---

#### Day 39 (Thu) — Paper: Discussion + Conclusion + Proofread

- [ ] Write Section VIII: Discussion
  - What worked, what failed, failure taxonomy (extend from minor project)
  - Minimum viable parameter threshold for multi-agent coordination
  - Latency viability analysis
- [ ] Write Section IX: Conclusion + Future Work
- [ ] Write Acknowledgements
- [ ] Full paper proofread pass

---

#### Day 40 (Fri) — Code Documentation + Final Deliverables

**Morning (3h): Code documentation**
- [ ] Add docstrings to all node files
- [ ] Write `README.md` for `major_project` package:
  - Prerequisites
  - PC-1 setup instructions
  - PC-2 setup instructions
  - How to run all 3 SITL scenarios
  - How to run benchmark evaluation
- [ ] Write `REPLICATION.md`: step-by-step reproduction guide

**Afternoon (2h): Final checklist**
- [ ] All code committed to Git
- [ ] Benchmark results CSV archived
- [ ] Demo videos in `docs/demo_videos/`
- [ ] Paper draft in `docs/paper_draft.pdf`
- [ ] Architecture document updated to reflect final implementation

**Final Demo:**
- [ ] Live demo to Prof.: 3 SITL scenarios, GCS monitor, clarification cascade
- [ ] Present paper draft
- [ ] Collect feedback

---

#### Day 41–42 (Sat–Sun) — Buffer + Prof. Feedback Integration

- [ ] Address Prof. feedback on paper draft
- [ ] Address Prof. feedback on system design
- [ ] Final paper revisions
- [ ] Prepare submission package

---

## Risk Register

| Risk | Probability | Impact | Mitigation |
|---|---|---|---|
| Ollama latency > 2000ms over WiFi | Medium | High | Use async NLU (last-known setpoint at >2Hz independently); or switch to 1.5b model |
| CycloneDDS multicast fails on WiFi router | Medium | High | Configure unicast peer list in cyclonedds.xml with explicit PC IPs |
| Qwen2.5-Coder:3b fails schema compliance at multi-agent scale | Medium | High | Fallback to 7b model on one PC; or add retry loop with simplified prompt |
| PX4 v1.17 multi-vehicle namespace behavior changed | Low | Medium | Pin to tested tag; check changelog against v1.14 |
| PC-1 CPU overloaded (Gazebo + 2 SITL + YOLO + Ollama) | High | Medium | Move Gazebo to PC-2; run Ollama on PC-2 for Lead (over WiFi); split load |
| Gazebo camera plugin not available for x500 model | Low | Medium | Use ground truth position plugin instead; skip camera, use text-only SA |
| Faster-Whisper mic capture fails on Ubuntu | Low | Low | Substitute pre-recorded audio files for evaluation; fix mic driver separately |
| Week slippage | Medium | Medium | Each week has a 1-day buffer (Days 7, 14, 21, 28, 35, 41–42) |

---

## Dependency Map

```
Week 1 (Infra) → Week 2 (Lead stack) → Week 3 (Wingman + cross-PC) → Week 4 (Eval)
                                                                              │
                                                                       Week 5 (Ablation)
                                                                              │
                                                                       Week 6 (Paper)
```

**Critical path:** Day 6 latency benchmark → Day 10 Lead commander → Day 16 Wingman → Day 17 cross-PC chain → Day 22-24 scenarios

---

## References

1. arXiv:2605.03788 — "Say the Mission, Execute the Swarm" (IEEE WoWMoM 2026)
2. arXiv:2602.21670 — "Hierarchical LLM-Based Multi-Agent Framework with Prompt Optimization for Multi-Robot Task Planning" (Feb 2026)
3. arXiv:2506.07509 — "Taking Flight with Dialogue" (June 2025)
4. arXiv:2505.04364 — SwarmBench (May 2025)
5. PX4 Multi-Vehicle Simulation Docs — https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation
6. PX4 ROS2 Multi-Vehicle Docs — https://docs.px4.io/main/en/ros2/multi_vehicle
7. ROS2 DDS Tuning Guide — https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html
8. Gohil, D. & Maiti, T.K. — "Confidence-Gated Intent Parsing for Voice-Controlled UAVs Using Edge-Deployed Small Language Models" (Minor Project, 2025)

---

*Proposal v1.0 — 2026-06-03*
*Based on deep research report: major_project_research.md*
*Architecture details: major_project_architecture.md*
