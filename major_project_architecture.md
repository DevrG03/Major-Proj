# Major Project: System Architecture
## Rank-Based Multi-SLM Drone Pilot System

**Version:** 0.1 (post-research, pre-implementation)
**Date:** 2026-06-03
**Extends:** Minor project — Confidence-Gated Intent Parsing (single drone)

---

## 1. System Overview

Two PCs connected over WiFi, each running one PX4 SITL drone and one SLM pilot. The Lead Pilot SLM (PC-1) receives commands from a human Ground Commander via voice, decomposes the mission, controls Drone-0, and issues structured orders to the Wingman SLM (PC-2). The Wingman controls Drone-1 and reports status back to the Lead.

```
┌─────────────────────────────────────────────────────────────────┐
│                    HUMAN GROUND COMMANDER                       │
│                   (Voice + Display terminal)                    │
└─────────────────────────┬───────────────────────────────────────┘
                          │ voice
                          ▼
╔═════════════════════════════════════════════╗  WiFi  ╔══════════════════════════════════════╗
║              PC-1  (Lead Pilot)             ║◄──────►║          PC-2  (Wingman)             ║
║                                             ║        ║                                      ║
║  [STT Node]  mic → /voice_commands          ║        ║  [Wingman NLU Node]                  ║
║       │                                     ║        ║   Qwen2.5-Coder:3b via Ollama        ║
║       ▼                                     ║        ║   /wingman/order → intent → cmd      ║
║  [Lead NLU Node]                            ║        ║       │                              ║
║   Qwen2.5-Coder:3b via Ollama               ║        ║       ▼                              ║
║   voice → mission decomposition             ║        ║  [Wingman PX4 Commander]             ║
║       │                                     ║        ║   /px4_1/fmu/...                     ║
║       ├──→ /lead/approved_intent            ║        ║       │                              ║
║       ├──→ /wingman/order ─────────────────►║        ║       ▼                              ║
║       └──→ /clarification_request           ║        ║  [PX4 SITL - Drone 1]                ║
║                                             ║◄───────║  /wingman/status_report              ║
║  [Lead PX4 Commander]                       ║        ║  /drone_1/situation                  ║
║   /px4_0/fmu/...                            ║        ║                                      ║
║       │                                     ║        ║  [Wingman Sensor Aggregator]         ║
║       ▼                                     ║        ║   Camera→YOLO, GPS, IMU, Baro        ║
║  [PX4 SITL - Drone 0 + Gazebo Server]       ║        ║   → /drone_1/situation               ║
║                                             ║        ║                                      ║
║  [Lead Sensor Aggregator]                   ║        ╚══════════════════════════════════════╝
║   Camera→YOLO, GPS, IMU, Baro               ║
║   → /drone_0/situation                      ║
║                                             ║
║  [XRCE-DDS Agent] ← both drones connect    ║
║  [Clarification Speaker Node]               ║
║  [Mission Monitor / GCS Display]            ║
╚═════════════════════════════════════════════╝
```

---

## 2. ROS2 Node Graph

### PC-1 Nodes

| Node | Subscribes | Publishes | Description |
|---|---|---|---|
| `stt_node` | mic (audio) | `/voice_commands` | Faster-Whisper v1.2.1, int8, offline |
| `lead_nlu_node` | `/voice_commands`, `/wingman/status_report`, `/drone_0/situation`, `/drone_1/situation` | `/lead/approved_intent`, `/wingman/order`, `/clarification_request`, `/mission_status` | Lead Pilot SLM — core node |
| `lead_px4_commander_node` | `/lead/approved_intent` | `/px4_0/fmu/in/vehicle_command` | Translates intents to PX4 setpoints |
| `lead_intent_bridge_node` | `/lead/approved_intent` | MAVLink (land, RTL) | Direct MAVLink commands |
| `lead_sensor_aggregator_node` | `/px4_0/fmu/out/vehicle_local_position`, `/px4_0/fmu/out/vehicle_status`, `/px4_0/fmu/out/battery_status`, `/camera_0/detections` | `/drone_0/situation` | Formats sensor data to text block |
| `camera_0_detection_node` | `/camera_0/image_raw` | `/camera_0/detections` | YOLO object detection, CPU-only |
| `clarification_speaker_node` | `/clarification_request` | terminal / TTS | Speaks clarification to commander |
| `mission_monitor_node` | `/mission_status`, `/drone_0/situation`, `/drone_1/situation` | terminal display | GCS dashboard |
| `emergency_stop_node` | keyboard input | `/emergency_stop` | Safety: broadcasts to all drones |

### PC-2 Nodes

| Node | Subscribes | Publishes | Description |
|---|---|---|---|
| `wingman_nlu_node` | `/wingman/order`, `/drone_1/situation` | `/wingman/approved_intent`, `/wingman/status_report`, `/wingman/clarification_to_lead` | Wingman SLM — core node |
| `wingman_px4_commander_node` | `/wingman/approved_intent` | `/px4_1/fmu/in/vehicle_command` | Translates intents to PX4 setpoints |
| `wingman_intent_bridge_node` | `/wingman/approved_intent` | MAVLink (land, RTL) | Direct MAVLink commands |
| `wingman_sensor_aggregator_node` | `/px4_1/fmu/out/vehicle_local_position`, `/px4_1/fmu/out/vehicle_status`, `/px4_1/fmu/out/battery_status`, `/camera_1/detections` | `/drone_1/situation`, `/wingman/status_report` | Formats sensor data to text block |
| `camera_1_detection_node` | `/camera_1/image_raw` | `/camera_1/detections` | YOLO object detection, CPU-only |
| `emergency_stop_subscriber_node` | `/emergency_stop` | MAVLink kill switch | Forces land on Drone-1 |

### Cross-PC ROS2 Topics (over WiFi DDS)

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/wingman/order` | PC-1 → PC-2 | `WingmanOrder` | Lead issues structured orders |
| `/wingman/status_report` | PC-2 → PC-1 | `StatusReport` | Wingman reports back |
| `/drone_1/situation` | PC-2 → PC-1 | `SituationalAwareness` | Wingman sensor state for lead's context |
| `/emergency_stop` | PC-1 → PC-2 | `std_msgs/Bool` | Kill switch |

---

## 3. Pydantic Message Schemas

### Existing (from minor project — unchanged)

```python
from pydantic import BaseModel
from typing import Optional, Literal, List

class FlightIntent(BaseModel):
    action: Literal["takeoff","move","hover","land","rtl",
                    "search","search_stop","search_resume","search_expand"]
    altitude: Optional[float] = None      # metres, 0.5–50
    distance: Optional[float] = None      # metres, 0.1–100
    direction: Optional[str] = None
    speed: Optional[float] = None
    then: Optional["FlightIntent"] = None  # chained command
    confidence: Literal["high","medium","low"]
    clarification_question: Optional[str] = None
```

### New: Lead → Wingman

```python
class WingmanOrder(BaseModel):
    order_id: str                          # UUID, for status tracking
    mission_context: str                   # NL brief: "Search eastern quadrant while I hold perimeter"
    intent: FlightIntent                   # structured command (reuses existing schema)
    priority: Literal["routine","urgent","emergency"]
    lead_position: dict                    # {"x": 0.0, "y": 0.0, "z": 50.0, "heading": 90}
    confidence: Literal["high","medium","low"]  # lead's confidence in this order
    clarification_question: Optional[str] = None  # if lead is unsure, asks wingman
```

### New: Wingman → Lead

```python
class StatusReport(BaseModel):
    order_id: str                          # echoes the WingmanOrder.order_id
    status: Literal["acknowledged","executing","completed","failed","needs_clarification"]
    drone_position: dict                   # {"x": 5.0, "y": 0.0, "z": 50.0}
    battery_pct: float
    obstacle_detected: bool
    situation_summary: str                 # NL: "Moving to eastern quadrant. Path clear. ETA 45s."
    clarification_question: Optional[str] = None  # if wingman needs clarification from lead
    confidence: Literal["high","medium","low"]
```

### New: Situational Awareness (prompt injection block)

```python
class SituationalAwareness(BaseModel):
    timestamp: float
    drone_id: str                          # "LEAD" or "WINGMAN"
    position: dict                         # {"x","y","z","heading","speed"}
    battery_pct: float
    flight_mode: str                       # PX4 mode: OFFBOARD, HOLD, etc.
    camera_summary: str                    # "Forward clear" / "Obstacle at 3m bearing 045"
    gps_fix: bool
    altitude_baro: float
```

---

## 4. SLM Prompt Architecture

### Lead Pilot System Prompt

```
You are LEAD PILOT of a 2-drone formation. Rank: LEAD.
Responsibilities:
  1. Execute your own drone (Drone-0) flight commands
  2. Issue orders to your Wingman (Drone-1)  
  3. Maintain mission situational awareness
  4. Report mission status to Ground Commander

Confidence gating policy (same as minor project):
  high   → execute immediately, issue wingman order immediately
  medium → execute with warning, ask wingman to confirm understanding
  low    → withhold, request clarification from Ground Commander

Output JSON with these fields:
  my_intent         : FlightIntent (for your own drone, or null if no action)
  wingman_order     : WingmanOrder (for Drone-1, or null if no order)
  confidence        : "high" | "medium" | "low"
  situation_report  : string (NL summary for GCS display — the "radio chatter")
  clarification_question : string | null
```

### Lead Pilot Prompt — Context Injection

```
[SITUATIONAL AWARENESS]
[DRONE-0 | LEAD]     pos: (0.0, 0.0, 50.0) | hdg: 090 | spd: 2.1m/s | bat: 87% | mode: OFFBOARD
[DRONE-1 | WINGMAN]  pos: (5.0, 0.0, 50.0) | hdg: 090 | spd: 0.0m/s | bat: 84% | mode: HOLD
[CAMERA-0]           Forward clear. No obstacles.
[MISSION]            Phase: patrol | objective: search grid A | coverage: 23%
[WINGMAN REPORT]     "Holding at waypoint Bravo. Battery 84%. Ready for next assignment." [status: executing]

[GROUND COMMANDER]
"Search the eastern sector and report any anomalies."
```

### Wingman System Prompt

```
You are WINGMAN PILOT of a 2-drone formation. Rank: WINGMAN.
Responsibilities:
  1. Execute orders from Lead Pilot precisely
  2. Report your status and situation back to Lead
  3. Request clarification from Lead (NOT from Ground Commander) if order is ambiguous

Confidence gating policy:
  high   → execute immediately
  medium → execute with warning, report assumption to Lead
  low    → withhold, request clarification from Lead

Output JSON with these fields:
  intent                 : FlightIntent (your action, or null)
  status_report          : StatusReport
  confidence             : "high" | "medium" | "low"
  clarification_question : string | null (directed at Lead, not human)
```

---

## 5. Confidence Gate — Extended Policy

The minor project's gate is extended to the full hierarchy:

```
Gate(c) for LEAD:
  high   → execute + send WingmanOrder immediately
  medium → execute + send WingmanOrder with warning to GCS
  low    → withhold + request clarification from Human Commander

Gate(c) for WINGMAN:
  high   → execute immediately + report to Lead
  medium → execute + report assumption to Lead
  low    → withhold + request clarification from Lead (Lead may escalate to human)
```

This creates a **two-level clarification cascade** — the wingman never speaks directly to the human commander.

---

## 6. Network Configuration

```
Both PCs:
  ROS_DOMAIN_ID=42
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  CYCLONEDDS_URI=<see DDS tuning guide for WiFi multicast>

PC-1:
  UXRCE_DDS_KEY=1  (Drone-0)
  Runs: micro-xrce-dds-agent udp4 -p 8888

PC-2:
  UXRCE_DDS_KEY=2  (Drone-1)
  Connects DDS agent at PC-1 IP: udp4 -h <PC1_IP> -p 8888

PX4 SITL Instances:
  Drone-0: px4_instance=0, namespace=default
  Drone-1: px4_instance=1, namespace=px4_1
```

---

## 7. Launch Sequence

```bash
# PC-1 Terminal 1: Start Gazebo + Drone-0
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 0

# PC-1 Terminal 2: Start Drone-1 SITL (connects to existing Gazebo server on PC-1)
# Note: PC-2 runs its PX4 SITL connecting to PC-1's Gazebo via network
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1

# PC-1 Terminal 3: XRCE-DDS agent
MicroXRCEAgent udp4 -p 8888

# PC-1 Terminal 4: ROS2 nodes (lead stack)
ros2 launch major_project lead_pilot.launch.py

# PC-2 Terminal 1: ROS2 nodes (wingman stack)
ros2 launch major_project wingman_pilot.launch.py
```

---

## 8. Sensor Pipeline Per Drone

```
Camera (video stream)
    └─► camera_N_detection_node (YOLO/MobileNet, CPU)
            └─► /camera_N/detections → sensor_aggregator_node

GPS + IMU + Barometer
    └─► PX4 EKF2 (onboard state estimation)
            └─► /px4_N/fmu/out/vehicle_local_position
            └─► /px4_N/fmu/out/vehicle_status
            └─► /px4_N/fmu/out/battery_status
                    └─► sensor_aggregator_node

sensor_aggregator_node
    └─► formats SituationalAwareness text block
    └─► publishes /drone_N/situation  (every 1s)
    └─► injects into SLM prompt on each inference cycle
```

---

## 9. Novel Contributions (Research Gaps This Project Fills)

1. **Minimum inter-agent message schema for sub-1B SLM drone coordination** (WingmanOrder + StatusReport schemas above — not defined in any prior paper)
2. **Confidence-gated hierarchical command propagation** — two-level gate: human→lead→wingman, each layer has confidence check
3. **Empirical latency benchmark** of Qwen2.5-Coder:3b in ROS2 WiFi control loop (explicitly noted as a gap in arXiv:2506.07509)
4. **Rank-based SLM pilot system with clarification cascade** — no paper implements Air Force-style rank hierarchy; closest is hierarchical planners using GPT-4o

---

## 10. Implementation Phases

### Phase 1 — Infrastructure (Week 1-2)
- [ ] Set up ROS2 Humble + PX4 v1.17 + Gazebo Garden on both PCs
- [ ] Verify CycloneDDS cross-PC topic discovery over WiFi
- [ ] Launch 2-drone SITL on PC-1, confirm `/px4_1/fmu/...` namespacing
- [ ] **Benchmark Ollama latency over WiFi in ROS2 loop** (resolve #1 risk)

### Phase 2 — Single-Drone Lead (Week 3-4)
- [ ] Port minor project NLU stack to this codebase
- [ ] Add `lead_sensor_aggregator_node` (GPS+IMU+Baro text block)
- [ ] Add camera detection node (YOLO, CPU-only)
- [ ] Test Lead Pilot SLM controlling Drone-0 with situational awareness injection

### Phase 3 — Wingman Integration (Week 5-6)
- [ ] Implement `WingmanOrder` and `StatusReport` Pydantic schemas
- [ ] Implement `wingman_nlu_node` with confidence gate
- [ ] Wire `/wingman/order` and `/wingman/status_report` cross-PC topics
- [ ] Test Lead issuing order → Wingman executing → Wingman reporting back

### Phase 4 — Mission Scenarios (Week 7-8)
- [ ] Implement 3 SITL validation scenarios:
  - Scenario A: Clear command — formation search (lead + wingman cover different sectors)
  - Scenario B: Ambiguous command — clarification cascade (wingman asks lead, lead asks human)
  - Scenario C: Emergency — lead broadcasts emergency stop to wingman
- [ ] Benchmark confidence accuracy, false execution rate (compare to minor project baseline)

### Phase 5 — Paper & Evaluation (Week 9-10)
- [ ] Latency benchmarks across all 5 pipeline stages
- [ ] Confidence calibration evaluation (HIGH/MEDIUM/LOW accuracy per drone)
- [ ] Ablation: with vs without wingman confidence gate
- [ ] Write paper

---

*Architecture v0.1 — subject to revision after Phase 1 latency benchmarks.*
