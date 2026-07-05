# Multi-Drone SLM Pilot System — Architecture Reference

> Complete technical reference for the 2-drone ROS2 + PX4 SITL + Ollama agentic system.
> Use this document to understand, modify, or extend any component.
> Source of truth: tutorial_merged.md (Sections 1–10).

---

## Table of Contents

1. [System Topology](#1-system-topology)
2. [Hardware & Software Stack](#2-hardware--software-stack)
3. [Package Structure](#3-package-structure)
4. [Node Inventory](#4-node-inventory)
5. [ROS2 Topic Map](#5-ros2-topic-map)
6. [Python Module Reference](#6-python-module-reference)
7. [Agent Loop — Deep Dive](#7-agent-loop--deep-dive)
8. [Tool Registry Reference](#8-tool-registry-reference)
9. [Context Window & Memory System](#9-context-window--memory-system)
10. [Inter-Agent Communication Protocol](#10-inter-agent-communication-protocol)
11. [Safety Architecture](#11-safety-architecture)
12. [Configuration Reference](#12-configuration-reference)
13. [Launch File Reference](#13-launch-file-reference)
14. [Data Flow Walkthrough](#14-data-flow-walkthrough)
15. [How to Modify or Extend](#15-how-to-modify-or-extend)
16. [Appendices](#appendices)

---

## 1. System Topology

```
┌──────────────────────────────────────────────────────────────────────┐
│                         PC-1 (Lead Drone)                            │
│  IP: 10.34.211.86    ROS_DOMAIN_ID=42    Ubuntu 26.04 LTS            │
│                                                                      │
│  ┌─────────────────┐   ┌──────────────────────────────────────────┐  │
│  │  PX4 SITL       │   │  ROS2 Lyrical                            │  │
│  │  Drone-0        │   │                                          │  │
│  │  Gazebo Harmonic│◄──│  lead_sensor_aggregator_node             │  │
│  │                 │   │  camera_detection_node                   │  │
│  │  namespace:     │   │  safety_monitor_node                     │  │
│  │  /fmu/          │◄──│  lead_agent_node  ←── Ollama             │  │
│  │                 │   │  lead_px4_commander_node                 │  │
│  │                 │◄──│  lead_intent_bridge_node                 │  │
│  └────────┬────────┘   │  stt_node                               │  │
│           │            │  clarification_speaker_node             │  │
│  MicroXRCE│            │  mission_monitor_node                   │  │
│  DDS port │            │  emergency_stop_node                    │  │
│  8888     │            └──────────────────────────────────────────┘  │
│           │                                                          │
└───────────┼──────────────────────────────────────────────────────────┘
            │ DDS (CycloneDDS over WiFi, port 7400)
┌───────────┼──────────────────────────────────────────────────────────┐
│           │              PC-2 (Wingman Drone)                        │
│  IP: 10.34.211.15    ROS_DOMAIN_ID=42    Ubuntu 26.04 LTS            │
│           │                                                          │
│  ┌────────┴────────┐   ┌──────────────────────────────────────────┐  │
│  │  PX4 SITL       │   │  ROS2 Lyrical                            │  │
│  │  Drone-1        │   │                                          │  │
│  │  Gazebo Harmonic│◄──│  wingman_sensor_aggregator_node          │  │
│  │                 │   │  wingman_agent_node  ←── Ollama          │  │
│  │  namespace:     │◄──│  wingman_px4_commander_node              │  │
│  │  /px4_1/fmu/    │   │                                          │  │
│  └─────────────────┘   └──────────────────────────────────────────┘  │
│  MicroXRCE DDS port 8888 (separate instance)                         │
└──────────────────────────────────────────────────────────────────────┘

Cross-PC topics (DDS bridged over WiFi, automatic with same ROS_DOMAIN_ID):
  /agent/lead_to_wingman        Lead → Wingman orders/messages
  /agent/wingman_to_lead        Wingman → Lead reports/queries
  /safety/event                 Safety Monitor → All nodes
  /wingman/order                Lead → Wingman (legacy WingmanOrder JSON)
  /mission_status               Lead → GCS monitor
  /clarification_request        Lead → GCS speaker
```

---

## 2. Hardware & Software Stack

### 2.1 Both PCs

| Component | Value |
|---|---|
| OS | Ubuntu 26.04 LTS |
| ROS | ROS2 Lyrical |
| DDS middleware | CycloneDDS |
| `ROS_DOMAIN_ID` | 42 |
| Python | 3.12+ |
| Pydantic | v2 (with `model_rebuild()`) |
| Ollama | latest |
| SLM model | `qwen3.5:2b` |

### 2.2 PC-1 Only

| Component | Value |
|---|---|
| Simulator | Gazebo Harmonic |
| PX4 build | `px4_sitl_default` |
| MicroXRCE-DDS agent | Port 8888 |
| Camera source | Gazebo camera plugin → `/camera/image_raw` |
| YOLO model | YOLOv8-nano (`yolov8n.pt`) |
| Ollama context window | `num_ctx=2048` (Lead agent) |

### 2.3 PC-2 Only

| Component | Value |
|---|---|
| Simulator | Gazebo Harmonic (separate SITL) |
| PX4 instance flag | `-i 1` → namespace `/px4_1/fmu/` |
| Ollama context window | `num_ctx=1024` (Wingman agent) |

### 2.4 PX4 Drone Namespaces

| Drone | ROS2 Prefix | Target System ID | PX4 Flag |
|---|---|---|---|
| Drone-0 (Lead) | `/fmu/` | 1 | default |
| Drone-1 (Wingman) | `/px4_1/fmu/` | 2 | `-i 1` |

---

## 3. Package Structure

```
major_ws/
└── src/
    └── major_project/
        ├── setup.py                           ← entry points for all 13 executables
        ├── package.xml
        ├── requirements.txt
        ├── config/
        │   ├── lead_config.yaml              ← all Lead node ROS2 parameters
        │   └── wingman_config.yaml           ← all Wingman node ROS2 parameters
        ├── launch/
        │   ├── lead_pilot.launch.py          ← launches all 10 Lead nodes
        │   └── wingman_pilot.launch.py       ← launches all 3 Wingman nodes
        └── major_project/
            ├── __init__.py
            ├── common/
            │   ├── schemas.py                ← Pydantic data models + expand_compact_values
            │   ├── ollama_client.py          ← HTTP wrapper for Ollama /api/generate
            │   ├── confidence_gate.py        ← confidence threshold helper (legacy)
            │   ├── normaliser.py             ← text normalisation utilities
            │   ├── tool_registry.py          ← all tool definitions + execute dispatcher
            │   ├── context_manager.py        ← bounded prompt window with compression
            │   └── agent_memory.py           ← SQLite-backed remember/recall
            ├── gcs/
            │   ├── stt_node.py               ← microphone → /voice_commands
            │   ├── clarification_speaker_node.py  ← /clarification_request → TTS
            │   ├── mission_monitor_node.py   ← /mission_status → terminal display
            │   └── emergency_stop_node.py    ← kills all flight on E-stop
            ├── lead_pilot/
            │   ├── lead_sensor_aggregator_node.py  ← PX4 telemetry → /drone_0/situation
            │   ├── camera_detection_node.py        ← YOLO → /camera_0/detections + obstacle_vector
            │   ├── safety_monitor_node.py          ← hard battery/GPS rules, no SLM
            │   ├── lead_agent_node.py              ← Lead SLM agent loop (main brain)
            │   ├── lead_px4_commander_node.py      ← FlightIntent JSON → PX4 OFFBOARD commands
            │   ├── lead_intent_bridge_node.py      ← intent re-publishing bridge
            │   └── prompts/
            │       └── lead_agent_system.txt       ← Lead system prompt (~250 tokens)
            └── wingman_pilot/
                ├── wingman_sensor_aggregator_node.py  ← PX4 telemetry → /drone_1/situation
                ├── wingman_agent_node.py               ← Wingman SLM agent loop
                ├── wingman_px4_commander_node.py       ← FlightIntent JSON → PX4 commands
                └── prompts/
                    └── wingman_agent_system.txt        ← Wingman system prompt
```

**Entry points in setup.py (all 13):**
```
stt_node, clarification_speaker, mission_monitor, emergency_stop,
camera_detection, lead_sensor_aggregator, lead_px4_commander,
lead_intent_bridge, lead_agent, safety_monitor,
wingman_sensor_aggregator, wingman_px4_commander, wingman_agent
```

---

## 4. Node Inventory

### 4.1 `lead_sensor_aggregator_node`

**File:** `lead_pilot/lead_sensor_aggregator_node.py`
**Purpose:** Aggregates raw PX4 telemetry + camera detections + temporal context into one human-readable situation string, published at 1 Hz to `/drone_0/situation`. This string is the Lead agent's primary world view each inference cycle.

**Subscribes to:**

| Topic | Type | Used For |
|---|---|---|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | x, y, z, vx, vy, vz in NED frame |
| `/fmu/out/battery_status` | `BatteryStatus` | `remaining` (0.0–1.0) |
| `/fmu/out/vehicle_status` | `VehicleStatus` | arming state, nav_state (flight mode) |
| `/fmu/out/vehicle_gps_position` | `SensorGps` | fix_type, satellites_used |
| `/camera_0/detections` | `String` | human-readable detection text |
| `/camera_0/obstacle_vector` | `String` | `"label:dir:dist"` machine tokens |
| `/mission/plan` | `String` | mission phase tracking (legacy, harmless) |
| `/mission/step_assessment` | `String` | mission phase tracking (legacy, harmless) |

**Publishes to:**

| Topic | Type | Rate |
|---|---|---|
| `/drone_0/situation` | `String` | 1 Hz |

**Situation string format:**
```
bat:90% alt:10.2m mode:OFFBOARD armed:YES gps:OK(12sat)
pos:(49.8,0.2) vel:(0.1,0.0,-0.0)
camera:person(91%) ahead close | car(78%) right far
obstacle:person:ahead:close
temporal:elapsed=45s dist_home=50m phase=searching
```

**Temporal enrichment fields:**
- `elapsed` — seconds since node startup (proxy for mission time)
- `dist_home` — Euclidean distance from (0,0) in metres (proxy for distance from launch)
- `phase` — string state: `idle / executing / searching / returning`

**Parameters (lead_config.yaml):**
```yaml
lead_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0
    gps_min_satellites: 6
```

**To modify:** Change `publish_situation()` to add/remove fields. The format of this string directly determines what the agent sees.

---

### 4.2 `wingman_sensor_aggregator_node`

**File:** `wingman_pilot/wingman_sensor_aggregator_node.py`
**Purpose:** Same as Lead aggregator for Drone-1. Uses `/px4_1/fmu/` namespace. No temporal enrichment or camera subscription (simpler node).

**Subscribes to:** Same PX4 topics under `/px4_1/fmu/` prefix.
**Publishes to:** `/drone_1/situation` (String, 1 Hz)

---

### 4.3 `camera_detection_node`

**File:** `lead_pilot/camera_detection_node.py`
**Purpose:** Runs YOLOv8-nano on Lead drone's camera feed at 2 Hz. Publishes detection text for human context AND structured obstacle tokens for machine parsing.

**Subscribes to:**

| Topic | Type | Source |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin |

**Publishes to:**

| Topic | Type | Example Content |
|---|---|---|
| `/camera_0/detections` | `String` | `"person(91%) ahead close \| car(78%) right far"` |
| `/camera_0/obstacle_vector` | `String` | `"person:ahead:close \| car:right:far"` |

**Monocular distance heuristic** (bbox width fraction of frame width):

| Fraction | Label |
|---|---|
| ≥ 30% | `very_close` |
| ≥ 10% | `close` |
| ≥ 3% | `medium` |
| < 3% | `far` |

**Direction heuristic** (bbox centre x relative to frame centre):
```
rel = (cx - frame_w/2) / (frame_w/2)
rel < -0.35  →  "left"
rel > +0.35  →  "right"
else         →  "ahead"
```

**Parameters:**
```yaml
camera_detection_node:
  ros__parameters:
    model_path: "yolov8n.pt"
    confidence_threshold: 0.6
    publish_rate_hz: 2.0
```

**To change model:** Update `model_path` in config. Any Ultralytics YOLO model works.
**To change thresholds:** Edit `DIST_THRESHOLDS` list and `_bbox_direction` constants in the node file.
**To add Wingman camera:** Duplicate this node with `/camera_1/` topics, launch on PC-2.

---

### 4.4 `safety_monitor_node`

**File:** `lead_pilot/safety_monitor_node.py`
**Purpose:** Hard safety rules with no SLM involvement. Monitors battery and GPS for BOTH drones. Directly commands PX4 RTL when thresholds are crossed. The SLM agent cannot override these actions.

**Subscribes to:**

| Topic | Drone | Threshold |
|---|---|---|
| `/fmu/out/battery_status` | Drone-0 | warn ≤ 20%, RTL ≤ 15% |
| `/px4_1/fmu/out/battery_status` | Drone-1 | warn ≤ 20%, RTL ≤ 15% |
| `/fmu/out/vehicle_gps_position` | Drone-0 | RTL if fix_type < 3 |
| `/px4_1/fmu/out/vehicle_gps_position` | Drone-1 | RTL if fix_type < 3 |

**Publishes to:**

| Topic | When |
|---|---|
| `/safety/event` | On every warn/RTL trigger (JSON payload) |
| `/fmu/in/vehicle_command` | RTL VehicleCommand × 3 for Drone-0 |
| `/px4_1/fmu/in/vehicle_command` | RTL VehicleCommand × 3 for Drone-1 |

**Safety event JSON format:**
```json
{
  "event_type": "battery_rtl",
  "drone_id": "drone_0",
  "severity": "critical",
  "message": "Battery 14.2% — RTL forced",
  "value": 14.2
}
```

**Per-drone state flags** (prevent repeated triggers):
- `_drone0_warned`, `_drone0_rtl_triggered`
- `_drone1_warned`, `_drone1_rtl_triggered`

**Parameters:**
```yaml
safety_monitor_node:
  ros__parameters:
    battery_warn_pct: 20.0
    battery_rtl_pct: 15.0
    gps_min_fix_type: 3
```

**To add a new rule:** Add subscriber + threshold callback + `/safety/event` publish + optional VehicleCommand. Follow the existing battery pattern.

---

### 4.5 `lead_agent_node`

**File:** `lead_pilot/lead_agent_node.py`
**Purpose:** The always-active SLM brain for Drone-0. Runs a continuous think-act-observe loop in a daemon thread. Replaces the old lead_nlu + mission_executor + mission_memory nodes entirely.

**Subscribes to:**

| Topic | Handler | Effect |
|---|---|---|
| `/drone_0/situation` | `_on_situation` | Updates `own_situation`; parses `battery_pct` via regex |
| `/camera_0/detections` | `_on_camera` | Updates `camera_summary` |
| `/camera_0/obstacle_vector` | `_on_obstacle` | Updates `obstacle_vector` |
| `/voice_commands` | `_on_voice` | New goal OR fills `_human_response` (dual-purpose) |
| `/agent/wingman_to_lead` | `_on_wingman_message` | Injects into context inter-agent block |
| `/safety/event` | `_on_safety_event` | Injects into context memory block |

**Publishes to:**

| Topic | Trigger |
|---|---|
| `/lead/approved_intent` | Every flight tool call (takeoff/move/hover/search/land/rtl) |
| `/agent/lead_to_wingman` | `message_wingman` tool call |
| `/wingman/order` | Legacy (not used by wingman_agent_node; kept for compat) |
| `/clarification_request` | `ask_human`, `notify_human`, mission_complete |
| `/mission_status` | After every tool call (`{"lead":"...", "wingman":"—"}` JSON) |

**Thread model:**
```
Main thread:  rclpy.spin()  — handles all ROS2 subscriptions
Daemon thread: _agent_loop() — runs inference, blocks on ask_human
Shared state via: threading.Lock (self.lock)
```

**Voice command dual-purpose logic:**
```python
def _on_voice(text):
    if self._waiting_for_human:
        self._human_response = text
        self._human_event.set()     # unblocks ask_human tool in daemon thread
    else:
        self._assign_goal(text)     # starts or replaces current mission
```

**Mission lifecycle:**
```
_assign_goal(text)
  → ctx.clear_history()   # wipe old context
  → ctx.set_goal(text)    # set new goal
  → _mission_done = False
  → if not _agent_running: start Thread(_agent_loop)
  # If already running: goal replaces old one, loop continues
```

**Parameters:**
```yaml
lead_agent_node:
  ros__parameters:
    ollama_host: "localhost"
    ollama_port: 11434
    model: "qwen3.5:2b"
    num_ctx: 2048
    loop_pause_sec: 0.5
```

---

### 4.6 `wingman_agent_node`

**File:** `wingman_pilot/wingman_agent_node.py`
**Purpose:** Same architecture as Lead agent but for Drone-1 with these differences:

| Property | Lead Agent | Wingman Agent |
|---|---|---|
| `num_ctx` default | 2048 | 1024 |
| Human comms | `ask_human`, `notify_human` | None |
| Peer comms | `message_wingman` | `ask_lead`, `notify_lead`, `message_lead` |
| Goal source topic | `/voice_commands` | `/agent/lead_to_wingman` |
| Mission complete action | Publishes to `/clarification_request` | Publishes `{type:status,...}` to `/agent/wingman_to_lead` |
| Wingman query pending flag | `_wingman_query_pending` | N/A |

**Backward compatibility:** `_on_legacy_order()` converts old `WingmanOrder` JSON from `/wingman/order` into a natural language goal string. Allows a Part 5 Lead node to control a Part 6 Wingman agent.

**Parameters:**
```yaml
wingman_agent_node:
  ros__parameters:
    ollama_host: "localhost"
    ollama_port: 11434
    model: "qwen3.5:2b"
    num_ctx: 1024
    loop_pause_sec: 0.5
```

---

### 4.7 `lead_px4_commander_node`

**File:** `lead_pilot/lead_px4_commander_node.py`
**Purpose:** Translates `FlightIntent` JSON into PX4 OFFBOARD mode commands. Runs a 10 Hz keepalive loop (PX4 exits OFFBOARD if no setpoints for >0.5s).

**Subscribes to:**

| Topic | Type |
|---|---|
| `/lead/approved_intent` | `String` (FlightIntent JSON) |
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` |
| `/fmu/out/vehicle_status` | `VehicleStatus` |

**Publishes to:**

| Topic | Type | Rate |
|---|---|---|
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | 10 Hz |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | 10 Hz |
| `/fmu/in/vehicle_command` | `VehicleCommand` | On arm/mode-switch/land/RTL |

**NED frame convention:**
- `target_x` = North offset (+ = North)
- `target_y` = East offset (+ = East)
- `target_z` = Down (−altitude; e.g. altitude 10m → `target_z = -10.0`)

**Direction offset table (DIRECTION_OFFSETS):**
```python
'north':     (1, 0),      'south':     (-1, 0),
'east':      (0, 1),      'west':      (0, -1),
'northeast': (0.707, 0.707),  'northwest': (0.707, -0.707),
'southeast': (-0.707, 0.707), 'southwest': (-0.707, -0.707),
```

**Altitude in move (integrated Part 6 patch):**
```python
new_alt = data.get('altitude', None)
self.target_z = -float(new_alt) if new_alt is not None else z
```
If the `FlightIntent` JSON contains an `altitude` key, the drone changes altitude as part of the move. Otherwise it maintains current altitude.

**To add a new action:** Add `elif action == 'new_action':` in `_on_intent()`, set `self.target_x/y/z`.

---

### 4.8 `wingman_px4_commander_node`

**File:** `wingman_pilot/wingman_px4_commander_node.py`
**Purpose:** Identical to Lead commander but:
- Subscribes to `/wingman/approved_intent`
- Publishes to `/px4_1/fmu/in/*` topics
- Sets `target_system = 2` in all VehicleCommand messages

---

### 4.9 `lead_intent_bridge_node`

**File:** `lead_pilot/lead_intent_bridge_node.py`
**Purpose:** Re-publishes `/lead/approved_intent` to the commander, handling mode-switch ordering. Uses `_BRIDGE_MARKER` tag to prevent echo loops.

---

### 4.10 GCS Nodes

All on PC-1. None interact with the SLM.

| Node | File | Subscribes | Publishes | Purpose |
|---|---|---|---|---|
| `stt_node` | `gcs/stt_node.py` | Microphone (via `speech_recognition`) | `/voice_commands` (String) | Converts speech → text |
| `clarification_speaker_node` | `gcs/clarification_speaker_node.py` | `/clarification_request` (String) | Speaker (via `pyttsx3`) | Speaks agent questions aloud |
| `mission_monitor_node` | `gcs/mission_monitor_node.py` | `/mission_status` (String) | Terminal display | Parses `{"lead":"...","wingman":"..."}` and shows status |
| `emergency_stop_node` | `gcs/emergency_stop_node.py` | `/emergency_stop` + keyboard | `/fmu/in/vehicle_command`, `/px4_1/fmu/in/vehicle_command` | Instant kill-switch for both drones |

---

## 5. ROS2 Topic Map

All topics share `ROS_DOMAIN_ID=42`. CycloneDDS bridges them across PCs over WiFi automatically — no explicit bridging config needed as long as both PCs are on the same subnet and domain ID.

### 5.1 Sensor / State

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | PX4 SITL | lead_sensor_aggregator, lead_px4_commander |
| `/fmu/out/battery_status` | `BatteryStatus` | PX4 SITL | lead_sensor_aggregator, safety_monitor |
| `/fmu/out/vehicle_status` | `VehicleStatus` | PX4 SITL | lead_sensor_aggregator, lead_px4_commander |
| `/fmu/out/vehicle_gps_position` | `SensorGps` | PX4 SITL | lead_sensor_aggregator, safety_monitor |
| `/px4_1/fmu/out/*` | same types | PX4 SITL (Drone-1) | wingman_sensor_aggregator, safety_monitor, wingman_px4_commander |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera | camera_detection_node |
| `/drone_0/situation` | `String` | lead_sensor_aggregator | lead_agent_node |
| `/drone_1/situation` | `String` | wingman_sensor_aggregator | wingman_agent_node |
| `/camera_0/detections` | `String` | camera_detection_node | lead_sensor_aggregator, lead_agent_node |
| `/camera_0/obstacle_vector` | `String` | camera_detection_node | lead_sensor_aggregator |

### 5.2 Intent / Command

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/lead/approved_intent` | `String` (FlightIntent JSON) | lead_agent_node | lead_intent_bridge, lead_px4_commander |
| `/wingman/approved_intent` | `String` (FlightIntent JSON) | wingman_agent_node | wingman_px4_commander |
| `/wingman/order` | `String` (WingmanOrder JSON) | lead_agent_node | wingman_agent_node (`_on_legacy_order`) |

### 5.3 PX4 Control Inputs

| Topic | Type | Publisher | Rate |
|---|---|---|---|
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | lead_px4_commander | 10 Hz |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | lead_px4_commander | 10 Hz |
| `/fmu/in/vehicle_command` | `VehicleCommand` | lead_px4_commander, safety_monitor, emergency_stop | On demand |
| `/px4_1/fmu/in/*` | same types | wingman_px4_commander, safety_monitor | same |

### 5.4 Agent Communication

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/agent/lead_to_wingman` | `String` | lead_agent_node | wingman_agent_node | Natural language; any content |
| `/agent/wingman_to_lead` | `String` | wingman_agent_node | lead_agent_node | JSON: `{"type": "status\|query\|message\|ack", "content": "..."}` |

### 5.5 Safety

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/safety/event` | `String` (JSON) | safety_monitor_node | lead_agent_node, wingman_agent_node |
| `/emergency_stop` | `String` | emergency_stop_node (keyboard) | emergency_stop_node (self-loop) |

### 5.6 GCS

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/voice_commands` | `String` | stt_node | lead_agent_node |
| `/clarification_request` | `String` | lead_agent_node | clarification_speaker_node |
| `/mission_status` | `String` | lead_agent_node | mission_monitor_node |

---

## 6. Python Module Reference

### 6.1 `common/schemas.py`

**Active schemas:**

| Schema | Key Fields | Used By |
|---|---|---|
| `FlightIntent` | `action`, `direction?`, `distance?`, `altitude?`, `confidence` | Tool registry → `/lead/approved_intent` |
| `WingmanOrder` | `order_id`, `mission_context`, `intent: FlightIntent`, `priority`, `confidence` | `wingman_agent_node._on_legacy_order` |
| `LeadOutput` | `my_intent`, `wingman_order?`, `confidence`, `situation_report` | Legacy parse helpers (not used in Part 6 loop) |
| `WingmanOutput` | `my_intent`, `confidence`, `situation_report` | Legacy |
| `SituationalAwareness` | `drone_id`, `battery_pct`, `altitude_m`, `position`, `flight_mode`, `gps_ok`, `camera_detections` | Internal sensor aggregator |
| `DronePosition` | `x`, `y`, `z`, `vx`, `vy`, `vz` | Within SituationalAwareness |
| `SafetyEvent` | `event_type`, `drone_id`, `severity`, `message`, `value` | safety_monitor → `/safety/event` |

**Removed schemas (not in codebase):** `MissionPlan`, `MissionStep`, `StepAssessment`, `WingmanProposal`

**`expand_compact_values(data)` — abbreviation decoder:**

Recursively traverses dicts and lists, expanding:

| Compact | Expanded | Field |
|---|---|---|
| `H` / `M` / `L` | `high` / `medium` / `low` | `confidence` |
| `N` / `S` / `E` / `W` / `NE` / `NW` / `SE` / `SW` | compass names | `direction` |
| `R` / `U` / `Em` | `routine` / `urgent` / `emergency` | `priority` |

Handles both nested dicts and lists (list support added in Part 6).

---

### 6.2 `common/ollama_client.py`

**`OllamaClient(host, port, model, num_ctx)`**

`infer(prompt: str, system: str) → tuple[str | None, float]`
- POSTs to `http://host:port/api/generate`
- `stream=False`, blocking, timeout 120s
- Returns `(response_text, latency_seconds)` or `(None, 0.0)` on any error

---

### 6.3 `common/confidence_gate.py`

**`ConfidenceGate(threshold=0.5)`**

`approve(output) → bool` — checks `output.confidence` against threshold. Used by legacy helpers; not called in the Part 6 agent loop (agents self-determine confidence via tool choice).

---

### 6.4 `common/tool_registry.py`

Full reference in [Section 8](#8-tool-registry-reference).

---

### 6.5 `common/context_manager.py`

Full reference in [Section 9](#9-context-window--memory-system).

---

### 6.6 `common/agent_memory.py`

**`AgentMemory(db_name: str)`**

SQLite DB at `~/.ros/<db_name>`.

| Method | Description |
|---|---|
| `remember(fact: str)` | INSERT with current timestamp |
| `recall(query: str, limit=6) → list[str]` | `LIKE %query%` keyword search, newest first |
| `get_recent(n=5) → list[str]` | Newest n facts, no filter |
| `clear()` | DELETE all rows (testing only) |

**DB files:**
- `~/.ros/lead_agent_memory.db`
- `~/.ros/wingman_agent_memory.db`

Facts accumulate indefinitely across node restarts. No automatic expiry.

---

## 7. Agent Loop — Deep Dive

### 7.1 Loop Lifecycle

```
Voice → _on_voice()
  └── _assign_goal(text)
        ctx.clear_history()
        ctx.set_goal(text)
        _mission_done = False
        Thread(_agent_loop, daemon=True).start()

_agent_loop():
  while rclpy.ok() and not _mission_done:
    1. prompt = ctx.build_prompt()
    2. tool_name, params = _infer_tool_call(prompt)
    3. if tool_name is None: sleep(pause); continue
    4. result = tools.execute(tool_name, params)
    5. ctx.add_tool_result(tool_name, params, result)
    6. _publish_status(f"{tool_name}: {result[:80]}")
    7. sleep(loop_pause_sec)

  if _mission_done:
    publish "[MISSION COMPLETE] {report}" to /clarification_request
    _agent_running = False
```

### 7.2 Inference with 3-Attempt Retry

```python
for attempt in range(3):
    full_prompt = prompt + error_ctx   # error_ctx empty on first try
    raw, latency = ollama.infer(full_prompt, system_prompt)
    
    try:
        raw_clean = raw[raw.find('{') : raw.rfind('}')+1]  # strip markdown
        data = json.loads(raw_clean)
        tool_name = data['tool']
        params    = data.get('params', {}) or {}
        
        if tools.is_valid(tool_name):
            return tool_name, params
        else:
            error_ctx = f"Unknown tool '{tool_name}'. Valid: {list(tools.tools.keys())}"
    except json.JSONDecodeError as e:
        error_ctx = f"JSON error: {e}. Output only valid JSON."

return None, {}   # all 3 attempts failed → skip cycle
```

Parse failure rate: ~5–15% on first attempt (Qwen2.5-Coder:3b), <2% after retries.

### 7.3 Context Prompt Structure

Built by `ctx.build_prompt()` each inference cycle:

```
[MISSION GOAL]
find a vehicle in the east sector

[CURRENT SITUATION]
bat:88% alt:10m mode:OFFBOARD armed:YES gps:OK(14sat)
pos:(0,59.7) vel:(0.0,0.0,0.0)
camera:car(82%) right far
obstacle:car:right:far
temporal:elapsed=65s dist_home=60m phase=searching

[MEMORY]
Earlier: get_situation()→bat:95%... | takeoff(alt=10)→Takeoff initiated | wait(s=15)→Waited 15s | move(dir=E,dist=60)→Moving east 60m

[MESSAGES FROM OTHER AGENT]
[WINGMAN] Starting south sector sweep.

[RECENT ACTIONS]
→ get_situation()
← bat:92% alt:10m mode:OFFBOARD gps:OK
→ wait(seconds=33)
← Waited 33s.

[NEXT ACTION] Output one tool call JSON:
```

### 7.4 Token Budget (per inference cycle)

| Section | Approx Tokens |
|---|---|
| System prompt | ~300 |
| MISSION GOAL | ~20 |
| CURRENT SITUATION | ~80 |
| MEMORY | ~100 |
| MESSAGES FROM OTHER AGENT | ~50 |
| RECENT ACTIONS (≤8 entries) | ~400 |
| NEXT ACTION anchor | ~10 |
| **Total input** | **~960** |
| Output (tool call JSON) | ~30–50 |
| **Grand total** | **~1010** |

Fits within `num_ctx=2048` with ~50% headroom.

---

## 8. Tool Registry Reference

### 8.1 Class Hierarchy

```
BaseToolRegistry
├── _register_base_tools()      16 tools: flight + sensing + memory + control
├── execute(name, params) → str
├── is_valid(name) → bool
└── schema_block() → str        compact one-liner per tool for system prompt

LeadToolRegistry(BaseToolRegistry)
└── _register_lead_tools()      adds: message_wingman, ask_human, notify_human

WingmanToolRegistry(BaseToolRegistry)
└── _register_wingman_tools()   adds: message_lead, ask_lead, notify_lead
```

### 8.2 Complete Tool Table

#### Base Tools (both agents)

| Tool | Params | Side Effect | Returns | Clamps |
|---|---|---|---|---|
| `takeoff` | `altitude: float` | Publishes FlightIntent `action=takeoff` | Confirmation + ETA | 1–30m |
| `move` | `direction: str, distance: float, altitude?: float` | Publishes FlightIntent `action=move` | ETA string | dist 1–100m |
| `hover` | — | Publishes FlightIntent `action=hover` | "Hovering" | — |
| `search` | `duration_sec: int` | Issues hover, polls camera for duration | Detections or "clear" | 5–60s |
| `land` | — | Publishes FlightIntent `action=land` | "Landing" | — |
| `rtl` | — | Publishes FlightIntent `action=rtl` | "RTL initiated" | — |
| `get_situation` | — | Reads `ros.own_situation` under lock | Full situation string | — |
| `scan_camera` | — | Reads `ros.camera_summary` + `ros.obstacle_vector` | Detection + obstacle text | — |
| `get_battery` | — | Reads `ros.battery_pct` (regex-parsed from situation) | Battery % string | — |
| `remember` | `fact: str` | `agent_memory.remember(fact)` → SQLite INSERT | "Remembered" | — |
| `recall` | `query: str` | `agent_memory.recall(query)` → LIKE query | Up to 5 matching facts | — |
| `wait` | `seconds: int` | `time.sleep(seconds)` (blocks daemon thread) | "Waited Xs" | 1–30s |
| `mission_complete` | `report: str` | Sets `ros._mission_done = True` | "MISSION COMPLETE: ..." | — |

#### Lead-Only Tools

| Tool | Params | Blocking? | Timeout | Notes |
|---|---|---|---|---|
| `message_wingman` | `message: str` | No | — | Publishes to `/agent/lead_to_wingman` |
| `ask_human` | `question: str` | Yes | 120s | Publishes to `/clarification_request`; blocks on `_human_event.wait(120)` |
| `notify_human` | `message: str` | No | — | Publishes `[LEAD] message` to `/clarification_request` |

#### Wingman-Only Tools

| Tool | Params | Blocking? | Timeout | Notes |
|---|---|---|---|---|
| `message_lead` | `message: str` | No | — | Publishes `{type:message,...}` to `/agent/wingman_to_lead` |
| `ask_lead` | `question: str` | Yes | 60s | Publishes `{type:query,...}`; blocks on `_lead_event.wait(60)` |
| `notify_lead` | `message: str` | No | — | Publishes `{type:status,...}` to `/agent/wingman_to_lead` |

### 8.3 How a Tool Gets Called

```
SLM output → {"tool": "move", "params": {"direction": "N", "distance": 50}}
  ↓
_infer_tool_call() parses → ("move", {"direction": "N", "distance": 50})
  ↓
tools.execute("move", {"direction": "N", "distance": 50})
  ↓
LeadToolRegistry.execute() → tools["move"].execute(params)
  ↓
BaseToolRegistry._move({"direction": "N", "distance": 50})
  → dir_map["N"] = "north"
  → _publish_intent({"action":"move","direction":"north","distance":50,"confidence":"high"})
  → pub_intent.publish(json.dumps(intent))
  → return "Moving north 50m. ETA ~28s. Call wait(28) then get_situation()."
  ↓
result injected into ctx.add_tool_result("move", params, result)
```

### 8.4 Adding a New Tool

1. Add method to appropriate registry class in `tool_registry.py`:
```python
def _zoom_camera(self, params: dict) -> str:
    level = float(params.get('level', 1.0))
    # call into self.ros to publish a zoom command
    return f"Camera zoomed to {level}x"
```

2. Register in `_register_*_tools()`:
```python
self.tools["zoom_camera"] = Tool(
    description="Zoom camera to level (1.0–4.0)",
    params={"level": "float: zoom factor 1.0–4.0"},
    execute=self._zoom_camera)
```

3. Add to system prompt (both the tool line AND an example if non-obvious).

4. If the tool needs a new ROS2 topic, add the publisher in `lead_agent_node.__init__()`.

---

## 9. Context Window & Memory System

### 9.1 ContextManager — Fields

| Field | Type | Set By | Used In Prompt |
|---|---|---|---|
| `goal` | `str` | `set_goal()` | `[MISSION GOAL]` |
| `situation` | `str` | `update_situation()` | `[CURRENT SITUATION]` |
| `memory_block` | `str` | `add_memory_note()`, `_compress_oldest()` | `[MEMORY]` |
| `inter_agent` | `list[str]` | `add_inter_agent_message()` | `[MESSAGES FROM OTHER AGENT]` (max 4) |
| `history` | `list[dict]` | `add_tool_result()` | `[RECENT ACTIONS]` (max 8 before compress) |

### 9.2 History Entry Format

```python
{
    "tool": "move",
    "params_str": "direction=N, distance=50",    # max 20 chars per value
    "result": "Moving north 50m. ETA ~28s..."    # capped at 120 chars
}
```

### 9.3 Compression — When and How

```
Constants:
  MAX_HISTORY    = 8    entries before compression
  COMPRESS_BATCH = 4    how many to compress per trigger

When len(history) > 8:
  batch  = history[:4]   (oldest 4)
  history = history[4:]  (keep newest 4)
  
  For each entry in batch:
    "move(direction=N, distance=50)→Moving north 50m..."   (40 char result cap)
  
  compressed = "Earlier: mv(...) → ... | wait(...) → ... | ..."
  
  memory_block = (compressed + "\n" + memory_block).strip()[:600]
```

After compression:
- `history` has 4 entries (newest)
- `memory_block` has the gist of older actions (capped at 600 chars)
- Total context stays bounded

### 9.4 AgentMemory — SQLite Schema

```sql
CREATE TABLE memory (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL NOT NULL,    -- Unix epoch, seconds
    fact      TEXT NOT NULL
);
```

`recall(query)` → `WHERE fact LIKE '%query%' ORDER BY timestamp DESC LIMIT 6`

Facts persist across restarts. No TTL. Use `mem.clear()` to wipe for a fresh mission trial.

---

## 10. Inter-Agent Communication Protocol

### 10.1 Lead → Wingman

**Topic:** `/agent/lead_to_wingman`
**Format:** Plain natural language string (no JSON schema)
**Examples:**
```
"Mission: survey east sector 60m. Search for vehicles. RTL after."
"Football found N50m. Cover the east sector now."
"Abort your current task. Return to base immediately."
"Answer: Yes, hover and observe the person."    ← reply to wingman's ask_lead
```

**Wingman receives it in `_on_lead_message()`:**
```python
if self._waiting_for_lead:
    self._lead_response = content
    self._lead_event.set()       # unblocks ask_lead tool
else:
    ctx.add_inter_agent_message("LEAD", content)
    _assign_goal(content)        # start/replace wingman mission
```

### 10.2 Wingman → Lead

**Topic:** `/agent/wingman_to_lead`
**Format:** JSON string with `type` and `content` fields:

| `type` | When used | Effect on Lead |
|---|---|---|
| `"status"` | Routine reports, mission complete | Injected into context inter-agent block |
| `"query"` | Wingman calls `ask_lead(question)` | Sets `_wingman_query_pending = True`; Lead sees note to respond |
| `"message"` | Non-blocking `message_lead` | Injected into context inter-agent block |
| `"ack"` | Confirming receipt of order | Injected into context |

**Lead receives it in `_on_wingman_message()`:**
```python
ctx.add_inter_agent_message("WINGMAN", content)
if msg_type == 'query':
    _wingman_query_pending = True
    # next loop cycle: agent sees "[NOTE] Wingman has a pending query..."
```

### 10.3 Timing

| Flow | Typical Latency | Blocking Agent? |
|---|---|---|
| `message_wingman` | ~5ms (DDS WiFi) | No |
| `notify_lead` | ~5ms | No |
| `ask_human` | 5–120s (human response time) | Yes — Lead loop pauses |
| `ask_lead` | 5–60s (Lead's inference + reply) | Yes — Wingman loop pauses |
| `_human_event.wait(120)` timeout | 120s | Yes → returns fallback message |
| `_lead_event.wait(60)` timeout | 60s | Yes → returns fallback message |

---

## 11. Safety Architecture

### 11.1 Two-Layer Model

```
LAYER 1: SLM Agent (soft, ~5s response)
  Agent calls rtl() when battery low via get_battery() or from safety_event
  Agent calls ask_human() when uncertain about safety
  Agent calls hover() before sensitive manoeuvres
  CAN be slow — not relied upon for safety-critical timing

LAYER 2: Safety Monitor (hard, <100ms response)
  No SLM. No Ollama. No agent loop.
  Direct PX4 VehicleCommand publishing.
  Agent CANNOT override safety monitor actions.
  Fires on: battery ≤ 15% OR GPS fix_type < 3
```

### 11.2 Safety Event → Agent Awareness Flow

```
1. safety_monitor detects battery(drone_0) = 14.2%  (≤ 15% threshold)
   ↓
2. Publishes /safety/event:
   {"event_type":"battery_rtl","drone_id":"drone_0","severity":"critical","message":"Battery 14.2% — RTL forced","value":14.2}
   ↓
3. Publishes VehicleCommand RTL × 3 to /fmu/in/vehicle_command
   PX4 executes RTL immediately
   ↓
4. lead_agent_node._on_safety_event() receives the event:
   ctx.add_memory_note("[SAFETY] Battery critical 14.2%. RTL forced.")
   ↓
5. Next agent inference cycle sees [SAFETY] note in [MEMORY]
   Agent typically calls: mission_complete("Aborted — battery RTL forced")
   (PX4 is already returning regardless of what agent does)
```

### 11.3 Emergency Stop

**Trigger:** Keyboard shortcut OR publish to `/emergency_stop` topic (any String message)

**Action:** Publishes both to Drone-0 and Drone-1:
- `VEHICLE_CMD_DO_FLIGHTMODE` → mode change
- `VEHICLE_CMD_COMPONENT_ARM_DISARM` (param1=0.0) → disarm

This is instant and bypasses all nodes including safety_monitor and both agent loops.

---

## 12. Configuration Reference

### 12.1 `config/lead_config.yaml` (complete final)

```yaml
lead_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0
    gps_min_satellites: 6

camera_detection_node:
  ros__parameters:
    model_path: "yolov8n.pt"
    confidence_threshold: 0.6
    publish_rate_hz: 2.0

safety_monitor_node:
  ros__parameters:
    battery_warn_pct: 20.0
    battery_rtl_pct: 15.0
    gps_min_fix_type: 3

lead_agent_node:
  ros__parameters:
    ollama_host: "localhost"
    ollama_port: 11434
    model: "qwen3.5:2b"
    num_ctx: 2048
    loop_pause_sec: 0.5

lead_px4_commander_node:
  ros__parameters:
    position_tolerance_m: 1.0
    control_rate_hz: 10.0
```

### 12.2 `config/wingman_config.yaml` (complete final)

```yaml
wingman_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0

wingman_agent_node:
  ros__parameters:
    ollama_host: "localhost"
    ollama_port: 11434
    model: "qwen3.5:2b"
    num_ctx: 1024
    loop_pause_sec: 0.5

wingman_px4_commander_node:
  ros__parameters:
    position_tolerance_m: 1.0
    control_rate_hz: 10.0
```

---

## 13. Launch File Reference

### 13.1 `launch/lead_pilot.launch.py` — Node list

Nodes launched on PC-1 (all simultaneous, no ordering dependency):

```
lead_sensor_aggregator_node    cfg: lead_config.yaml
camera_detection_node          cfg: lead_config.yaml
safety_monitor_node            cfg: lead_config.yaml
lead_agent_node                cfg: lead_config.yaml
lead_px4_commander_node        cfg: lead_config.yaml
lead_intent_bridge_node
stt_node
clarification_speaker_node
mission_monitor_node
emergency_stop_node
```

### 13.2 `launch/wingman_pilot.launch.py` — Node list

Nodes launched on PC-2:

```
wingman_sensor_aggregator_node   cfg: wingman_config.yaml
wingman_agent_node               cfg: wingman_config.yaml
wingman_px4_commander_node       cfg: wingman_config.yaml
```

### 13.3 Full Deployment Sequence

```
PC-1 Terminal 1:  cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_iris
PC-1 Terminal 2:  PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=iris ./build/.../bin/px4 -i 1 -d ...
PC-1 Terminal 3:  MicroXRCEAgent udp4 -p 8888
PC-1 Terminal 4:  source install/setup.bash && ros2 launch major_project lead_pilot.launch.py

PC-2 Terminal 1:  MicroXRCEAgent udp4 -p 8888
PC-2 Terminal 2:  source install/setup.bash && ros2 launch major_project wingman_pilot.launch.py
```

---

## 14. Data Flow Walkthrough

Voice command: **"Find any vehicles in the east sector"**

```
[Human speaks] → Whisper STT
  → /voice_commands: "find any vehicles in the east sector"
  → lead_agent_node._on_voice()
  → _waiting_for_human? No → _assign_goal("find any vehicles in the east sector")
  → ctx.set_goal(...) → Thread(_agent_loop).start()

Cycle 1 — get_situation
  SLM: {"tool":"get_situation","params":{}}
  execute: reads self.own_situation under lock
  result: "bat:95% alt:0m mode:MANUAL armed:NO gps:OK(14sat) pos:(0,0)..."
  ctx.add_tool_result(...)

Cycle 2 — takeoff
  SLM: {"tool":"takeoff","params":{"altitude":10}}
  execute: _publish_intent({"action":"takeoff","altitude":10,"confidence":"high"})
  → /lead/approved_intent published
  → lead_intent_bridge → lead_px4_commander
  → lead_px4_commander: target_z=-10, publishes OffboardControlMode+TrajectorySetpoint@10Hz
  → PX4 SITL: drone ascends in Gazebo

  [In parallel]
  camera_detection_node @ 2Hz:
    Gazebo camera → YOLOv8-nano → no detections → publishes empty
  lead_sensor_aggregator_node @ 1Hz:
    reads PX4 telemetry + camera → publishes /drone_0/situation
  lead_agent_node._on_situation():
    → self.own_situation = "bat:93% alt:7.2m..."
    → self.battery_pct = 93.0

Cycle 3 — wait
  SLM: {"tool":"wait","params":{"seconds":15}}
  execute: time.sleep(15) [daemon thread sleeps, ROS callbacks keep running]

Cycle 4 — get_situation (confirm altitude)
  result: "bat:91% alt:9.8m mode:OFFBOARD armed:YES..."

Cycle 5 — move east
  SLM: {"tool":"move","params":{"direction":"E","distance":60}}
  execute: _publish_intent({action:move, direction:east, distance:60})
  → commander: target_y += 60 → drone flies east

Cycle 6 — wait(33)

Cycle 7 — get_situation
  result: "bat:87% alt:10m pos:(0,59.7) camera:car(82%) right far obstacle:car:right:far..."
  [camera_detection_node detected car while drone was flying]

Cycle 8 — search(20)
  execute: hover published → drone holds
  polls camera for 20s, aggregates detections
  result: "Search complete (20s). Detected: car(82%) right far [car:right:far]"

Cycle 9 — remember
  SLM: {"tool":"remember","params":{"fact":"car detected E60m altitude 10m"}}
  execute: agent_memory.remember() → SQLite INSERT

Cycle 10 — notify_human
  SLM: {"tool":"notify_human","params":{"message":"Vehicle (car) found E60m. Returning."}}
  execute: pub_clarification.publish("[LEAD] Vehicle (car) found E60m. Returning.")
  → clarification_speaker_node: TTS speaks this aloud

Cycle 11 — rtl
  SLM: {"tool":"rtl","params":{}}
  execute: _publish_intent({action:rtl})
  → commander → VehicleCommand RTL → PX4 returns to launch

Cycle 12 — wait(40)

Cycle 13 — mission_complete
  SLM: {"tool":"mission_complete","params":{"report":"Car found E60m. RTL complete."}}
  execute: _mission_done = True → loop exits
  → pub_clarification.publish("[MISSION COMPLETE] Car found E60m. RTL complete.")
  → clarification_speaker_node speaks it
  → _agent_running = False → awaiting next voice command
```

---

## 15. How to Modify or Extend

### 15.1 Change SLM Model

Edit both config files:
```yaml
lead_agent_node:
  ros__parameters:
    model: "qwen3.5:7b"
    num_ctx: 4096
```
Pull first: `ollama pull qwen3.5:7b`. Larger models improve accuracy, increase latency.

---

### 15.2 Add a New Tool

1. Add execute method to the correct registry class in `tool_registry.py`
2. Register it in `_register_*_tools()`
3. Add a one-line entry to the system prompt (`lead_agent_system.txt` or `wingman_agent_system.txt`)
4. If the tool needs new ROS2 data: add subscriber to agent node, store on `self`, read in the tool via `self.ros`

---

### 15.3 Add a Third Drone (Scout)

1. PX4 SITL: `-i 2` → namespace `/px4_2/fmu/`, target_system=3
2. New package dir: `scout_pilot/` mirroring `wingman_pilot/`
3. New `ScoutAgentNode` — copy WingmanAgentNode, change namespace and db name
4. New topics: `/agent/lead_to_scout`, `/agent/scout_to_lead`
5. Add `message_scout(message)` tool to `LeadToolRegistry`
6. Add scout agent + commander to launch file and setup.py

---

### 15.4 Swap Camera Model

In `camera_detection_node.py`, change the `ultralytics` model load. Keep output format identical:
- `/camera_0/detections` → human text: `"label(pct%) dir dist"`
- `/camera_0/obstacle_vector` → machine tokens: `"label:dir:dist"`

Distance/direction helpers are independent of the model.

---

### 15.5 Change Safety Thresholds

Edit `lead_config.yaml` and restart `safety_monitor_node`. No code change.

```yaml
safety_monitor_node:
  ros__parameters:
    battery_warn_pct: 25.0    # earlier warning
    battery_rtl_pct: 18.0    # earlier RTL
```

---

### 15.6 Add a Safety Rule (e.g., geofence)

In `safety_monitor_node.py`:
1. Subscribe to `/fmu/out/vehicle_local_position`
2. Check x/y against boundary in callback
3. On breach: publish `/safety/event` JSON + VehicleCommand (land or RTL)
4. Add `_drone0_geofence_triggered` flag

---

### 15.7 Change Agent Behaviour Without Code

Edit the system prompt file:
- `lead_pilot/prompts/lead_agent_system.txt`
- `wingman_pilot/prompts/wingman_agent_system.txt`

Examples:
- Make agent always confirm before RTL: add rule `"Before calling rtl(), call notify_human() with reason first."`
- Higher default altitude: change example from `tk(10)` to `tk(15)`
- Longer searches: change example `sc(20)` to `sc(30)`
- Make wingman more autonomous: remove rule `"ask_lead before acting if uncertain"`

---

### 15.8 Enable Real Hardware

1. Connect PX4 flight controller via USB
2. Change MicroXRCE-DDS: `MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600`
3. All ROS2 topics remain identical — PX4 publishes same message types on SITL and real hardware
4. Camera: use `v4l2_camera` ROS2 node for USB camera → same `/camera/image_raw` topic
5. No changes needed to agent, commander, or safety nodes

---

### 15.9 Reduce Inference Latency

Quick wins in order of impact:

| Change | Estimated Speedup | Trade-off |
|---|---|---|
| Add GPU to Ollama host | ~10× | Cost |
| Use `qwen3.5:1.5b` | ~2× | Lower accuracy |
| Reduce `num_ctx` Lead: 2048→1536 | ~1.3× | Less history |
| Reduce `num_ctx` Wingman: 1024→768 | ~1.2× | Less history |
| Implement MiniSpec-like compact syntax | ~3.5× | Implementation effort, debug complexity |
| Remote Ollama on GPU PC | ~10× | Network latency added |

---

## Appendices

### A. Compact JSON Abbreviations

Used in FlightIntent and WingmanOrder, decoded by `expand_compact_values()`:

| Compact | Expanded | Field |
|---|---|---|
| `H` | `high` | confidence |
| `M` | `medium` | confidence |
| `L` | `low` | confidence |
| `N` | `north` | direction |
| `S` | `south` | direction |
| `E` | `east` | direction |
| `W` | `west` | direction |
| `NE` | `northeast` | direction |
| `NW` | `northwest` | direction |
| `SE` | `southeast` | direction |
| `SW` | `southwest` | direction |
| `R` | `routine` | priority |
| `U` | `urgent` | priority |
| `Em` | `emergency` | priority |

---

### B. PX4 VehicleCommand Reference

| Command | ID | Used By | Key Params |
|---|---|---|---|
| `VEHICLE_CMD_DO_SET_MODE` | 176 | commander (arm + OFFBOARD) | param1=1 (custom), param2=6 (OFFBOARD sub-mode) |
| `VEHICLE_CMD_COMPONENT_ARM_DISARM` | 400 | commander, emergency_stop | param1=1.0 arm / 0.0 disarm |
| `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` | 20 | commander, safety_monitor | no params |
| `VEHICLE_CMD_NAV_LAND` | 21 | commander | no params |

All commands: `from_external=True`, `target_system=1` (Drone-0) or `2` (Drone-1), `target_component=1`.
Safety monitor publishes RTL command × 3 (triple publish for reliability).

---

### C. Ollama API

```
POST http://<host>:11434/api/generate
Content-Type: application/json

{
  "model": "qwen3.5:2b",
  "prompt": "<full context prompt>",
  "system": "<system prompt>",
  "stream": false,
  "options": {"num_ctx": 2048}
}

Response:
{
  "model": "qwen3.5:2b",
  "response": "{\"tool\":\"move\",\"params\":{\"direction\":\"N\",\"distance\":50}}",
  "done": true,
  "total_duration": 4821000000,  // nanoseconds
  ...
}
```

To allow remote access: `OLLAMA_HOST=0.0.0.0 ollama serve` on the serving PC.
Point agent at remote Ollama: set `ollama_host: "10.34.211.86"` in config.

---

### D. Key Design Decisions and Rationale

| Decision | Rationale |
|---|---|
| One tool call per inference | Prevents hallucinated multi-step sequences; each step gets real feedback |
| 3-attempt retry with error context | Qwen2.5-Coder:3b fails JSON parse ~10% of time; retry with error message recovers most |
| Safety monitor independent of SLM | SLM latency (~5s) too slow for safety-critical decisions |
| SQLite for memory | Persists across node restarts; survives crashes; simple keyword recall |
| Context compression at 8 entries | Keeps prompt under 1000 tokens; preserves recent actions in full detail |
| Natural language Lead→Wingman | No schema parsing needed; SLM generates and understands natural language natively |
| JSON Wingman→Lead | Allows Lead to route on `type` field (query vs status) without LLM parsing |
| Wingman `num_ctx=1024` | Wingman context is simpler (no human comms, shorter goal); saves ~2s per inference |
| `battery_pct` parsed via regex | Sensor aggregator publishes situation as text; regex avoids a separate battery topic subscription in the agent |
| Altitude in `move` via optional param | Enables single-tool direction+altitude change; backward compatible (param omission = keep altitude) |
