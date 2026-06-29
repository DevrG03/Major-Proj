# Bulletproof 2-Drone Swarm — Implementation Plan & Checklist

> Each submodule is independently buildable and testable.
> Work in order M1 → M6. Do not start a module until its predecessor's checklist is ✅ complete.

---

## Architecture After All Fixes

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         PC-1 — LEAD (Drone-0)                            │
│                                                                          │
│  [M3] GazeboCameraNode ──► /camera_0/image_raw                           │
│       ↓ YOLOv8-nano                                                      │
│  /camera_0/detections ──► [M2] LeadSensorAggregator                      │
│  /px4_1/fmu/out/vehicle_local_position ──► LeadSensorAggregator          │
│       ↓ /drone_0/situation  (now includes Wingman pos)                   │
│                                                                          │
│  [M1] LeadAgentNode  ◄──────────────────────────────────────────────┐   │
│    ├─ Boot loop (no voice needed)                                    │   │
│    ├─ Abort signal on new goal                                       │   │
│    ├─ Non-blocking ask_human                                         │   │
│    ├─ SLM health monitor → RTL fallback                              │   │
│    └─ Publishes /lead/approved_intent                                │   │
│                                                                      │   │
│  [M4] SafetyMonitor (upgraded)                                       │   │
│    ├─ Battery RTL → BOTH drones                                      │   │
│    └─ /emergency_stop → BOTH drones (direct VehicleCommand)          │   │
│                                                                      │   │
│  LeadPX4Commander ── 10Hz keepalive ──► /fmu/in/trajectory_setpoint  │   │
│  [M5] follow_lead impl ──────────────────────────────────────────────┘   │
│                                                                          │
│  MicroXRCE-DDS Agent :8888 ──── DDS bridge ────────────────────────┐    │
└──────────────────────────────────────────────────────────────────────┼───┘
                                    WiFi CycloneDDS                   │
┌──────────────────────────────────────────────────────────────────────┼───┐
│                        PC-2 — WINGMAN (Drone-1)                      │   │
│                                                                      │   │
│  [M3] GazeboCameraNode ──► /camera_1/image_raw                       │   │
│       ↓ YOLOv8-nano                                                  │   │
│  /camera_1/detections ──► WingmanSensorAggregator                    │   │
│                                                                      │   │
│  [M1] WingmanAgentNode                                               │   │
│    ├─ Typed message envelope parser                                  │   │
│    ├─ Abort signal on new task                                       │   │
│    ├─ Non-blocking ask_lead                                          │   │
│    └─ Publishes /wingman/approved_intent                             │   │
│                                                                      │   │
│  [M5] WingmanPX4Commander                                            │   │
│    ├─ follow_lead (subscribes to Lead pos, maintains offset)         │   │
│    └─ /emergency_stop → direct VehicleCommand (no WiFi dependency)  ─┘   │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## Module Dependency Map

```
M1 (Agent Core) ← depends on → M2 (Inter-Agent Protocol)
M3 (Perception)   ← standalone, feeds M1 via sensor topics
M4 (Safety)       ← standalone, touches M1 context + M5 commanders
M5 (Commanders)   ← depends on M2 (Wingman pos topic)
M6 (Infra)        ← standalone, apply last
```

---

## M1: Agent Core Loop — Non-Blocking Real-Time Brain

**Fixes:** Loophole #1 (passive agent), #2 (sleep blocks), #3 (ask_human deadlock), #4 (race condition), #5 (silent stall)

**Files to modify:**
- `major_project/common/tool_registry.py`
- `major_project/lead_pilot/lead_agent_node.py`
- `major_project/wingman_pilot/wingman_agent_node.py`

### M1.1 — Non-Blocking Tool Execution (Fix #2)

Replace all `time.sleep()` inside tool execute functions with a **completion signal pattern**:
- Tools that take time (move, search, wait) publish an intent and return immediately with an ETA string
- The agent loop handles wait/monitoring by calling `get_situation()` on the next cycle
- Remove the `_search()` polling loop entirely — let the agent loop run `scan_camera()` each cycle during a "searching" phase

**Checklist:**
- [ ] `_wait(params)` → remove `time.sleep(secs)`, return `"Timer set: {secs}s. Call get_situation() after wait."`  and publish a hover intent so drone stays put
- [ ] `_search(params)` → remove the `while time.time() < end_time: time.sleep(2)` loop; instead publish hover intent and return `"Searching for {duration_sec}s. Call scan_camera() each cycle until timer done."` with a ROS2 timer expiry timestamp in the return string
- [ ] `_takeoff(params)` → no sleep needed (already returns immediately), verify this is clean
- [ ] `_move(params)` → no sleep needed (already returns ETA string), verify this is clean
- [ ] All other tools (`_land`, `_rtl`, `_hover`, `_remember`, `_recall`, `_get_situation`, `_scan_camera`, `_get_battery`, `_mission_complete`, `_message_wingman`, `_notify_human`) — verify none contain `time.sleep()`

**Verify:**
```bash
grep -n "time.sleep" ~/major_ws/src/major_project/major_project/common/tool_registry.py
# Must return 0 results
```

---

### M1.2 — Self-Starting Boot Loop (Fix #1)

The agent loop must start on node boot, before any voice command arrives.

**In `lead_agent_node.py` `__init__`:**
- Add `self._boot_goal = "STANDBY: Monitor situation and await mission goal from Ground Commander."`
- After all publishers/subscribers are created, call `self._assign_goal(self._boot_goal)`
- The agent starts immediately, calls `get_situation()` once, then calls `wait(10)` type behaviour in a low-frequency monitoring loop

**In `wingman_agent_node.py` `__init__`:**
- Add `self._boot_goal = "STANDBY: Await orders from Lead Pilot. Call get_situation() to monitor state."`
- Same pattern — agent starts immediately

**Checklist:**
- [ ] `lead_agent_node.__init__()` ends with `self._assign_goal(self._boot_goal)`
- [ ] `wingman_agent_node.__init__()` ends with `self._assign_goal(self._boot_goal)`
- [ ] `lead_agent_system.txt` adds rule: `"If goal is STANDBY: call get_situation() then wait(30), repeat."`
- [ ] `wingman_agent_system.txt` adds rule: `"If goal is STANDBY: call get_situation() then wait(30), repeat. Do NOT call mission_complete on STANDBY."`
- [ ] Verify agent starts logging within 5 seconds of node launch (before any voice command)

**Verify:**
```bash
ros2 topic echo /mission_status
# Must show output within 10s of launch with no voice command issued
```

---

### M1.3 — Abort Signal for Safe Goal Replacement (Fix #4)

Replace goal-overwrite with a clean thread abort → restart pattern.

**In both agent node files:**
```python
# Add to __init__:
self._abort_event = threading.Event()

# Modify _assign_goal():
def _assign_goal(self, goal: str):
    if self._agent_running:
        self._abort_event.set()   # signal current loop to stop
        # Wait max 2s for old thread to exit cleanly
        time.sleep(0.1)           # give thread one cycle to see abort

    self._abort_event.clear()
    self.ctx.clear_history()
    self.ctx.set_goal(goal)
    self._mission_done = False
    self._mission_report = ""
    self._agent_running = True
    thread = threading.Thread(target=self._agent_loop, daemon=True)
    thread.start()

# Modify _agent_loop() while condition:
while rclpy.ok() and not self._mission_done and not self._abort_event.is_set():
    ...
```

**Checklist:**
- [ ] `self._abort_event = threading.Event()` added to `__init__` in both agents
- [ ] `_assign_goal()` sets abort event before clearing context
- [ ] `_agent_loop()` while condition checks `self._abort_event.is_set()`
- [ ] Test: issue voice command while agent is mid-search → old loop exits, new loop starts cleanly
- [ ] Test: Wingman receives two rapid Lead messages → only the second goal executes

**Verify:**
```bash
# Issue first command, then 3s later issue second
ros2 topic pub --once /voice_commands std_msgs/msg/String '{data: "go north 100m"}'
sleep 3
ros2 topic pub --once /voice_commands std_msgs/msg/String '{data: "return home"}'
# Second command must take effect; no conflicting commands to drone
```

---

### M1.4 — Non-Blocking ask_human / ask_lead (Fix #3)

Replace blocking `_human_event.wait(120)` with a re-entry loop pattern.

**New pattern for `_ask_human`:**
```python
def _ask_human(self, params: dict) -> str:
    question = str(params.get('question', 'Please advise.'))
    # Publish question to GCS
    q_msg = String(); q_msg.data = question
    self.ros.pub_clarification.publish(q_msg)
    # Set pending flag — _on_voice will set _human_response when answered
    self.ros._human_response = None
    self.ros._waiting_for_human = True
    # Return immediately — agent loop will check next cycle
    return f"PENDING_HUMAN_RESPONSE: '{question[:60]}'. Call get_situation() while waiting."
```

**The agent loop must handle the `PENDING_HUMAN_RESPONSE` return:**
```python
result = self.tools.execute(tool_name, params)
if result.startswith("PENDING_HUMAN_RESPONSE"):
    # Add to context and continue loop normally — NOT blocking
    self.ctx.add_tool_result(tool_name, params, result)
    # On next cycles, check for response
    if self.ros._waiting_for_human and self.ros._human_response:
        human_answer = self.ros._human_response
        self.ros._waiting_for_human = False
        self.ros._human_response = None
        self.ctx.add_memory_note(f"[HUMAN ANSWERED] {human_answer}")
    continue
```

**Checklist:**
- [ ] `_ask_human()` in `LeadToolRegistry` — remove all `threading.Event().wait()` calls
- [ ] `_ask_lead()` in `WingmanToolRegistry` — remove all `threading.Event().wait()` calls
- [ ] Add `PENDING_HUMAN_RESPONSE` and `PENDING_LEAD_RESPONSE` sentinel string handling in both `_agent_loop()` methods
- [ ] `_on_voice()` sets `_human_response` but does NOT call `_assign_goal` when `_waiting_for_human` is True
- [ ] `_on_lead_message()` in Wingman: sets `_lead_response` when `_waiting_for_lead` is True
- [ ] Test: issue command → agent asks human → issue 5 more voice commands while waiting → only response is consumed; other commands are ignored until human responds or timeout
- [ ] Implement a timeout counter: after 24 cycles (~120s at 5s/cycle) if no human answer, inject `[HUMAN TIMEOUT] No response. Proceeding with best judgment.` into memory and clear `_waiting_for_human`

**Verify:**
```bash
# Start agent, trigger ask_human scenario via obstacle detection
ros2 topic pub --rate 2 /camera_0/obstacle_vector std_msgs/msg/String '{data: "person:ahead:very_close"}'
ros2 topic echo /clarification_request
# Must show question within 10s
ros2 topic echo /mission_status
# Must show agent continuing get_situation() calls, NOT frozen
```

---

### M1.5 — SLM Health Monitor with RTL Fallback (Fix #5)

Track consecutive inference failures. After threshold, trigger safe fallback.

**In both agent node files:**
```python
# Add to __init__:
self._consecutive_failures = 0
self.MAX_CONSECUTIVE_FAILURES = 5
self._slm_healthy = True

# In _agent_loop(), after _infer_tool_call():
if tool_name is None:
    self._consecutive_failures += 1
    if self._consecutive_failures >= self.MAX_CONSECUTIVE_FAILURES:
        if self._slm_healthy:
            self._slm_healthy = False
            self.get_logger().error("SLM health failure — triggering RTL fallback")
            self._publish_status("SLM_HEALTH_FAILURE: RTL initiated")
            # Notify human
            msg = String(); msg.data = "CRITICAL: SLM inference failed 5 consecutive times. Initiating RTL for safety."
            self.pub_clarification.publish(msg)
            # Issue RTL directly
            rtl_msg = String(); rtl_msg.data = '{"action":"rtl","confidence":"high"}'
            self.pub_intent.publish(rtl_msg)
        time.sleep(self.loop_pause)
        continue
    time.sleep(self.loop_pause)
    continue
else:
    self._consecutive_failures = 0
    self._slm_healthy = True
```

**Checklist:**
- [ ] `self._consecutive_failures = 0` in `__init__` for both agents
- [ ] `self.MAX_CONSECUTIVE_FAILURES = 5` (configurable via YAML parameter)
- [ ] RTL issued via `pub_intent` after threshold
- [ ] Human notification sent to `/clarification_request`
- [ ] `/agent/health` topic publishing `{"node": "lead"|"wingman", "slm_ok": bool, "consecutive_failures": N}` every 10s
- [ ] Add `slm_health_timeout_sec` to `lead_config.yaml` and `wingman_config.yaml`

**Verify:**
```bash
# Stop Ollama service
sudo systemctl stop ollama
ros2 topic echo /agent/health
# Must show slm_ok: false within 30s
ros2 topic echo /clarification_request
# Must show CRITICAL notification
ros2 topic echo /lead/approved_intent
# Must show RTL command
sudo systemctl start ollama
ros2 topic echo /agent/health
# Must show slm_ok: true within 60s of Ollama restart
```

---

## M2: Inter-Agent Protocol — Typed Message Envelopes

**Fixes:** Loophole #6 (no Wingman position), #7 (status treated as task), #8 (context compression)

**Files to modify:**
- `major_project/common/schemas.py`
- `major_project/common/context_manager.py`
- `major_project/common/tool_registry.py` (message_wingman, message_lead)
- `major_project/lead_pilot/lead_agent_node.py` (`_on_wingman_message`)
- `major_project/wingman_pilot/wingman_agent_node.py` (`_on_lead_message`)
- `major_project/lead_pilot/lead_sensor_aggregator_node.py`

### M2.1 — Typed Inter-Agent Message Envelope (Fix #7)

**New schema in `schemas.py`:**
```python
class AgentMessage(BaseModel):
    """All inter-agent messages must use this envelope."""
    type: Literal["task", "status", "reply", "abort", "query", "position"]
    sender: Literal["LEAD", "WINGMAN"]
    content: str
    order_id: Optional[str] = None

def make_agent_msg(type: str, sender: str, content: str, order_id: str = None) -> str:
    return AgentMessage(type=type, sender=sender, content=content, order_id=order_id).model_dump_json()
```

**Update `message_wingman` in `LeadToolRegistry`:**
```python
def _message_wingman(self, params):
    msg_type = params.get('msg_type', 'status')  # 'task', 'status', 'reply', 'query'
    content = str(params.get('message', ''))
    payload = make_agent_msg(type=msg_type, sender='LEAD', content=content)
    ...
```

**Update `_on_lead_message` in `wingman_agent_node`:**
```python
def _on_lead_message(self, msg: String):
    try:
        envelope = AgentMessage.model_validate_json(msg.data)
        msg_type = envelope.type
        content = envelope.content
    except Exception:
        # Legacy fallback: treat raw string as task
        msg_type = 'task'
        content = msg.data

    if self._waiting_for_lead and msg_type in ('reply', 'status'):
        self._lead_response = content
        self._lead_event.set()
        return

    if msg_type == 'task':
        self._assign_goal(content)
    elif msg_type == 'abort':
        self._abort_event.set()
    elif msg_type in ('status', 'position'):
        self.ctx.add_inter_agent_message("LEAD", f"[{msg_type.upper()}] {content}")
    elif msg_type == 'query':
        self.ctx.add_inter_agent_message("LEAD", f"[QUERY] {content}")
        # Will be answered in next inference cycle
```

**Checklist:**
- [ ] `AgentMessage` Pydantic model added to `schemas.py`
- [ ] `make_agent_msg()` helper function added to `schemas.py`
- [ ] `message_wingman` tool updated to accept `msg_type` parameter
- [ ] `message_lead` tool updated to include type in envelope
- [ ] `_on_lead_message()` parses envelope first, falls back gracefully
- [ ] `_on_wingman_message()` in Lead parses envelope, only triggers `_wingman_query_pending` for type `query`
- [ ] System prompts updated: `message_wingman(message:str, msg_type:str)` where `msg_type` is `task|status|reply`
- [ ] Test: Lead sends `status` message → Wingman does NOT abort current task
- [ ] Test: Lead sends `task` message → Wingman starts new agent loop

**Verify:**
```bash
# Manually inject a status message
ros2 topic pub --once /agent/lead_to_wingman std_msgs/msg/String \
  '{data: "{\"type\":\"status\",\"sender\":\"LEAD\",\"content\":\"I am searching north sector\"}"}'
ros2 topic echo /wingman/status_report_text
# Must NOT show a new mission start for Wingman
```

---

### M2.2 — Wingman Position in Lead's Situational Awareness (Fix #6)

**In `lead_sensor_aggregator_node.py`:**
```python
# Add subscription to Drone-1's position
self.sub_wingman_pos = self.create_subscription(
    VehicleLocalPosition,
    '/px4_1/fmu/out/vehicle_local_position',
    self.on_wingman_position, BEST_EFFORT_QOS)

self.wingman_pos = {'x': None, 'y': None, 'z': None}

def on_wingman_position(self, msg: VehicleLocalPosition):
    self.wingman_pos = {
        'x': round(msg.x, 1),
        'y': round(msg.y, 1),
        'z': round(msg.z, 1),
    }

# In publish_situation(), append wingman position:
if self.wingman_pos['x'] is not None:
    text += (f"\nwingman_pos:({self.wingman_pos['x']},"
             f"{self.wingman_pos['y']}) alt:{-self.wingman_pos['z']:.1f}m")
else:
    text += "\nwingman_pos:unknown"
```

**Add `get_wingman_situation()` tool to `LeadToolRegistry`:**
```python
"get_wingman_situation": Tool(
    description="Get Wingman's last known position and battery",
    params={},
    execute=self._get_wingman_situation)

def _get_wingman_situation(self, params: dict) -> str:
    with self.ros.lock:
        sit = self.ros.own_situation  # already contains wingman_pos line
    if "wingman_pos" in sit:
        # Extract just the wingman line
        for line in sit.split('\n'):
            if 'wingman_pos' in line:
                return line
    return "Wingman position: unknown"
```

**Checklist:**
- [ ] `/px4_1/fmu/out/vehicle_local_position` subscription added to `lead_sensor_aggregator_node`
- [ ] `wingman_pos` field appended to the `/drone_0/situation` text block
- [ ] `get_wingman_situation()` tool added to `LeadToolRegistry`
- [ ] `lead_agent_system.txt` mentions `get_wingman_situation()` tool
- [ ] Test: confirm `/drone_0/situation` contains `wingman_pos:(x,y)` when Drone-1 is flying
- [ ] Safety check: if wingman is within 5m of Lead, log a proximity warning to `/safety/event`

**Verify:**
```bash
ros2 topic echo /drone_0/situation
# Must include: wingman_pos:(x,y) alt:Nm
```

---

### M2.3 — Context Compression Preserves Critical Detections (Fix #8)

**In `context_manager.py`:**
```python
# Increase result retention in compression
for e in batch:
    r = e['result'][:100].replace('\n', ' ')  # was 40, now 100
    ...

# In add_tool_result(), add critical detection auto-flag:
CRITICAL_KEYWORDS = ('person', 'detected', 'obstacle', 'SAFETY', 'CRITICAL', 'battery')
if any(kw in result for kw in CRITICAL_KEYWORDS):
    # Inject into memory block immediately (not just compressed history)
    self.add_memory_note(f"[CRITICAL] {tool}→{result[:80]}")
```

**Checklist:**
- [ ] Compression result truncation increased from 40 to 100 chars in `_compress_oldest()`
- [ ] `CRITICAL_KEYWORDS` list defined in `context_manager.py`
- [ ] Critical results auto-injected to `memory_block` via `add_memory_note()`
- [ ] `MAX_HISTORY` increased from 8 to 12 (fits within 2048 tokens with adjusted budget)
- [ ] `COMPRESS_BATCH` increased from 4 to 6
- [ ] Token budget recalculated: with 12 history entries × ~60 tokens each = 720 + 300 system + 200 situation + 100 memory + 50 inter-agent + 50 output = ~1420 tokens (within 2048)
- [ ] Compression test: after 15 tool calls, verify memory_block contains original detection strings

**Verify:**
```python
ctx = ContextManager()
for i in range(15):
    ctx.add_tool_result("search", {"duration_sec": 20}, f"Detected: person(91%) ahead very_close bearing 045")
# Must retain person in memory_block after compression
assert "person" in ctx.memory_block
```

---

## M3: Perception Layer — Gazebo Camera Integration

**Fixes:** Loophole #9 (USB camera vs Gazebo topic), #10 (no Wingman camera)

**Files to modify:**
- `major_project/lead_pilot/camera_detection_node.py` — replace `cv2.VideoCapture` with ROS2 image subscription
- Add new `major_project/wingman_pilot/wingman_camera_detection_node.py` (copy + adapt)
- `launch/lead_pilot.launch.py` — update camera node source
- `launch/wingman_pilot.launch.py` — add camera node
- `config/lead_config.yaml` — update camera params
- `config/wingman_config.yaml` — add camera params

### M3.1 — Gazebo Camera Subscription for Lead (Fix #9)

**Replace the USB camera approach with a ROS2 image subscriber:**
```python
# New imports
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraDetectionNode(Node):
    def __init__(self):
        ...
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.bridge = CvBridge()
        
        image_topic = self.get_parameter('image_topic').value
        self.sub_image = self.create_subscription(
            Image, image_topic, self._on_image, 
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1))
        
        # Rate limiter: only process at publish_rate_hz
        self._last_detect_time = 0.0
        self._detect_period = 1.0 / rate_hz

    def _on_image(self, msg: Image):
        now = time.time()
        if now - self._last_detect_time < self._detect_period:
            return
        self._last_detect_time = now
        
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._process_frame(frame)
```

**Lead camera config (`lead_config.yaml`):**
```yaml
camera_detection_node:
  ros__parameters:
    image_topic: "/camera/image_raw"   # Gazebo Jetty camera topic for Drone-0
    model_path: "yolov8n.pt"
    confidence_threshold: 0.4
    publish_rate_hz: 2.0
    obstacle_labels: ["person", "car", "truck", "bicycle", "bird"]
```

**Checklist:**
- [ ] `cv_bridge` added to `package.xml` as a dependency: `<depend>cv_bridge</depend>`
- [ ] `cv_bridge` added to `package.xml`: `<depend>sensor_msgs</depend>`
- [ ] `cv2.VideoCapture` path removed from `_detect_loop()` (or gated behind `use_usb_camera` param)
- [ ] `_on_image()` callback added with rate limiting
- [ ] `_process_frame(frame)` extracted as a shared method usable by both paths
- [ ] Fallback: if `/camera/image_raw` receives no messages for 5s, publish "No camera signal from Gazebo"
- [ ] Install `cv_bridge` on both PCs: `sudo apt install ros-lyrical-cv-bridge`
- [ ] Confirm Gazebo publishes camera image: `ros2 topic hz /camera/image_raw` (should be ~30Hz)

**Verify:**
```bash
ros2 topic echo /camera_0/detections
# Must show YOLO results, not "Camera not available"
ros2 topic hz /camera_0/detections
# Must show ~2Hz
```

---

### M3.2 — Wingman Camera Node (Fix #10)

**Create new file** `wingman_pilot/wingman_camera_detection_node.py`:
- Same implementation as the updated `camera_detection_node.py`
- Publishes to `/camera_1/detections` and `/camera_1/obstacle_vector` (instead of `_0`)
- Subscribes to `/px4_1/camera/image_raw` (or the appropriate Drone-1 Gazebo camera topic)

**Wingman camera config (`wingman_config.yaml`):**
```yaml
wingman_camera_detection_node:
  ros__parameters:
    image_topic: "/px4_1/camera/image_raw"   # Drone-1's Gazebo camera topic
    model_path: "yolov8n.pt"
    confidence_threshold: 0.4
    publish_rate_hz: 2.0
    obstacle_labels: ["person", "car", "truck", "bicycle", "bird"]
```

**Add to `wingman_pilot.launch.py`:**
```python
Node(package='major_project',
     executable='wingman_camera_detection',
     name='wingman_camera_detection_node',
     output='screen',
     parameters=[cfg]),
```

**Add to `setup.py` console_scripts:**
```python
'wingman_camera_detection = major_project.wingman_pilot.wingman_camera_detection_node:main',
```

**Checklist:**
- [ ] `wingman_camera_detection_node.py` created in `wingman_pilot/`
- [ ] Publishes to `/camera_1/detections` and `/camera_1/obstacle_vector`
- [ ] Subscribes to Drone-1 Gazebo camera topic (verify topic name with `ros2 topic list | grep px4_1`)
- [ ] Entry point added to `setup.py`
- [ ] Added to `wingman_pilot.launch.py`
- [ ] `wingman_config.yaml` has `wingman_camera_detection_node` section
- [ ] Rebuild on PC-2: `colcon build --packages-select major_project`
- [ ] `WingmanSensorAggregator` already subscribes to `/camera_1/detections` — verify it works

**Verify:**
```bash
# On PC-2 after launch:
ros2 topic echo /camera_1/detections
# Must show YOLO detections for Drone-1's camera
ros2 topic echo /drone_1/situation
# Must contain camera: field with actual detections (not empty)
```

---

## M4: Safety Layer — Bulletproof Emergency Handling

**Fixes:** Loophole #12 (E-stop reliability for Drone-1)

**Files to modify:**
- `major_project/lead_pilot/safety_monitor_node.py`

### M4.1 — Emergency Stop via Safety Monitor (Fix #12)

The Safety Monitor already has RELIABLE_QOS publishers to both drone namespaces. Route `/emergency_stop` through it:

```python
class SafetyMonitorNode(Node):
    def __init__(self):
        ...
        # Add: subscribe to emergency stop
        from std_msgs.msg import Bool
        self.create_subscription(Bool, '/emergency_stop', self._on_emergency_stop, RELIABLE_QOS)

    def _on_emergency_stop(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().error("Emergency stop received — commanding land on BOTH drones")
        # Use RELIABLE_QOS publishers already set up for both drones
        self._send_vehicle_command(self._cmd_lead, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._send_vehicle_command(self._cmd_wingman, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        # Also publish safety event for agents to add to context
        self._publish_event("emergency_stop", "ALL", "critical",
                            "Emergency stop commanded — both drones landing")
```

**Checklist:**
- [ ] `/emergency_stop` subscription added to `SafetyMonitorNode` with RELIABLE_QOS
- [ ] `_on_emergency_stop()` method sends `VEHICLE_CMD_NAV_LAND` to BOTH drone publishers
- [ ] Safety event published to `/safety/event` so both agent contexts are updated
- [ ] Safety monitor is on PC-1 → it handles both drones reliably without WiFi dependency for Drone-1 land command (goes through DDS bridge which is already established)
- [ ] Test: trigger E-stop → verify both drones receive land command within 500ms

**Verify:**
```bash
# With both drones flying:
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 topic echo /safety/event
# Must show emergency_stop event for "ALL"
ros2 topic echo /fmu/in/vehicle_command
ros2 topic echo /px4_1/fmu/in/vehicle_command
# Must both show NAV_LAND within 1s
```

---

### M4.2 — Proximity Alert Between Drones

Add a proximity check using the Wingman position now available in Lead's sensor aggregator:

```python
# In lead_sensor_aggregator_node.py publish_situation():
if self.wingman_pos['x'] is not None:
    dx = self.pos['x'] - self.wingman_pos['x']
    dy = self.pos['y'] - self.wingman_pos['y']
    separation = math.sqrt(dx**2 + dy**2)
    if separation < 5.0:  # 5 metre minimum separation
        prox_event = {
            "event_type": "proximity_warning",
            "drone_id": "BOTH",
            "severity": "warning",
            "message": f"Drones within {separation:.1f}m of each other — collision risk",
            "value": separation
        }
        prox_msg = String()
        prox_msg.data = json.dumps(prox_event)
        self.pub_safety_event.publish(prox_msg)  # add safety event publisher
```

**Checklist:**
- [ ] `pub_safety_event` publisher added to `lead_sensor_aggregator_node`: `/safety/event`
- [ ] Separation calculation added using Wingman position from M2.2
- [ ] Proximity warning threshold configurable via YAML (`min_separation_m: 5.0`)
- [ ] Warning rate-limited: at most 1 warning per 5 seconds to avoid log flooding

---

## M5: Flight Execution — follow_lead & Commander Hardening

**Fixes:** Loophole #11 (follow_lead not implemented), #13 (unnecessary pause)

**Files to modify:**
- `major_project/wingman_pilot/wingman_px4_commander_node.py`
- `major_project/lead_pilot/lead_px4_commander_node.py` (loop_pause fix)
- `major_project/wingman_pilot/wingman_agent_node.py` (loop_pause fix)

### M5.1 — Implement follow_lead in Wingman Commander (Fix #11)

```python
class WingmanPX4CommanderNode(Node):
    def __init__(self):
        ...
        # follow_lead state
        self._follow_lead_active = False
        self._lead_pos = {'x': None, 'y': None, 'z': None}
        self._follow_offset_x = 5.0   # metres behind/beside Lead
        self._follow_offset_y = 5.0

        # Subscribe to Lead's position for follow_lead action
        self.sub_lead_pos = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',   # Lead's position (Drone-0)
            self._on_lead_position, BEST_EFFORT_QOS)

    def _on_lead_position(self, msg: VehicleLocalPosition):
        self._lead_pos = {'x': msg.x, 'y': msg.y, 'z': msg.z}
        if self._follow_lead_active and self._lead_pos['x'] is not None:
            # Update target to follow Lead with offset
            with self.lock:
                self.target_x = self._lead_pos['x'] - self._follow_offset_x
                self.target_y = self._lead_pos['y'] + self._follow_offset_y
                self.target_z = self._lead_pos['z']  # same altitude

    def on_intent(self, msg: String):
        ...
        elif action == 'follow_lead':
            offset = data.get('offset_m', 5.0)
            self._follow_offset_x = offset
            self._follow_offset_y = offset
            self._follow_lead_active = True
            self.get_logger().info(f"follow_lead activated with {offset}m offset")
        elif action in ('hover', 'hold', 'land', 'rtl', 'move', 'takeoff'):
            self._follow_lead_active = False   # Deactivate follow on any new command
            ...
```

**Checklist:**
- [ ] `sub_lead_pos` subscription added to Wingman commander
- [ ] `_on_lead_position()` updates target when `_follow_lead_active` is True
- [ ] `follow_lead` case in `on_intent()` activates follow mode
- [ ] All other actions deactivate `_follow_lead_active`
- [ ] `offset_m` parameter in FlightIntent for configurable formation distance
- [ ] Add `follow_lead` to Wingman system prompt examples
- [ ] Test: issue `follow_lead` → Wingman maintains 5m offset behind Lead as Lead moves

**Verify:**
```bash
# After both drones are airborne:
ros2 topic pub --once /wingman/approved_intent std_msgs/msg/String \
  '{data: "{\"action\":\"follow_lead\",\"offset_m\":5,\"confidence\":\"high\"}"}'
ros2 topic echo /px4_1/fmu/in/trajectory_setpoint
# Setpoint must update every 100ms tracking Lead position + offset
```

---

### M5.2 — Remove Unnecessary Loop Pause (Fix #13)

**In both agent nodes:**
```python
# REMOVE this from _agent_loop():
if not self._mission_done:
    time.sleep(self.loop_pause)   # ← DELETE

# Instead, only pause on instantaneous tools to avoid SLM hammering:
FAST_TOOLS = {'notify_human', 'notify_lead', 'message_wingman', 'message_lead', 'hover', 'remember'}
if tool_name in FAST_TOOLS and not self._mission_done:
    time.sleep(0.2)  # brief 200ms pause only for instant-return tools
```

**Checklist:**
- [ ] Unconditional `time.sleep(self.loop_pause)` removed from both agent loops
- [ ] 200ms pause added only for `FAST_TOOLS` set
- [ ] `loop_pause_sec` parameter removed from both config YAML files
- [ ] Mission timing test: 10-step mission completes at least 25% faster

---

## M6: Infrastructure & Quality Fixes

**Fixes:** Loophole #14 (SQLite connections), #15 (silent Wingman camera absence)

**Files to modify:**
- `major_project/common/agent_memory.py`
- Both config YAML files

### M6.1 — SQLite Connection Pooling (Fix #14)

```python
class AgentMemory:
    def __init__(self, db_name: str = "lead_agent_memory.db"):
        ...
        # Persistent connection with WAL mode
        self.conn = sqlite3.connect(
            self.db_path, check_same_thread=False, timeout=10.0)
        self.conn.execute("PRAGMA journal_mode=WAL")
        self.conn.execute("PRAGMA synchronous=NORMAL")
        self._init_db()

    def _init_db(self):
        with self._lock:
            self.conn.execute("""
                CREATE TABLE IF NOT EXISTS memory (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    fact TEXT NOT NULL
                )""")
            self.conn.commit()

    def remember(self, fact: str):
        with self._lock:
            self.conn.execute(
                "INSERT INTO memory (timestamp, fact) VALUES (?, ?)",
                (time.time(), fact.strip()))
            self.conn.commit()

    def recall(self, query: str = "", limit: int = 6) -> list[str]:
        with self._lock:
            if query:
                rows = self.conn.execute(
                    "SELECT fact FROM memory WHERE fact LIKE ? ORDER BY timestamp DESC LIMIT ?",
                    (f"%{query}%", limit)).fetchall()
            else:
                rows = self.conn.execute(
                    "SELECT fact FROM memory ORDER BY timestamp DESC LIMIT ?",
                    (limit,)).fetchall()
        return [r[0] for r in rows]

    def __del__(self):
        if hasattr(self, 'conn'):
            self.conn.close()
```

**Checklist:**
- [ ] `self.conn` persistent connection in `__init__`
- [ ] `PRAGMA journal_mode=WAL` enables concurrent reads
- [ ] All methods use `self.conn` instead of opening new connections
- [ ] `__del__` closes connection on node shutdown
- [ ] Thread safety maintained via `self._lock` (already exists)

---

### M6.2 — Startup Diagnostics Node

**Create** `major_project/gcs/diagnostics_node.py`:
- Checks all expected topics are publishing within expected rate thresholds
- Publishes a `/system/health` topic with per-subsystem OK/WARN/ERROR status
- Prints a startup readiness table to terminal

**Checklist:**
- [ ] `diagnostics_node.py` created in `gcs/`
- [ ] Checks: `/drone_0/situation` (≥0.9 Hz), `/drone_1/situation` (≥0.9 Hz), `/camera_0/detections` (≥1.5 Hz), `/camera_1/detections` (≥1.5 Hz), `/agent/health` (≥0.05 Hz), `/mission_status` (publishing)
- [ ] Startup: waits 15s then prints readiness table
- [ ] Ongoing: republishes readiness every 30s
- [ ] Entry point added to `setup.py`
- [ ] Added to `lead_pilot.launch.py`

---

## Master Checklist — All Modules

### M1 — Agent Core Loop
- [ ] M1.1 All `time.sleep()` removed from tool registry
- [ ] M1.2 Agents self-start on node boot
- [ ] M1.3 Abort event added, race condition eliminated
- [ ] M1.4 `ask_human` / `ask_lead` non-blocking
- [ ] M1.5 SLM health monitor with RTL fallback

### M2 — Inter-Agent Protocol
- [ ] M2.1 Typed message envelopes (`AgentMessage` schema)
- [ ] M2.2 Wingman position in Lead's situation block + `get_wingman_situation()` tool
- [ ] M2.3 Context compression retains 100 chars, critical detections auto-flagged

### M3 — Perception Layer
- [ ] M3.1 Lead camera uses Gazebo `/camera/image_raw` ROS2 topic via `cv_bridge`
- [ ] M3.2 Wingman camera node created and wired to Drone-1 Gazebo topic

### M4 — Safety Layer
- [ ] M4.1 Emergency stop routes through Safety Monitor to BOTH drones
- [ ] M4.2 Proximity alert (5m separation warning) added

### M5 — Flight Execution
- [ ] M5.1 `follow_lead` fully implemented in Wingman commander
- [ ] M5.2 Unnecessary loop pause removed

### M6 — Infrastructure
- [ ] M6.1 SQLite persistent connection + WAL mode
- [ ] M6.2 Startup diagnostics node

---

## Integration Test Suite

After all modules complete, run these end-to-end tests in order:

### Test 1 — Boot Autonomy (validates M1.2)
```bash
ros2 launch major_project lead_pilot.launch.py
# Wait 10s — NO voice command issued
ros2 topic echo /mission_status
# ✅ PASS: Agent shows get_situation() results without voice input
```

### Test 2 — Goal Preemption (validates M1.3)
```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String '{data: "fly north 200m"}'
sleep 3
ros2 topic pub --once /voice_commands std_msgs/msg/String '{data: "return home"}'
ros2 topic echo /lead/approved_intent
# ✅ PASS: RTL command appears within 2s of second voice input
# ✅ PASS: No conflicting move+rtl setpoints
```

### Test 3 — Non-Blocking Human Escalation (validates M1.4)
```bash
# Simulate obstacle during flight
ros2 topic pub --rate 3 /camera_0/obstacle_vector std_msgs/msg/String '{data: "person:ahead:very_close"}'
ros2 topic echo /clarification_request   # Question published
ros2 topic echo /mission_status          # Agent keeps running (get_situation cycles)
# ✅ PASS: Agent publishes status while waiting for human (not frozen)
```

### Test 4 — SLM Health Fallback (validates M1.5)
```bash
sudo systemctl stop ollama
sleep 60  # wait for 5 failure threshold
ros2 topic echo /clarification_request
# ✅ PASS: "CRITICAL: SLM inference failed" message appears
ros2 topic echo /lead/approved_intent
# ✅ PASS: RTL command published
sudo systemctl start ollama
```

### Test 5 — Camera Vision (validates M3)
```bash
ros2 topic echo /camera_0/detections  # Lead camera
ros2 topic echo /camera_1/detections  # Wingman camera
# ✅ PASS: Both show YOLO results (not "Camera not available")
```

### Test 6 — Full Mission with 2-Drone Coordination (validates all modules)
```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String \
  '{data: "split search: you cover north, send wingman south, meet in 2 minutes"}'
ros2 topic echo /agent/lead_to_wingman   # Watch typed envelopes
ros2 topic echo /drone_0/situation       # Watch wingman_pos field update
ros2 topic echo /mission_status          # Both agents report progress
# ✅ PASS: Lead coordinates mission, Wingman executes separately, both RTL
```

### Test 7 — Emergency Stop (validates M4)
```bash
# With both drones flying
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
# Within 1 second:
ros2 topic echo /fmu/in/vehicle_command        # Drone-0 LAND
ros2 topic echo /px4_1/fmu/in/vehicle_command  # Drone-1 LAND
# ✅ PASS: Both drones land within 2s of E-stop
```

---

## Build Order

```bash
# Step 1: Apply all code changes
# Step 2: Install new dependencies
sudo apt install ros-lyrical-cv-bridge
pip install 'empy==3.3.4'  # verify empy hasn't been upgraded

# Step 3: Build on PC-1
cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash

# Step 4: Smoke test all imports
python3 -c "
from major_project.common.tool_registry import LeadToolRegistry, WingmanToolRegistry
from major_project.common.context_manager import ContextManager
from major_project.common.agent_memory import AgentMemory
from major_project.common.schemas import AgentMessage, make_agent_msg
print('All imports OK')
"

# Step 5: Sync to PC-2
rsync -av --progress ~/major_ws/src/major_project/ dev@<PC2_IP>:~/major_ws/src/major_project/

# Step 6: Build on PC-2
colcon build --packages-select major_project --symlink-install
source install/setup.bash

# Step 7: Run integration tests 1-7 above
```
