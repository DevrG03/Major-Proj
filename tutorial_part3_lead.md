# Multi-Drone SLM Pilot System — Tutorial Part 3
## Lead Pilot Stack (Sensor Aggregator, NLU Node, PX4 Commander)

> Continues from Part 2. Common package is built and tested.

---

## PART 10: Lead Sensor Aggregator

The sensor aggregator subscribes to raw PX4 telemetry and camera detections, formats them into a human-readable situational awareness text block, and publishes it every second. The Lead NLU injects this block into every SLM prompt.

### 10.1 Write lead_sensor_aggregator_node.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_sensor_aggregator_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus
import json
import math


BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)


class LeadSensorAggregatorNode(Node):
    def __init__(self):
        super().__init__('lead_sensor_aggregator_node')

        # PX4 Drone-0 telemetry (no namespace prefix)
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

        # Camera detection summary (from camera_detection_node)
        self.sub_camera = self.create_subscription(
            String, '/camera_0/detections', self.on_camera, 10)

        # Output: situation text block
        self.pub_situation = self.create_publisher(String, '/drone_0/situation', 10)

        # State
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                    'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
                    'heading': 0.0, 'alt_baro': 0.0}
        self.battery_pct = 0.0
        self.flight_mode = "UNKNOWN"
        self.arming_state = "DISARMED"
        self.gps_fix = False
        self.camera_summary = "No camera data"

        # Publish SA block every 1 second
        self.timer = self.create_timer(1.0, self.publish_situation)
        self.get_logger().info("Lead sensor aggregator started (Drone-0)")

    def on_position(self, msg: VehicleLocalPosition):
        speed = math.sqrt(msg.vx**2 + msg.vy**2)
        heading = math.degrees(math.atan2(msg.vy, msg.vx)) % 360
        self.pos = {
            'x': round(msg.x, 1),
            'y': round(msg.y, 1),
            'z': round(msg.z, 1),     # NED: negative z = up
            'vx': round(msg.vx, 1),
            'vy': round(msg.vy, 1),
            'alt_baro': round(-msg.z, 1),  # positive altitude
            'speed': round(speed, 1),
            'heading': round(heading, 0),
        }
        self.gps_fix = msg.xy_global

    def on_status(self, msg: VehicleStatus):
        # arming_state enum in PX4: 1=STANDBY (disarmed), 2=ARMED.
        # Verify against your px4_msgs build:
        #   ros2 interface show px4_msgs/msg/VehicleStatus | grep arming_state
        # If your version uses 1=ARMED, swap the condition below.
        self.arming_state = "ARMED" if msg.arming_state == 2 else "DISARMED"
        # nav_state maps to flight mode string
        nav_map = {
            14: "OFFBOARD", 2: "POSITION", 1: "MANUAL",
            3: "ALTITUDE", 17: "AUTO_TAKEOFF", 4: "ACRO",
            0: "MANUAL", 12: "LOITER", 5: "STABILIZED"
        }
        self.flight_mode = nav_map.get(msg.nav_state, f"MODE_{msg.nav_state}")

    def on_battery(self, msg: BatteryStatus):
        # BatteryStatus.remaining is a 0.0–1.0 fraction in PX4 (not 0–100).
        # Multiply by 100 to get percentage. Verify in your px4_msgs version:
        #   ros2 interface show px4_msgs/msg/BatteryStatus | grep remaining
        self.battery_pct = round(msg.remaining * 100.0, 1)

    def on_camera(self, msg: String):
        self.camera_summary = msg.data

    def publish_situation(self):
        alt = self.pos.get('alt_baro', 0.0)
        speed = self.pos.get('speed', 0.0)
        heading = self.pos.get('heading', 0.0)

        text = (
            f"pos:({self.pos.get('x',0):.1f},{self.pos.get('y',0):.1f},"
            f"{self.pos.get('z',0):.1f}m) "
            f"alt:{alt:.1f}m hdg:{heading:.0f}° spd:{speed:.1f}m/s "
            f"bat:{self.battery_pct:.0f}% "
            f"mode:{self.flight_mode} {self.arming_state} "
            f"gps:{'OK' if self.gps_fix else 'NO'}\n"
            f"camera:{self.camera_summary}"
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
EOF
```

---

## PART 11: Lead NLU Node

This is the most complex node. It:
1. Subscribes to voice commands from the human
2. Subscribes to situational awareness from both drones
3. Subscribes to wingman status reports
4. Builds a rich context prompt and calls Ollama
5. Validates output through normaliser + Pydantic
6. Routes through the confidence gate
7. Publishes to approved_intent, wingman/order, or clarification_request

### 11.1 Write the Lead System Prompt

```bash
# [PC-1] Create prompts directory and write lead system prompt
mkdir -p ~/major_ws/src/major_project/major_project/lead_pilot/prompts

cat << 'PROMPT_EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/prompts/lead_system.txt
You are LEAD PILOT of a 2-drone formation operating under a human Ground Commander.
Drone-0 is YOUR drone. Drone-1 is your WINGMAN.

YOUR RESPONSIBILITIES:
1. Execute your own Drone-0 flight based on Ground Commander orders
2. Issue orders to your Wingman (Drone-1) when needed
3. Maintain situational awareness of both drones
4. Report mission status in plain language for the GCS display

CONFIDENCE RULES (apply to BOTH your intent and wingman orders):
- "high"   : Command is clear, unambiguous, all parameters known or safely defaultable
- "medium" : Command is parseable but one parameter was assumed or phrasing was vague
- "low"    : Command is ambiguous, incomplete, dangerous, or outside drone capabilities

EXECUTION POLICY:
- high   → execute immediately, no clarification needed
- medium → execute but explain your assumption in situation_report
- low    → DO NOT execute, set clarification_question to ask the Ground Commander

WINGMAN ORDERS:
- Only issue a wingman_order when the Ground Commander explicitly or implicitly directs Drone-1
- A wingman_order must include: mission_context (WHY), intent (WHAT), priority (HOW URGENT)
- If wingman task is also ambiguous, set wingman_order confidence to "low" with clarification_question

SCHEMA:
Output ONLY valid JSON matching this exact structure:
{
  "my_intent": {
    "action": "<takeoff|move|hover|land|rtl|search|search_stop|hold>",
    "altitude": <number 0.5-50 or null>,
    "distance": <number 0.1-100 or null>,
    "direction": "<north|south|east|west|northeast|northwest|southeast|southwest|forward|backward|left|right or null>",
    "speed": <number 0.1-10 or null>,
    "confidence": "<high|medium|low>",
    "clarification_question": "<string or null>"
  },
  "wingman_order": {
    "order_id": "<unique short id like W001>",
    "mission_context": "<1 sentence explaining why>",
    "intent": { <same FlightIntent schema as my_intent> },
    "priority": "<routine|urgent|emergency>",
    "confidence": "<high|medium|low>",
    "clarification_question": "<string or null>"
  },
  "confidence": "<high|medium|low — your overall confidence in this response>",
  "situation_report": "<1-2 sentence NL summary for GCS display>",
  "clarification_question": "<question for Ground Commander, or null>"
}

Set my_intent to null if no action needed for Drone-0.
Set wingman_order to null if no Wingman action needed.
PROMPT_EOF
```

### 11.2 Write lead_nlu_node.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_nlu_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import uuid
import os
import threading
import time

# Import from common (adjust path if needed)
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from major_project.common.schemas import (
    FlightIntent, WingmanOrder, DronePosition,
    LeadOutput, parse_lead_output
)
from major_project.common.normaliser import normalise_parsed
from major_project.common.confidence_gate import gate_lead, LeadAction
from major_project.common.ollama_client import OllamaClient


def load_system_prompt() -> str:
    prompt_path = os.path.join(
        os.path.dirname(__file__), 'prompts', 'lead_system.txt')
    with open(prompt_path) as f:
        return f.read()


class LeadNLUNode(Node):
    def __init__(self):
        super().__init__('lead_nlu_node')

        # Parameters
        self.declare_parameter('ollama_host', 'localhost')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('model', 'qwen2.5-coder:3b')
        self.declare_parameter('num_ctx', 2048)
        self.declare_parameter('async_mode', False)

        host = self.get_parameter('ollama_host').value
        port = self.get_parameter('ollama_port').value
        model = self.get_parameter('model').value
        num_ctx = self.get_parameter('num_ctx').value
        self.async_mode = self.get_parameter('async_mode').value

        self.ollama = OllamaClient(host=host, port=port, model=model,
                                    num_ctx=num_ctx)
        self.system_prompt = load_system_prompt()

        # State
        self.drone0_situation = "No data yet"
        self.drone1_situation = "No data yet"
        self.last_wingman_report = "No report yet"
        self.order_counter = 0
        self.lock = threading.Lock()

        # Async mode: background inference thread
        self._pending_command = None
        self._inference_running = False
        if self.async_mode:
            self.get_logger().info("Running in ASYNC mode (non-blocking inference)")
            self._inference_thread = threading.Thread(
                target=self._inference_loop, daemon=True)
            self._inference_thread.start()

        # Subscriptions
        self.sub_voice = self.create_subscription(
            String, '/voice_commands', self.on_voice, 10)
        self.sub_d0 = self.create_subscription(
            String, '/drone_0/situation', self.on_drone0_situation, 10)
        self.sub_d1 = self.create_subscription(
            String, '/drone_1/situation', self.on_drone1_situation, 10)
        self.sub_wingman = self.create_subscription(
            String, '/wingman/status_report_text',
            self.on_wingman_report, 10)
        self.sub_wingman_clarify = self.create_subscription(
            String, '/wingman/clarification_to_lead',
            self.on_wingman_clarification, 10)

        # Publishers
        self.pub_lead_intent = self.create_publisher(
            String, '/lead/approved_intent', 10)
        self.pub_wingman_order = self.create_publisher(
            String, '/wingman/order', 10)
        self.pub_clarification = self.create_publisher(
            String, '/clarification_request', 10)
        self.pub_mission_status = self.create_publisher(
            String, '/mission_status', 10)

        self.get_logger().info(f"Lead NLU ready (Ollama: {host}:{port}, model: {model})")

    # ─── Subscription callbacks ───────────────────────────────────

    def on_drone0_situation(self, msg: String):
        with self.lock:
            self.drone0_situation = msg.data

    def on_drone1_situation(self, msg: String):
        with self.lock:
            self.drone1_situation = msg.data

    def on_wingman_report(self, msg: String):
        with self.lock:
            self.last_wingman_report = msg.data

    def on_wingman_clarification(self, msg: String):
        """Wingman couldn't understand its order — escalate to human."""
        question = msg.data
        self.get_logger().warning(f"Wingman needs clarification: {question}")
        # Escalate to ground commander
        clarify_msg = String()
        clarify_msg.data = f"[WINGMAN asks]: {question}"
        self.pub_clarification.publish(clarify_msg)

    def on_voice(self, msg: String):
        """Called when STT publishes a new voice command."""
        command = msg.data.strip()
        if not command:
            return
        self.get_logger().info(f"Voice command received: '{command}'")

        if self.async_mode:
            with self.lock:
                self._pending_command = command
        else:
            self._process_command(command)

    # ─── Async inference loop (used when latency > 400ms) ────────

    def _inference_loop(self):
        while rclpy.ok():
            command = None
            with self.lock:
                if self._pending_command:
                    command = self._pending_command
                    self._pending_command = None
            if command:
                self._process_command(command)
            else:
                time.sleep(0.05)

    # ─── Core inference ──────────────────────────────────────────

    def _build_context_prompt(self, command: str) -> str:
        with self.lock:
            d0 = self.drone0_situation
            d1 = self.drone1_situation
            wingman_rpt = self.last_wingman_report

        return (
            f"[SITUATIONAL AWARENESS]\n"
            f"[DRONE-0 | LEAD]    {d0}\n"
            f"[DRONE-1 | WINGMAN] {d1}\n"
            f"[WINGMAN REPORT]    {wingman_rpt}\n\n"
            f"[GROUND COMMANDER SAYS]\n"
            f"{command}"
        )

    def _process_command(self, command: str):
        prompt = self._build_context_prompt(command)
        t0 = time.perf_counter()
        raw_json, latency = self.ollama.infer(prompt, self.system_prompt)
        self.get_logger().debug(f"Ollama latency: {latency*1000:.0f}ms")

        if raw_json is None:
            self.get_logger().error("Ollama inference failed — publishing clarification")
            msg = String()
            msg.data = "System error: could not parse command. Please repeat."
            self.pub_clarification.publish(msg)
            return

        # Parse and validate
        try:
            data = json.loads(raw_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode failed: {e} | raw: {raw_json[:100]}")
            return

        # Validate overall structure
        lead_output = parse_lead_output(raw_json)
        if lead_output is None:
            self.get_logger().warning("LeadOutput schema validation failed")
            # Try to extract just the confidence and clarification_question
            confidence = data.get('confidence', 'low')
            clarify_q = data.get('clarification_question')
            if clarify_q:
                msg = String()
                msg.data = clarify_q
                self.pub_clarification.publish(msg)
            return

        # Publish situation report to GCS
        status_msg = String()
        status_msg.data = json.dumps({
            "lead": lead_output.situation_report,
            "wingman": self.last_wingman_report
        })
        self.pub_mission_status.publish(status_msg)

        # Apply confidence gate
        gate_result = gate_lead(lead_output.confidence)
        self.get_logger().info(
            f"Confidence: {lead_output.confidence} → gate: {gate_result.value}")

        if gate_result == LeadAction.WITHHOLD_CLARIFY_HUMAN:
            # Do not execute anything — ask human
            if lead_output.clarification_question:
                msg = String()
                msg.data = lead_output.clarification_question
                self.pub_clarification.publish(msg)
            return

        # Execute my_intent for Drone-0
        if lead_output.my_intent is not None:
            intent_msg = String()
            intent_msg.data = lead_output.my_intent.model_dump_json(exclude_none=True)
            self.pub_lead_intent.publish(intent_msg)
            self.get_logger().info(
                f"Lead intent: {lead_output.my_intent.action} "
                f"[{lead_output.my_intent.confidence}]")

            if gate_result == LeadAction.EXECUTE_WITH_WARNING:
                warn_msg = String()
                warn_msg.data = (
                    f"[LEAD WARNING] Executing with assumption: "
                    f"{lead_output.my_intent.clarification_question or 'parameters assumed'}")
                self.pub_clarification.publish(warn_msg)

        # Issue wingman order for Drone-1
        if lead_output.wingman_order is not None:
            wo = lead_output.wingman_order
            # Assign order ID if not set
            if not wo.order_id:
                self.order_counter += 1
                wo.order_id = f"W{self.order_counter:03d}"

            order_msg = String()
            order_msg.data = wo.model_dump_json(exclude_none=True)
            self.pub_wingman_order.publish(order_msg)
            self.get_logger().info(
                f"Wingman order issued: {wo.order_id} "
                f"action={wo.intent.action} [{wo.confidence}]")


def main(args=None):
    rclpy.init(args=args)
    node = LeadNLUNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

---

## PART 12: Lead PX4 Commander Node

Translates `FlightIntent` JSON into PX4 trajectory setpoints and vehicle commands.

### 12.1 Write lead_px4_commander_node.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_px4_commander_node.py
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

        # Subscribe to approved intents from Lead NLU
        self.sub_intent = self.create_subscription(
            String, '/lead/approved_intent', self.on_intent, 10)

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
            with self.lock:
                self.target_x = x + dx * distance
                self.target_y = y + dy * distance
                self.target_z = z  # keep current altitude
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
            # For search: move in expanding pattern
            # Simple implementation: move to a nearby area
            dx, dy = self.DIRECTION_OFFSETS.get(direction, (1.0, 0.0))
            with self.lock:
                self.target_x = x + dx * distance
                self.target_y = y + dy * distance
                self.target_z = z
            self.get_logger().info(f"Search toward {direction}")

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
EOF
```

---

## PART 12.5: Lead Intent Bridge Node

The intent bridge handles chained `FlightIntent` commands — when the SLM outputs `{"action": "takeoff", ..., "then": {"action": "move", ...}}`, the bridge dispatches the second step automatically after a delay. The main `lead_px4_commander` sees both steps sequentially.

### 12.5.1 Write lead_intent_bridge_node.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_intent_bridge_node.py
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
EOF
```

> **Note on `_BRIDGE_MARKER`:** The extra field is ignored by `lead_px4_commander` since it reads only known keys with `data.get('action')` etc. Pydantic is not invoked on this side.

> **Note on `wingman_intent_bridge`:** The architecture document mentions a `wingman_intent_bridge_node` for Drone-1. It is **not implemented** in this tutorial — the wingman commander handles all action types directly. If your project needs chained orders for the wingman, implement it by cloning this node and changing the topic from `/lead/approved_intent` to `/wingman/approved_intent`.

---

## PART 13: Config Files and Launch File

### 13.1 Lead Config

```bash
cat << 'EOF' > ~/major_ws/src/major_project/config/lead_config.yaml
lead_nlu_node:
  ros__parameters:
    ollama_host: "localhost"
    ollama_port: 11434
    model: "qwen2.5-coder:3b"
    num_ctx: 2048
    async_mode: false    # set true if latency benchmark showed > 400ms

lead_px4_commander_node:
  ros__parameters:
    drone_namespace: ""    # Drone-0 uses default namespace

lead_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0
EOF
```

### 13.2 Lead Launch File

```bash
cat << 'EOF' > ~/major_ws/src/major_project/launch/lead_pilot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('major_project'),
        'config', 'lead_config.yaml')

    return LaunchDescription([
        # GCS nodes (run on PC-1 alongside lead)
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

        # Lead Pilot nodes
        Node(package='major_project',
             executable='lead_sensor_aggregator',
             name='lead_sensor_aggregator_node',
             parameters=[config],
             output='screen'),

        Node(package='major_project',
             executable='lead_nlu',
             name='lead_nlu_node',
             parameters=[config],
             output='screen'),

        Node(package='major_project',
             executable='lead_px4_commander',
             name='lead_px4_commander_node',
             parameters=[config],
             output='screen'),

        Node(package='major_project',
             executable='lead_intent_bridge',
             name='lead_intent_bridge_node',
             parameters=[config],
             output='screen'),

        Node(package='major_project',
             executable='camera_detection',
             name='camera_detection_node',
             output='screen'),
    ])
EOF
```

### 13.3 Build and Test Lead Stack

```bash
# [PC-1]
cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash
```

**Verify the build:**
```bash
ros2 pkg executables major_project
# Should list all executables including lead_nlu, lead_px4_commander, etc.
```

---

## PART 14: End-to-End Lead Pilot Test

### 14.1 Start SITL + DDS

**Terminal 1:**
```bash
# [PC-1] Start 2-drone SITL
# PX4_GZ_MODEL_POSE format: "x,y,z,roll,pitch,yaw" (metres, radians)
#   x,y = ground position; z = 0 for ground level; roll/pitch/yaw = 0,0,0 for upright
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_UXRCE_DDS_KEY=1 ./build/px4_sitl_default/bin/px4 -i 0 -d
```

**Terminal 2:**
```bash
# [PC-1] Drone-1 — spawned 5m east of Drone-0 (PX4_GZ_MODEL_POSE="x,y,z,r,p,y")
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="5,0,0,0,0,0" \
PX4_UXRCE_DDS_KEY=2 ./build/px4_sitl_default/bin/px4 -i 1 -d
```

**Terminal 3:**
```bash
# [PC-1] DDS Agent
MicroXRCEAgent udp4 -p 8888
```

### 14.2 Launch Lead Stack

**Terminal 4:**
```bash
# [PC-1]
source ~/.bashrc
ros2 launch major_project lead_pilot.launch.py
```

Wait until you see all nodes started (no errors).

### 14.3 Test With Manual Voice Command Publication

Without using mic (to isolate the NLU test):

**Terminal 5:**
```bash
# [PC-1]
source ~/.bashrc

# Test 1: Clear command — should take off Drone-0
ros2 topic pub /voice_commands std_msgs/msg/String \
  "data: 'take off to 5 meters'" --once

# Watch Gazebo: Drone-0 should arm and rise to 5m
# Watch Terminal 4 output: should show HIGH confidence + execute
```

```bash
# Test 2: Ambiguous command — should request clarification
ros2 topic pub /voice_commands std_msgs/msg/String \
  "data: 'go that way'" --once
# Watch Terminal 4: should show LOW confidence
# Watch Terminal with clarification_speaker: should print question
```

```bash
# Test 3: Multi-drone command
ros2 topic pub /voice_commands std_msgs/msg/String \
  "data: 'take off to 10 meters. Wingman, hold at 8 meters and cover the south'" --once
# Drone-0 should take off
# /wingman/order topic should have a WingmanOrder JSON published
# Verify:
ros2 topic echo /wingman/order --once
```

**Expected output for Test 3 on `/wingman/order`:**
```json
{
  "order_id": "W001",
  "mission_context": "Cover the south sector while Lead takes off",
  "intent": {"action": "hover", "altitude": 8.0, "confidence": "high", ...},
  "priority": "routine",
  "confidence": "high"
}
```

---

*End of Part 3. Continue in tutorial_part4_wingman.md*
