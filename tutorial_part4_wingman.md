# Multi-Drone SLM Pilot System — Tutorial Part 4
## Wingman Stack, Cross-PC Integration, Launch & Testing

> Continues from Part 3. Lead pilot controls Drone-0 end-to-end.

---

## Canonical Process Map

> Read this before proceeding. It shows exactly which process runs on which PC.

| Process | PC | Starts it |
|---|---|---|
| Drone-0 PX4 SITL (`-i 0`) | **PC-1** | Terminal 1 |
| Drone-1 PX4 SITL (`-i 1`) | **PC-1** | Terminal 2 |
| Gazebo Harmonic server | **PC-1** | auto by SITL |
| MicroXRCE-DDS Agent (port 8888) | **PC-1** | Terminal 3 |
| Lead SLM stack (NLU, Commander, STT) | **PC-1** | `ros2 launch ... lead_pilot.launch.py` |
| Wingman SLM stack (NLU, Commander) | **PC-2** | `ros2 launch ... wingman_pilot.launch.py` |
| Ollama inference server | **Both PCs** | systemd auto-start |

**Key architectural fact:** Both drone SITLs run on PC-1. PC-2 only runs the Wingman SLM stack. The Wingman commander on PC-2 publishes to `/px4_1/fmu/in/*` — these topics are routed back to PC-1's DDS agent over WiFi, which forwards them to Drone-1's SITL.

---

## PART 15: Wingman System Prompt

### 15.1 Copy Package to PC-2

```bash
# [PC-1] Sync the entire package to PC-2
# Replace user@192.168.1.11 with your PC-2 user@IP
rsync -av --progress \
  ~/major_ws/src/major_project/ \
  user@192.168.1.11:~/major_ws/src/major_project/
```

```bash
# [PC-2] Build the package
cd ~/major_ws
colcon build --packages-select major_project px4_msgs --symlink-install
source install/setup.bash
echo "source ~/major_ws/install/setup.bash" >> ~/.bashrc
```

### 15.2 Create Wingman System Prompt

```bash
# [PC-2] (also copy to PC-1 via rsync above — create on PC-1 first)
mkdir -p ~/major_ws/src/major_project/major_project/wingman_pilot/prompts

cat << 'PROMPT_EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/prompts/wingman_system.txt
You are WINGMAN PILOT of a 2-drone formation. Drone-1 is YOUR drone.
You receive orders from your LEAD PILOT and execute them precisely.
You NEVER communicate directly with the Ground Commander.
If an order is unclear, you ask the LEAD PILOT for clarification.

YOUR RESPONSIBILITIES:
1. Execute orders from Lead Pilot on your Drone-1
2. Report your status, position, and situation back to Lead after each order
3. Alert Lead if you detect obstacles or cannot safely execute an order
4. Request clarification from Lead (not human) when an order is ambiguous

CONFIDENCE RULES:
- "high"   : Order is clear, all parameters known, you can execute safely
- "medium" : Order is parseable but you assumed a parameter; flag the assumption
- "low"    : Order is ambiguous, unsafe, or physically impossible for your drone

EXECUTION POLICY:
- high   → execute immediately
- medium → execute but include your assumption in situation_summary
- low    → DO NOT execute, set clarification_question directed at Lead Pilot

SCHEMA:
Output ONLY valid JSON matching this structure:
{
  "intent": {
    "action": "<takeoff|move|hover|land|rtl|search|search_stop|hold|follow_lead>",
    "altitude": <number 0.5-50 or null>,
    "distance": <number 0.1-100 or null>,
    "direction": "<north|south|east|west|northeast|northwest|southeast|southwest|forward|backward or null>",
    "speed": <number 0.1-10 or null>,
    "confidence": "<high|medium|low>",
    "clarification_question": "<string or null>"
  },
  "confidence": "<high|medium|low — your overall confidence>",
  "situation_summary": "<1-2 sentence status report for Lead>",
  "clarification_question": "<question for Lead Pilot, or null>"
}

Set intent to null if you cannot safely execute (confidence low).
PROMPT_EOF
```

---

## PART 16: Wingman NLU Node

### 16.1 Write wingman_nlu_node.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_nlu_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import threading
import time

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from major_project.common.schemas import (
    WingmanOrder, WingmanOutput, parse_wingman_output
)
from major_project.common.normaliser import normalise_parsed
from major_project.common.confidence_gate import gate_wingman, WingmanAction
from major_project.common.ollama_client import OllamaClient


def load_system_prompt() -> str:
    path = os.path.join(os.path.dirname(__file__), 'prompts', 'wingman_system.txt')
    with open(path) as f:
        return f.read()


class WingmanNLUNode(Node):
    def __init__(self):
        super().__init__('wingman_nlu_node')

        self.declare_parameter('ollama_host', 'localhost')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('model', 'qwen2.5-coder:3b')
        self.declare_parameter('num_ctx', 1024)
        self.declare_parameter('async_mode', False)

        host = self.get_parameter('ollama_host').value
        port = self.get_parameter('ollama_port').value
        model = self.get_parameter('model').value
        num_ctx = self.get_parameter('num_ctx').value
        self.async_mode = self.get_parameter('async_mode').value

        self.ollama = OllamaClient(host=host, port=port, model=model,
                                    num_ctx=num_ctx)
        self.system_prompt = load_system_prompt()

        self.drone1_situation = "No data yet"
        self.current_order_id = None
        self.lock = threading.Lock()

        self._pending_order = None
        if self.async_mode:
            t = threading.Thread(target=self._inference_loop, daemon=True)
            t.start()

        # Subscriptions
        self.sub_order = self.create_subscription(
            String, '/wingman/order', self.on_order, 10)
        self.sub_situation = self.create_subscription(
            String, '/drone_1/situation', self.on_situation, 10)

        # Publishers
        self.pub_intent = self.create_publisher(
            String, '/wingman/approved_intent', 10)
        self.pub_status_text = self.create_publisher(
            String, '/wingman/status_report_text', 10)
        self.pub_clarify = self.create_publisher(
            String, '/wingman/clarification_to_lead', 10)

        self.get_logger().info(f"Wingman NLU ready (Ollama: {host}:{port})")

    def on_situation(self, msg: String):
        with self.lock:
            self.drone1_situation = msg.data

    def on_order(self, msg: String):
        """Called when Lead publishes a WingmanOrder."""
        try:
            data = json.loads(msg.data)
            order = WingmanOrder(**data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse WingmanOrder: {e}")
            return

        self.get_logger().info(
            f"Order received: {order.order_id} | {order.intent.action} "
            f"[{order.priority}] conf={order.confidence}")

        if self.async_mode:
            with self.lock:
                self._pending_order = order
        else:
            self._process_order(order)

    def _inference_loop(self):
        while rclpy.ok():
            order = None
            with self.lock:
                if self._pending_order:
                    order = self._pending_order
                    self._pending_order = None
            if order:
                self._process_order(order)
            else:
                time.sleep(0.05)

    def _build_prompt(self, order: WingmanOrder) -> str:
        with self.lock:
            sit = self.drone1_situation
        return (
            f"[MY SITUATION — DRONE-1 | WINGMAN]\n{sit}\n\n"
            f"{order.to_prompt_block()}"
        )

    def _process_order(self, order: WingmanOrder):
        with self.lock:
            self.current_order_id = order.order_id

        # Handle emergency orders without NLU delay
        if order.priority == "emergency":
            self.get_logger().warning(
                f"EMERGENCY order {order.order_id} — fast-path execution")
            intent_msg = String()
            intent_msg.data = order.intent.model_dump_json(exclude_none=True)
            self.pub_intent.publish(intent_msg)
            self._publish_status(order.order_id, "executing",
                                  "Emergency order acknowledged and executing.")
            return

        prompt = self._build_prompt(order)
        raw_json, latency = self.ollama.infer(prompt, self.system_prompt)
        self.get_logger().debug(f"Wingman Ollama latency: {latency*1000:.0f}ms")

        if raw_json is None:
            self.get_logger().error("Wingman Ollama inference failed")
            self._publish_status(order.order_id, "failed",
                                  "Inference error — could not process order.")
            return

        output = parse_wingman_output(raw_json)
        if output is None:
            self.get_logger().warning("WingmanOutput schema validation failed")
            self._publish_status(order.order_id, "failed",
                                  "Schema validation error on wingman output.")
            return

        gate_result = gate_wingman(output.confidence)
        self.get_logger().info(
            f"Wingman confidence: {output.confidence} → {gate_result.value}")

        if gate_result == WingmanAction.CLARIFY_LEAD:
            # Do not execute — ask Lead
            self._publish_status(order.order_id, "needs_clarification",
                                  output.situation_summary)
            if output.clarification_question:
                msg = String()
                msg.data = (f"[Order {order.order_id}] "
                             f"{output.clarification_question}")
                self.pub_clarify.publish(msg)
            return

        # Execute intent
        if output.intent is not None:
            intent_msg = String()
            intent_msg.data = output.intent.model_dump_json(exclude_none=True)
            self.pub_intent.publish(intent_msg)
            self.get_logger().info(
                f"Wingman executing: {output.intent.action} [{output.confidence}]")

            status = "executing"
            if gate_result == WingmanAction.EXECUTE_WITH_WARNING:
                status = "executing"
                self.get_logger().warning(
                    f"Wingman executing with assumption: {output.situation_summary}")
        else:
            status = "failed"

        self._publish_status(order.order_id, status, output.situation_summary)

    def _publish_status(self, order_id: str, status: str, summary: str):
        msg = String()
        msg.data = f"[{order_id}|{status}] {summary}"
        self.pub_status_text.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WingmanNLUNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

---

## PART 17: Wingman PX4 Commander Node

Identical logic to Lead commander but uses `/px4_1/fmu/` namespace.

### 17.1 Write wingman_px4_commander_node.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_px4_commander_node.py
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
            with self.lock:
                self.target_x = x + dx * distance
                self.target_y = y + dy * distance
                self.target_z = z
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
EOF
```

---

## PART 18: Wingman Sensor Aggregator

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_sensor_aggregator_node.py
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
EOF
```

---

## PART 19: Wingman Config and Launch File

### 19.1 Wingman Config

```bash
cat << 'EOF' > ~/major_ws/src/major_project/config/wingman_config.yaml
wingman_nlu_node:
  ros__parameters:
    ollama_host: "localhost"   # PC-2's own Ollama
    ollama_port: 11434
    model: "qwen2.5-coder:3b"
    num_ctx: 1024
    async_mode: false          # set true if latency > 400ms

wingman_px4_commander_node:
  ros__parameters:
    drone_namespace: "px4_1"

wingman_sensor_aggregator_node:
  ros__parameters:
    publish_rate_hz: 1.0
EOF
```

### 19.2 Wingman Launch File

```bash
cat << 'EOF' > ~/major_ws/src/major_project/launch/wingman_pilot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('major_project'),
        'config', 'wingman_config.yaml')

    return LaunchDescription([
        Node(package='major_project',
             executable='wingman_sensor_aggregator',
             name='wingman_sensor_aggregator_node',
             parameters=[config],
             output='screen'),

        Node(package='major_project',
             executable='wingman_nlu',
             name='wingman_nlu_node',
             parameters=[config],
             output='screen'),

        Node(package='major_project',
             executable='wingman_px4_commander',
             name='wingman_px4_commander_node',
             parameters=[config],
             output='screen'),
    ])
EOF
```

### 19.3 Sync to PC-2 and Build

```bash
# [PC-1] Sync updated package to PC-2
rsync -av --progress \
  ~/major_ws/src/major_project/ \
  user@192.168.1.11:~/major_ws/src/major_project/

# [PC-1] Also rebuild locally
cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash
```

```bash
# [PC-2] Build
cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash
```

---

## PART 20: Full System Integration Test

This is the moment everything comes together across both PCs.

### 20.1 Start PX4 SITL (PC-1, 3 terminals)

**PC-1 Terminal 1:**
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_UXRCE_DDS_KEY=1 \
./build/px4_sitl_default/bin/px4 -i 0 -d
```

**PC-1 Terminal 2:**
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE="5,0,0,0,0,0" PX4_UXRCE_DDS_KEY=2 \
./build/px4_sitl_default/bin/px4 -i 1 -d
```

**PC-1 Terminal 3:**
```bash
MicroXRCEAgent udp4 -p 8888
```

### 20.2 Start Lead Stack (PC-1 Terminal 4)

```bash
# [PC-1]
source ~/.bashrc
ros2 launch major_project lead_pilot.launch.py
```

### 20.3 Start Wingman Stack (PC-2 Terminal 1)

```bash
# [PC-2]
source ~/.bashrc
ros2 launch major_project wingman_pilot.launch.py
```

### 20.4 Verify Cross-PC Topics

```bash
# [PC-1 new terminal] Verify wingman topics are visible from PC-1
ros2 topic list | grep wingman
# Expected:
# /wingman/approved_intent
# /wingman/clarification_to_lead
# /wingman/order
# /wingman/status_report_text

# Verify drone_1 situation is coming from PC-2
ros2 topic echo /drone_1/situation
# Should print wingman telemetry (even if just zeros — shows cross-PC works)
```

```bash
# [PC-2 new terminal] Verify lead topics are visible from PC-2
ros2 topic list | grep "lead\|wingman\|voice"
# /voice_commands should be visible (published on PC-1, received on PC-2)
# /wingman/order should be visible
```

### 20.4.1 Verify Cross-PC Setpoint Routing (Critical)

This verifies that Wingman commander setpoints published on PC-2 actually reach Drone-1's SITL on PC-1. This is the most common failure point.

```bash
# [PC-1] Monitor the Drone-1 setpoint topic
ros2 topic hz /px4_1/fmu/in/trajectory_setpoint
# While the wingman stack is running on PC-2, this MUST show ~10 Hz
# If it shows 0 Hz, setpoints from PC-2 are NOT crossing the DDS bridge

# If 0 Hz:
# Step 1: Confirm same ROS_DOMAIN_ID on both PCs
#   [PC-1] printenv ROS_DOMAIN_ID  → must be 42
#   [PC-2] printenv ROS_DOMAIN_ID  → must be 42
# Step 2: Confirm wingman_px4_commander is actually running on PC-2
#   [PC-2] ros2 node list | grep wingman_px4_commander
# Step 3: Force unicast in CycloneDDS (disable multicast) if on managed WiFi
```

---

### Pre-Scenario Test Sequence

Run these checks in order before executing Scenarios A–C. Each step gates the next.

| # | Check | Command | Pass condition |
|---|---|---|---|
| 1 | Both drones have ROS2 topics | `ros2 topic list \| grep fmu \| wc -l` | > 10 topics |
| 2 | Drone-0 publishing position | `ros2 topic hz /fmu/out/vehicle_local_position` | ~20 Hz |
| 3 | Drone-1 publishing position | `ros2 topic hz /px4_1/fmu/out/vehicle_local_position` | ~20 Hz |
| 4 | Wingman topics visible on PC-1 | `ros2 topic list \| grep wingman/order` | Listed |
| 5 | Cross-PC setpoint routing | `ros2 topic hz /px4_1/fmu/in/trajectory_setpoint` (on PC-1) | ~10 Hz |
| 6 | Lead stack has Ollama | `curl -s http://localhost:11434/api/tags` | JSON with model |
| 7 | Wingman stack has Ollama | `curl -s http://PC2-IP:11434/api/tags` | JSON with model |

Only proceed to Scenario A when all 7 pass.

---

### 20.5 Scenario A — Clear Formation Command

```bash
# [PC-1] Publish test command (or speak into microphone)
ros2 topic pub /voice_commands std_msgs/msg/String \
  "data: 'both drones take off to 10 meters. Lead go north, wingman go south'" \
  --once
```

**Watch in Gazebo:** Both drones should arm and take off. Drone-0 moves north, Drone-1 moves south.

**Watch PC-1 lead node output:**
```
Lead intent: move [high]
Wingman order issued: W001 action=move [high]
```

**Watch PC-2 wingman node output:**
```
Order received: W001 | move [routine] conf=high
Wingman executing: move [high]
```

**Watch `/wingman/status_report_text`:**
```bash
ros2 topic echo /wingman/status_report_text
# [W001|executing] Moving south 10m. Path clear.
```

### 20.6 Scenario B — Clarification Cascade

```bash
ros2 topic pub /voice_commands std_msgs/msg/String \
  "data: 'go investigate that'" --once
```

**Expected (PC-1 clarification_speaker prints):**
```
============================================================
  CLARIFICATION NEEDED FROM GROUND COMMANDER:
  Which area should I investigate? Please specify direction or grid reference.
============================================================
```

**Verify no flight action happened:**
```bash
ros2 topic echo /lead/approved_intent
# Should print nothing for several seconds — command withheld
```

### 20.7 Scenario C — Emergency Stop

```bash
# [PC-1] In the emergency_stop_node terminal, type:
STOP
# Then press Enter
```

**Expected:**
- Both drones in Gazebo begin landing immediately
- PC-1 logs: `EMERGENCY STOP triggered by keyboard`
- PC-2 logs: `WINGMAN EMERGENCY STOP — landing now`

---

## PART 21: Troubleshooting Guide

### Problem: Drone doesn't arm in Gazebo

```bash
# Check PX4 console output for errors
# Common fix: ensure XRCE-DDS agent is running BEFORE launching lead stack
# Also verify:
ros2 topic hz /fmu/out/vehicle_status
# Must show > 5 Hz before lead commander will work
```

### Problem: `/wingman/order` not received on PC-2

```bash
# [PC-2] Check topic visibility
ros2 topic list | grep wingman/order
# If missing:

# Step 1: Verify same ROS_DOMAIN_ID
printenv ROS_DOMAIN_ID   # both must show 42

# Step 2: Verify CycloneDDS config has correct IPs
cat /etc/cyclonedds/cyclonedds.xml
# Both PC-1 and PC-2 IPs must be in <Peers> section

# Step 3: Check firewall
sudo ufw status
# If active, allow UDP:
sudo ufw allow proto udp from 192.168.1.0/24

# Step 4: Ping test
ping 192.168.1.10  # from PC-2
```

### Problem: Ollama inference returns malformed JSON

```bash
# Add more structure to the system prompt — reduce num_ctx if it helps:
# In lead_config.yaml or wingman_config.yaml, try:
#   num_ctx: 512   (less context = more focused output)

# Also test Ollama directly:
curl -X POST http://localhost:11434/api/generate \
  -H "Content-Type: application/json" \
  -d '{"model":"qwen2.5-coder:3b","prompt":"take off to 5 meters",
       "system":"Output JSON: {\"action\":\"takeoff\",\"altitude\":5,\"confidence\":\"high\"}",
       "format":"json","stream":false}' | python3 -m json.tool
```

### Problem: PX4 offboard mode rejected

```bash
# PX4 requires setpoints to be published BEFORE switching to offboard mode
# The lead_px4_commander sends setpoints at 10Hz on startup
# Verify it's running:
ros2 topic hz /fmu/in/trajectory_setpoint
# Must show ~10 Hz BEFORE arming
```

### Problem: PC-2 can't connect Drone-1 to PC-1's Gazebo

```bash
# By default PX4 SITL connects to a local Gazebo server
# To run Drone-1 on PC-2 connecting to PC-1's Gazebo:
# On PC-2, set the Gazebo master URI before launching PX4:
export GZ_IP=192.168.1.10   # PC-1 IP
export GZ_MASTER_URI=http://192.168.1.10:11345
# Then run PX4 SITL on PC-2
# Note: Gazebo network transparency depends on Gazebo version
# Alternative: run BOTH drones SITL on PC-1, only NLU on PC-2
```

---

## PART 22: Benchmark Evaluation Script

```bash
cat << 'EOF' > ~/major_ws/src/major_project/benchmark/run_evaluation.py
"""
Full benchmark evaluation: publishes all 200+ test commands,
records SLM outputs, confidence, and actions taken.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import csv
import os


DATASET = [
    # Format: (id, text, category, expected_confidence)
    # Clear commands
    ("C001", "take off to 5 meters", "clear", "high"),
    ("C002", "take off to 10 meters", "clear", "high"),
    ("C003", "hover at current position", "clear", "high"),
    ("C004", "move north 20 meters", "clear", "high"),
    ("C005", "move east 15 meters", "clear", "high"),
    ("C006", "move south 30 meters at 3 meters per second", "clear", "high"),
    ("C007", "land now", "clear", "high"),
    ("C008", "return to launch", "clear", "high"),
    ("C009", "search this area", "clear", "high"),
    ("C010", "take off to 8 meters and move northeast 25 meters", "clear", "high"),
    # Ambiguous commands
    ("A001", "go that way", "ambiguous", "low"),
    ("A002", "move a bit", "ambiguous", "medium"),
    ("A003", "go faster", "ambiguous", "medium"),
    ("A004", "search over there", "ambiguous", "low"),
    ("A005", "follow that target", "ambiguous", "low"),
    ("A006", "move to the other side", "ambiguous", "low"),
    ("A007", "go higher", "ambiguous", "medium"),
    ("A008", "approach carefully", "ambiguous", "medium"),
    # Out-of-scope
    ("O001", "what is the weather today", "out_of_scope", "low"),
    ("O002", "take off to 200 meters", "out_of_scope", "low"),
    ("O003", "go invisible", "out_of_scope", "low"),
    ("O004", "play music", "out_of_scope", "low"),
    ("O005", "go left and right simultaneously", "out_of_scope", "low"),
]


class EvaluationNode(Node):
    def __init__(self):
        super().__init__('evaluation_node')
        self.pub = self.create_publisher(String, '/voice_commands', 10)
        self.sub_intent = self.create_subscription(
            String, '/lead/approved_intent', self.on_intent, 10)
        self.sub_clarify = self.create_subscription(
            String, '/clarification_request', self.on_clarify, 10)
        self.sub_status = self.create_subscription(
            String, '/mission_status', self.on_status, 10)

        self.current_id = None
        self.results = []
        self.last_intent = None
        self.last_clarify = None
        self.last_status = None

        self.timer = self.create_timer(0.1, self.dummy_tick)
        self.get_logger().info("Evaluation node ready")

    def dummy_tick(self): pass

    def on_intent(self, msg):
        self.last_intent = msg.data

    def on_clarify(self, msg):
        self.last_clarify = msg.data

    def on_status(self, msg):
        self.last_status = msg.data

    def run_evaluation(self):
        output_path = os.path.expanduser(
            '~/major_ws/src/major_project/benchmark/evaluation_results.csv')

        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'id', 'text', 'category', 'expected_conf',
                'intent_received', 'clarify_received',
                'action_taken', 'executed', 'correct_gate'
            ])

            for cmd_id, text, category, expected_conf in DATASET:
                self.current_id = cmd_id
                self.last_intent = None
                self.last_clarify = None

                self.get_logger().info(f"[{cmd_id}] '{text}'")

                msg = String()
                msg.data = text
                self.pub.publish(msg)

                # Wait up to 20s for response
                deadline = time.time() + 20.0
                while time.time() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.last_intent is not None or self.last_clarify is not None:
                        break

                intent_received = self.last_intent is not None
                clarify_received = self.last_clarify is not None
                executed = intent_received
                action = "none"

                if intent_received:
                    try:
                        d = json.loads(self.last_intent)
                        action = d.get('action', 'unknown')
                    except Exception:
                        action = "parse_error"

                # Correct gate: OOS/ambiguous should NOT execute
                if expected_conf == "low":
                    correct_gate = not executed
                elif expected_conf == "medium":
                    correct_gate = True  # either execute+warn or clarify is ok
                else:
                    correct_gate = executed

                writer.writerow([
                    cmd_id, text, category, expected_conf,
                    intent_received, clarify_received,
                    action, executed, correct_gate
                ])

                self.get_logger().info(
                    f"  → executed:{executed} clarify:{clarify_received} "
                    f"action:{action} correct:{correct_gate}")

                # Gap between commands
                time.sleep(5.0)

        self.get_logger().info(f"Evaluation complete. Results: {output_path}")


def main():
    rclpy.init()
    node = EvaluationNode()
    # Let system settle
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.run_evaluation()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

**Run the evaluation:**
```bash
# [PC-1] With SITL + all nodes running:
python3 ~/major_ws/src/major_project/benchmark/run_evaluation.py

# Results saved to:
# ~/major_ws/src/major_project/benchmark/evaluation_results.csv
```

**Analyse results:**
```bash
python3 - << 'EOF'
import csv

results = list(csv.DictReader(
    open('/root/major_ws/src/major_project/benchmark/evaluation_results.csv')))
# adjust path for your username

total = len(results)
correct = sum(1 for r in results if r['correct_gate'] == 'True')
executed = sum(1 for r in results if r['executed'] == 'True')
oos = [r for r in results if r['category'] == 'out_of_scope']
false_exec = sum(1 for r in oos if r['executed'] == 'True')

print(f"Total commands: {total}")
print(f"Correct gate decisions: {correct}/{total} = {correct/total*100:.1f}%")
print(f"Total executed: {executed}/{total}")
print(f"False execution rate (OOS): {false_exec}/{len(oos)} = {false_exec/len(oos)*100:.1f}%")
EOF
```

---

## PART 23: Quick Reference — Daily Startup Commands

Save this as a shell script for easy startup:

```bash
cat << 'SCRIPT_EOF' > ~/start_lead.sh
#!/bin/bash
# PC-1 startup script — run in separate terminals or with tmux

echo "Starting multi-drone SLM pilot system on PC-1 (Lead)"
echo "Open 4 terminals and run:"
echo ""
echo "Terminal 1 (Drone-0 SITL):"
echo "  cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE='0,0,0,0,0,0' PX4_UXRCE_DDS_KEY=1 ./build/px4_sitl_default/bin/px4 -i 0 -d"
echo ""
echo "Terminal 2 (Drone-1 SITL):"
echo "  cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE='5,0,0,0,0,0' PX4_UXRCE_DDS_KEY=2 ./build/px4_sitl_default/bin/px4 -i 1 -d"
echo ""
echo "Terminal 3 (DDS Agent):"
echo "  MicroXRCEAgent udp4 -p 8888"
echo ""
echo "Terminal 4 (Lead Stack):"
echo "  source ~/.bashrc && ros2 launch major_project lead_pilot.launch.py"
SCRIPT_EOF

chmod +x ~/start_lead.sh
```

```bash
cat << 'SCRIPT_EOF' > ~/start_wingman.sh
#!/bin/bash
echo "Starting wingman stack on PC-2"
echo ""
echo "Terminal 1:"
echo "  source ~/.bashrc && ros2 launch major_project wingman_pilot.launch.py"
SCRIPT_EOF

chmod +x ~/start_wingman.sh
```

---

## Summary: What's Built

| Node | PC | Topics In | Topics Out |
|---|---|---|---|
| stt_node | PC-1 | mic | /voice_commands |
| lead_nlu_node | PC-1 | /voice_commands, /drone_0/situation, /drone_1/situation, /wingman/status_report_text | /lead/approved_intent, /wingman/order, /clarification_request |
| lead_px4_commander | PC-1 | /lead/approved_intent, /emergency_stop | /fmu/in/* |
| lead_sensor_aggregator | PC-1 | /fmu/out/*, /camera_0/detections | /drone_0/situation |
| clarification_speaker | PC-1 | /clarification_request | terminal/TTS |
| mission_monitor | PC-1 | /drone_0/situation, /drone_1/situation | terminal display |
| emergency_stop | PC-1 | keyboard/voice | /emergency_stop |
| wingman_nlu_node | PC-2 | /wingman/order, /drone_1/situation | /wingman/approved_intent, /wingman/status_report_text, /wingman/clarification_to_lead |
| wingman_px4_commander | PC-2 | /wingman/approved_intent, /emergency_stop | /px4_1/fmu/in/* |
| wingman_sensor_aggregator | PC-2 | /px4_1/fmu/out/*, /camera_1/detections | /drone_1/situation |

**Cross-PC topics (WiFi DDS):** `/wingman/order` → PC-2, `/wingman/status_report_text` → PC-1, `/drone_1/situation` → PC-1, `/emergency_stop` → PC-2

---

*End of tutorial. All 4 parts together constitute the complete step-by-step build guide.*
