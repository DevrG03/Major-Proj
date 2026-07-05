# Part 6 — Safety & Sensor Layer

This part covers the three nodes that form the safety and situational-awareness foundation of the swarm:

| Node | File | Role |
|------|------|------|
| `SafetyMonitorNode` | `lead_pilot/safety_monitor_node.py` | Cross-drone battery, GPS, proximity, emergency-stop |
| `LeadSensorAggregatorNode` | `lead_pilot/lead_sensor_aggregator_node.py` | Fuses Lead telemetry + wingman position → `/drone_0/situation` |
| `WingmanSensorAggregatorNode` | `wingman_pilot/wingman_sensor_aggregator_node.py` | Fuses Wingman telemetry → `/drone_1/situation` |

---

## 6.1 QoS Reference

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

> [!IMPORTANT]
> PX4 uXRCE telemetry topics (`/fmu/out/…`) **require** `BEST_EFFORT_QOS`. Using `RELIABLE` will silently drop all messages.

---

## 6.2 Safety Event JSON Schema

Every safety event published to `/safety/event` (String) is a JSON object with this structure:

```json
{
  "event_type": "low_battery | battery_rtl | gps_lost | emergency_stop | proximity_warning",
  "drone_id":   "DRONE_0 | DRONE_1 | BOTH | ALL",
  "severity":   "warning | critical",
  "message":    "Human-readable description",
  "value":      1.23
}
```

`value` is `null` when no numeric payload is applicable (e.g. `gps_lost`).

---

## 6.3 `SafetyMonitorNode`

**File:** `major_project/lead_pilot/safety_monitor_node.py`

### Design overview

```
/fmu/out/battery_status          ─┐
/px4_1/fmu/out/battery_status    ─┤  battery_cb (per drone)
/fmu/out/vehicle_status          ─┤  status_cb  (GPS fix, arming)
/px4_1/fmu/out/vehicle_status    ─┤
/fmu/out/vehicle_local_position  ─┤  position_cb (separation calc)
/px4_1/fmu/out/vehicle_local_position ┘
/emergency_stop  (Bool, RELIABLE) ─→  e-stop handler → NAV_LAND both drones

Timer 1 Hz ──→ _safety_check()
               ├─ battery warn / RTL
               ├─ GPS fix lost
               └─ proximity warning (rate-limited)

Publishers:
  /safety/event            String
  /fmu/in/vehicle_command  VehicleCommand  (Drone-0)
  /px4_1/fmu/in/vehicle_command  VehicleCommand  (Drone-1)
```

### Complete implementation

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/safety_monitor_node.py
#!/usr/bin/env python3
"""
safety_monitor_node.py
Cross-drone safety monitor for the two-drone PX4 swarm.

Monitors:
  - Battery level for DRONE_0 and DRONE_1
  - GPS fix status for both drones
  - Inter-drone separation (proximity warning)
  - Hardware /emergency_stop signal

Publishes:
  - /safety/event  (String JSON)
  - /fmu/in/vehicle_command  (VehicleCommand – Drone-0)
  - /px4_1/fmu/in/vehicle_command  (VehicleCommand – Drone-1)
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    BatteryStatus,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from std_msgs.msg import Bool, String

# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SafetyMonitorNode(Node):
    """
    Periodic (1 Hz) cross-drone safety monitor.

    Parameters
    ----------
    battery_warn_pct          : float = 20.0   – warn threshold (%)
    battery_rtl_pct           : float = 15.0   – RTL threshold (%)
    min_separation_m          : float = 5.0    – proximity danger distance (m)
    proximity_warn_interval_sec: float = 5.0   – rate-limit for proximity events
    """

    # PX4 VehicleCommand target_system IDs
    _TARGET_SYSTEM_DRONE0: int = 1
    _TARGET_SYSTEM_DRONE1: int = 2

    def __init__(self) -> None:
        super().__init__("safety_monitor_node")

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter("battery_warn_pct", 20.0)
        self.declare_parameter("battery_rtl_pct", 15.0)
        self.declare_parameter("min_separation_m", 3.0)
        self.declare_parameter("proximity_warn_interval_sec", 5.0)

        self._bat_warn_pct: float = (
            self.get_parameter("battery_warn_pct").get_parameter_value().double_value
        )
        self._bat_rtl_pct: float = (
            self.get_parameter("battery_rtl_pct").get_parameter_value().double_value
        )
        self._min_sep_m: float = (
            self.get_parameter("min_separation_m").get_parameter_value().double_value
        )
        self._prox_warn_interval: float = (
            self.get_parameter("proximity_warn_interval_sec")
            .get_parameter_value()
            .double_value
        )

        # ------------------------------------------------------------------ #
        # State
        # ------------------------------------------------------------------ #
        # Battery percentage (0–100). None = not yet received.
        self._bat_pct: dict[str, Optional[float]] = {
            "DRONE_0": None,
            "DRONE_1": None,
        }
        # Battery RTL already triggered flags (prevent repeated commands)
        self._rtl_triggered: dict[str, bool] = {
            "DRONE_0": False,
            "DRONE_1": False,
        }
        # Battery warn already fired (prevent log spam; resets if battery rises)
        self._bat_warned: dict[str, bool] = {
            "DRONE_0": False,
            "DRONE_1": False,
        }

        # GPS fix: True = fix OK, False = lost
        self._gps_ok: dict[str, Optional[bool]] = {
            "DRONE_0": None,
            "DRONE_1": None,
        }
        self._gps_lost_warned: dict[str, bool] = {
            "DRONE_0": False,
            "DRONE_1": False,
        }

        # Local positions
        self._pos: dict[str, Optional[tuple[float, float, float]]] = {
            "DRONE_0": None,
            "DRONE_1": None,
        }

        # Rate-limiting: last timestamp a proximity warning was published
        self._last_prox_warn_ts: float = 0.0

        # Emergency stop: only act once per rising edge
        self._estop_handled: bool = False

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self._safety_pub = self.create_publisher(
            String, "/safety/event", RELIABLE_QOS
        )
        self._cmd_pub_d0 = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", RELIABLE_QOS
        )
        self._cmd_pub_d1 = self.create_publisher(
            VehicleCommand, "/px4_1/fmu/in/vehicle_command", RELIABLE_QOS
        )

        # ------------------------------------------------------------------ #
        # Subscribers – Drone-0 (Lead)
        # ------------------------------------------------------------------ #
        self.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status_v1",
            lambda msg: self._on_battery(msg, "DRONE_0"),
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            lambda msg: self._on_position(msg, "DRONE_0"),
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            lambda msg: self._on_vehicle_status(msg, "DRONE_0"),
            BEST_EFFORT_QOS,
        )

        # ------------------------------------------------------------------ #
        # Subscribers – Drone-1 (Wingman)
        # ------------------------------------------------------------------ #
        self.create_subscription(
            BatteryStatus,
            "/px4_1/fmu/out/battery_status_v1",
            lambda msg: self._on_battery(msg, "DRONE_1"),
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/px4_1/fmu/out/vehicle_local_position_v1",
            lambda msg: self._on_position(msg, "DRONE_1"),
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            VehicleStatus,
            "/px4_1/fmu/out/vehicle_status_v1",
            lambda msg: self._on_vehicle_status(msg, "DRONE_1"),
            BEST_EFFORT_QOS,
        )

        # ------------------------------------------------------------------ #
        # Emergency-stop subscriber (RELIABLE so we never miss it)
        # ------------------------------------------------------------------ #
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._on_emergency_stop,
            RELIABLE_QOS,
        )

        # ------------------------------------------------------------------ #
        # Periodic safety check timer – 1 Hz
        # ------------------------------------------------------------------ #
        self.create_timer(1.0, self._safety_check)

        self.get_logger().info("SafetyMonitorNode started.")

    # ------------------------------------------------------------------ #
    # Callbacks – telemetry
    # ------------------------------------------------------------------ #

    def _on_battery(self, msg: BatteryStatus, drone_id: str) -> None:
        """Convert PX4 remaining (0.0–1.0) to percentage and cache."""
        pct = float(msg.remaining) * 100.0
        self._bat_pct[drone_id] = pct

        # Reset warn flag if battery somehow recovers (e.g. hot-swap)
        if pct >= self._bat_warn_pct:
            self._bat_warned[drone_id] = False
        if pct >= self._bat_rtl_pct:
            self._rtl_triggered[drone_id] = False

    def _on_position(self, msg: VehicleLocalPosition, drone_id: str) -> None:
        """Cache NED local position (x=North, y=East, z=Down)."""
        self._pos[drone_id] = (float(msg.x), float(msg.y), float(msg.z))

    def _on_vehicle_status(self, msg: VehicleStatus, drone_id: str) -> None:
        """
        Extract GPS fix quality.
        gps_node_used_mask > 0 means at least one GPS is used; a value of 0
        combined with vehicle_type != 0 (not unset) indicates GPS loss.
        We use the simpler heuristic: fix is OK when
        vehicle_gps_comm_failed == False (field name varies by PX4 version).
        Fallback: check pre_flight_checks_pass.
        """
        # VehicleStatus.pre_flight_checks_pass covers GPS among other things.
        # For GPS-specific: check if position setpoint type is not GPS-denied.
        # Most reliable field across PX4 1.14/1.15: vehicle_type + gps used.
        try:
            gps_ok: bool = bool(msg.pre_flight_checks_pass)
        except AttributeError:
            # Older px4_msgs: fall back to always True (best effort)
            gps_ok = True
        self._gps_ok[drone_id] = gps_ok
        if gps_ok:
            self._gps_lost_warned[drone_id] = False

    # ------------------------------------------------------------------ #
    # Emergency stop callback
    # ------------------------------------------------------------------ #

    def _on_emergency_stop(self, msg: Bool) -> None:
        """
        On rising edge of /emergency_stop, immediately land BOTH drones.
        Uses VEHICLE_CMD_NAV_LAND which works regardless of flight mode.
        """
        if not msg.data:
            # Falling edge — reset so next rising edge is handled
            self._estop_handled = False
            return

        if self._estop_handled:
            return  # Already processed this rising edge

        self._estop_handled = True
        self.get_logger().error(
            "EMERGENCY STOP received! Sending NAV_LAND to BOTH drones."
        )

        self._send_vehicle_command(
            pub=self._cmd_pub_d0,
            command=VehicleCommand.VEHICLE_CMD_NAV_LAND,
            target_system=self._TARGET_SYSTEM_DRONE0,
        )
        self._send_vehicle_command(
            pub=self._cmd_pub_d1,
            command=VehicleCommand.VEHICLE_CMD_NAV_LAND,
            target_system=self._TARGET_SYSTEM_DRONE1,
        )

        self._publish_safety_event(
            event_type="emergency_stop",
            drone_id="BOTH",
            severity="critical",
            message="Emergency stop activated — NAV_LAND sent to DRONE_0 and DRONE_1.",
            value=None,
        )

    # ------------------------------------------------------------------ #
    # Periodic safety check
    # ------------------------------------------------------------------ #

    def _safety_check(self) -> None:
        """Run all safety checks at 1 Hz."""
        self._check_battery("DRONE_0")
        self._check_battery("DRONE_1")
        self._check_gps("DRONE_0")
        self._check_gps("DRONE_1")
        self._check_proximity()

    # ------------------------------------------------------------------ #
    # Safety check helpers
    # ------------------------------------------------------------------ #

    def _check_battery(self, drone_id: str) -> None:
        pct = self._bat_pct[drone_id]
        if pct is None:
            return  # No data yet

        if pct < self._bat_rtl_pct and not self._rtl_triggered[drone_id]:
            self._rtl_triggered[drone_id] = True
            self._bat_warned[drone_id] = True  # Subsumes the warn
            self.get_logger().error(
                f"[{drone_id}] CRITICAL battery {pct:.1f}% < {self._bat_rtl_pct}%. "
                f"Sending RTL."
            )
            self._send_rtl(drone_id)
            self._publish_safety_event(
                event_type="battery_rtl",
                drone_id=drone_id,
                severity="critical",
                message=(
                    f"{drone_id} battery critically low — RTL commanded."
                ),
                value=round(pct, 1),
            )

        elif (
            self._bat_warn_pct > pct >= self._bat_rtl_pct
            and not self._bat_warned[drone_id]
        ):
            self._bat_warned[drone_id] = True
            self.get_logger().warning(
                f"[{drone_id}] LOW battery {pct:.1f}% < {self._bat_warn_pct}%."
            )
            self._publish_safety_event(
                event_type="low_battery",
                drone_id=drone_id,
                severity="warning",
                message=f"{drone_id} battery low at {pct:.1f}%.",
                value=round(pct, 1),
            )

    def _check_gps(self, drone_id: str) -> None:
        gps_ok = self._gps_ok[drone_id]
        if gps_ok is None:
            return  # No data yet

        if not gps_ok and not self._gps_lost_warned[drone_id]:
            self._gps_lost_warned[drone_id] = True
            self.get_logger().error(f"[{drone_id}] GPS fix LOST.")
            self._publish_safety_event(
                event_type="gps_lost",
                drone_id=drone_id,
                severity="critical",
                message=f"{drone_id} has lost GPS fix.",
                value=None,
            )

    def _check_proximity(self) -> None:
        pos0 = self._pos.get("DRONE_0")
        pos1 = self._pos.get("DRONE_1")
        if pos0 is None or pos1 is None:
            return  # Cannot compute separation

        dx = pos0[0] - pos1[0]
        dy = pos0[1] - pos1[1]
        dz = pos0[2] - pos1[2]
        separation = math.sqrt(dx * dx + dy * dy + dz * dz)

        if separation < self._min_sep_m:
            now = time.monotonic()
            if now - self._last_prox_warn_ts >= self._prox_warn_interval:
                self._last_prox_warn_ts = now
                self.get_logger().warning(
                    f"PROXIMITY WARNING: drones {separation:.2f}m apart "
                    f"(min {self._min_sep_m}m)."
                )
                self._publish_safety_event(
                    event_type="proximity_warning",
                    drone_id="BOTH",
                    severity="warning",
                    message=(
                        f"Drones are {separation:.2f}m apart — "
                        f"below minimum safe separation of {self._min_sep_m}m."
                    ),
                    value=round(separation, 2),
                )

    # ------------------------------------------------------------------ #
    # Command helpers
    # ------------------------------------------------------------------ #

    def _send_rtl(self, drone_id: str) -> None:
        """Send RTL command to the specified drone."""
        if drone_id == "DRONE_0":
            self._send_vehicle_command(
                pub=self._cmd_pub_d0,
                command=VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
                target_system=self._TARGET_SYSTEM_DRONE0,
            )
        elif drone_id == "DRONE_1":
            self._send_vehicle_command(
                pub=self._cmd_pub_d1,
                command=VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
                target_system=self._TARGET_SYSTEM_DRONE1,
            )

    def _send_vehicle_command(
        self,
        pub,
        command: int,
        target_system: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        """Construct and publish a VehicleCommand message."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.target_system = target_system
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.from_external = True
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Event publisher
    # ------------------------------------------------------------------ #

    def _publish_safety_event(
        self,
        event_type: str,
        drone_id: str,
        severity: str,
        message: str,
        value: Optional[float],
    ) -> None:
        payload = {
            "event_type": event_type,
            "drone_id": drone_id,
            "severity": severity,
            "message": message,
            "value": value,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._safety_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
EOF
```

> [!NOTE]
> **RTL vs NAV_LAND**: `battery_rtl` uses `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` — the drone flies home first before landing. Emergency stop uses `VEHICLE_CMD_NAV_LAND` — it lands *in place* immediately. Choose the safer behaviour for your mission scenario.

---

## 6.4 `LeadSensorAggregatorNode`

**File:** `major_project/lead_pilot/lead_sensor_aggregator_node.py`

### Design overview

```
/fmu/out/vehicle_local_position  ─┐  DRONE-0 (Lead) position
/px4_1/fmu/out/vehicle_local_position ─┘  DRONE-1 (Wingman) position

/fmu/out/vehicle_status          → arming, flight mode
/fmu/out/battery_status          → bat%
/camera_0/detections             → latest camera detections
/camera_0/obstacle_vector        → latest obstacle vector
/mission/plan                    → mission start trigger
/mission/step_assessment         → mission step completion

Timer 1 Hz ──→ publish /drone_0/situation (String)
                         └── includes wingman_pos line
                         └── proximity check → /safety/event (rate-limited)
```

### Situation block format

```
pos:(x,y) alt:Nm hdg:D° spd:Sm/s bat:B% mode:MODE ARMED gps:OK
camera:CAMERA_SUMMARY
obstacles:OBSTACLE_VECTOR
temporal:elapsed=Mm Ss dist_home=Dm CARDINAL phase=PHASE
wingman_pos:(wx,wy) alt:wNm
```

### Complete implementation

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_sensor_aggregator_node.py
#!/usr/bin/env python3
"""
lead_sensor_aggregator_node.py
Fuses all Lead-drone (DRONE_0) telemetry into a natural-language situation
block published on /drone_0/situation.

Key upgrade: also subscribes to /px4_1/fmu/out/vehicle_local_position
(Wingman position) so the situation block includes a `wingman_pos` line and
proximity warnings are published to /safety/event.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    BatteryStatus,
    VehicleLocalPosition,
    VehicleStatus,
)
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_DETECTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_FLIGHT_MODES: dict[int, str] = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    6: "ACRO",
    8: "STABILIZED",
    10: "OFFBOARD",
    14: "AUTO_LAND",
    17: "AUTO_TAKEOFF",
}

_CARDINALS = [
    "N", "NE", "E", "SE", "S", "SW", "W", "NW",
]

_MIN_SEPARATION_M: float = 5.0
_PROX_WARN_INTERVAL_SEC: float = 5.0


def _bearing_to_cardinal(bearing_deg: float) -> str:
    idx = int((bearing_deg + 22.5) / 45.0) % 8
    return _CARDINALS[idx]


def _flight_mode_name(nav_state: int) -> str:
    return _FLIGHT_MODES.get(nav_state, f"MODE_{nav_state}")


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LeadSensorAggregatorNode(Node):
    """
    Aggregates Lead (DRONE_0) sensor data into a structured situation string
    published on /drone_0/situation at 1 Hz.

    Also subscribes to Wingman local position so the situation block includes
    `wingman_pos` and proximity warnings can be issued.
    """

    def __init__(self) -> None:
        super().__init__("lead_sensor_aggregator_node")

        # ------------------------------------------------------------------ #
        # State – Lead (DRONE_0)
        # ------------------------------------------------------------------ #
        self._pos_x: float = 0.0           # North (m)
        self._pos_y: float = 0.0           # East  (m)
        self._pos_z: float = 0.0           # Down  (m) → alt = -z
        self._heading_deg: float = 0.0     # degrees [0, 360)
        self._speed_ms: float = 0.0        # m/s (horizontal)
        self._battery_pct: float = 100.0
        self._nav_state: int = 4           # default: AUTO_LOITER
        self._arming_state: int = 1        # 1=disarmed, 2=armed
        self._gps_ok: bool = False
        self._vx: float = 0.0
        self._vy: float = 0.0

        # Home position (set on first valid fix)
        self._home_x: Optional[float] = None
        self._home_y: Optional[float] = None
        self._home_set_ts: Optional[float] = None

        # Camera / obstacle state
        self._camera_summary: str = "none"
        self._obstacle_vector: str = "clear"

        # Mission phase
        self._mission_phase: str = "idle"

        # ------------------------------------------------------------------ #
        # State – Wingman (DRONE_1)
        # ------------------------------------------------------------------ #
        self._wingman_pos: Optional[tuple[float, float, float]] = None
        self._last_prox_warn_ts: float = 0.0

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self._situation_pub = self.create_publisher(
            String, "/drone_0/situation", RELIABLE_QOS
        )
        self._safety_pub = self.create_publisher(
            String, "/safety/event", RELIABLE_QOS
        )

        # ------------------------------------------------------------------ #
        # Subscribers – Lead telemetry
        # ------------------------------------------------------------------ #
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._on_lead_position,
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._on_vehicle_status,
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status_v1",
            self._on_battery,
            BEST_EFFORT_QOS,
        )

        # Camera topics (higher depth queue for burst detections)
        self.create_subscription(
            String,
            "/camera_0/detections",
            self._on_detections,
            _DETECTION_QOS,
        )
        self.create_subscription(
            String,
            "/camera_0/obstacle_vector",
            self._on_obstacle_vector,
            _DETECTION_QOS,
        )

        # Mission control topics
        self.create_subscription(
            String,
            "/mission/plan",
            self._on_mission_plan,
            _DETECTION_QOS,
        )
        self.create_subscription(
            String,
            "/mission/step_assessment",
            self._on_step_assessment,
            _DETECTION_QOS,
        )

        # ------------------------------------------------------------------ #
        # Subscriber – Wingman position (NEW — loophole fix M2.2)
        # ------------------------------------------------------------------ #
        self.create_subscription(
            VehicleLocalPosition,
            "/px4_1/fmu/out/vehicle_local_position_v1",
            self._on_wingman_position,
            BEST_EFFORT_QOS,
        )

        # ------------------------------------------------------------------ #
        # 1 Hz publish timer
        # ------------------------------------------------------------------ #
        self.create_timer(1.0, self._publish_situation)

        self.get_logger().info("LeadSensorAggregatorNode started.")

    # ------------------------------------------------------------------ #
    # Callbacks – Lead telemetry
    # ------------------------------------------------------------------ #

    def _on_lead_position(self, msg: VehicleLocalPosition) -> None:
        self._pos_x = float(msg.x)
        self._pos_y = float(msg.y)
        self._pos_z = float(msg.z)
        self._vx = float(msg.vx)
        self._vy = float(msg.vy)
        self._speed_ms = math.sqrt(self._vx ** 2 + self._vy ** 2)

        # Heading from velocity vector (only meaningful when moving)
        if self._speed_ms > 0.1:
            self._heading_deg = math.degrees(math.atan2(self._vy, self._vx)) % 360.0

        # Set home on first valid fix
        if self._home_x is None and msg.xy_valid:
            self._home_x = self._pos_x
            self._home_y = self._pos_y
            self._home_set_ts = time.monotonic()
            self.get_logger().info(
                f"Home position set: ({self._home_x:.1f}, {self._home_y:.1f})"
            )

        self._gps_ok = bool(msg.xy_valid)

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        self._nav_state = int(msg.nav_state)
        self._arming_state = int(msg.arming_state)

    def _on_battery(self, msg: BatteryStatus) -> None:
        self._battery_pct = float(msg.remaining) * 100.0

    def _on_detections(self, msg: String) -> None:
        """Store latest camera detection summary."""
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                label = data.get("label", "unknown")
                count = data.get("count", 1)
                confidence = data.get("confidence", 0.0)
                self._camera_summary = (
                    f"{count}x {label} ({confidence:.0%} conf)"
                )
            else:
                self._camera_summary = msg.data[:80]
        except (json.JSONDecodeError, KeyError):
            self._camera_summary = msg.data[:80]

    def _on_obstacle_vector(self, msg: String) -> None:
        """Store latest obstacle vector description."""
        self._obstacle_vector = msg.data.strip() if msg.data.strip() else "clear"

    def _on_mission_plan(self, msg: String) -> None:
        """Mission started — transition phase to 'active'."""
        self._mission_phase = "active"
        self.get_logger().info("Mission plan received — phase: active")

    def _on_step_assessment(self, msg: String) -> None:
        """Mission step completed — update phase from assessment payload."""
        try:
            data = json.loads(msg.data)
            phase = data.get("phase", "active")
            self._mission_phase = str(phase)
        except (json.JSONDecodeError, KeyError):
            self._mission_phase = "step_complete"

    # ------------------------------------------------------------------ #
    # Callbacks – Wingman position (NEW)
    # ------------------------------------------------------------------ #

    def _on_wingman_position(self, msg: VehicleLocalPosition) -> None:
        """Cache Wingman NED position and check proximity."""
        self._wingman_pos = (float(msg.x), float(msg.y), float(msg.z))
        self._check_proximity()

    # ------------------------------------------------------------------ #
    # Proximity check (rate-limited)
    # ------------------------------------------------------------------ #

    def _check_proximity(self) -> None:
        if self._wingman_pos is None:
            return

        dx = self._pos_x - self._wingman_pos[0]
        dy = self._pos_y - self._wingman_pos[1]
        dz = self._pos_z - self._wingman_pos[2]
        separation = math.sqrt(dx * dx + dy * dy + dz * dz)

        if separation < _MIN_SEPARATION_M:
            now = time.monotonic()
            if now - self._last_prox_warn_ts >= _PROX_WARN_INTERVAL_SEC:
                self._last_prox_warn_ts = now
                self.get_logger().warning(
                    f"Proximity warning from aggregator: {separation:.2f}m separation."
                )
                payload = {
                    "event_type": "proximity_warning",
                    "drone_id": "BOTH",
                    "severity": "warning",
                    "message": (
                        f"Lead aggregator: drones {separation:.2f}m apart "
                        f"(min {_MIN_SEPARATION_M}m)."
                    ),
                    "value": round(separation, 2),
                }
                msg = String()
                msg.data = json.dumps(payload)
                self._safety_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Situation publish
    # ------------------------------------------------------------------ #

    def _publish_situation(self) -> None:
        situation = self._build_situation()
        msg = String()
        msg.data = situation
        self._situation_pub.publish(msg)

    def _build_situation(self) -> str:
        # -- Line 1: primary state ----------------------------------------
        alt_m = -self._pos_z
        mode_str = _flight_mode_name(self._nav_state)
        armed_str = "ARMED" if self._arming_state == 2 else "DISARMED"
        gps_str = "OK" if self._gps_ok else "LOST"

        line1 = (
            f"pos:({self._pos_x:.1f},{self._pos_y:.1f}) "
            f"alt:{alt_m:.1f}m "
            f"hdg:{self._heading_deg:.0f}° "
            f"spd:{self._speed_ms:.1f}m/s "
            f"bat:{self._battery_pct:.0f}% "
            f"mode:{mode_str} {armed_str} "
            f"gps:{gps_str}"
        )

        # -- Line 2: camera -----------------------------------------------
        line2 = f"camera:{self._camera_summary}"

        # -- Line 3: obstacles --------------------------------------------
        line3 = f"obstacles:{self._obstacle_vector}"

        # -- Line 4: temporal ---------------------------------------------
        elapsed_str = "0m 0s"
        dist_home_str = "0m"
        cardinal_str = "N"

        now_mono = time.monotonic()
        if self._home_set_ts is not None:
            elapsed_sec = now_mono - self._home_set_ts
            elapsed_min = int(elapsed_sec // 60)
            elapsed_rem = int(elapsed_sec % 60)
            elapsed_str = f"{elapsed_min}m {elapsed_rem}s"

        if self._home_x is not None and self._home_y is not None:
            dx = self._pos_x - self._home_x
            dy = self._pos_y - self._home_y
            dist = math.sqrt(dx * dx + dy * dy)
            dist_home_str = f"{dist:.0f}m"
            if dist > 0.5:
                bear = math.degrees(math.atan2(dy, dx)) % 360.0
                cardinal_str = _bearing_to_cardinal(bear)

        line4 = (
            f"temporal:elapsed={elapsed_str} "
            f"dist_home={dist_home_str} {cardinal_str} "
            f"phase={self._mission_phase}"
        )

        # -- Line 5: wingman position (NEW) --------------------------------
        if self._wingman_pos is not None:
            wx, wy, wz = self._wingman_pos
            walt = -wz
            line5 = f"wingman_pos:({wx:.1f},{wy:.1f}) alt:{walt:.1f}m"
        else:
            line5 = "wingman_pos:unknown"

        return "\n".join([line1, line2, line3, line4, line5])


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeadSensorAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
EOF
```

---

## 6.5 `WingmanSensorAggregatorNode`

**File:** `major_project/wingman_pilot/wingman_sensor_aggregator_node.py`

### Situation block format (Wingman)

```
pos:(x,y) alt:Nm hdg:D° spd:Sm/s bat:B% mode:MODE ARMED gps:OK
camera:CAMERA_SUMMARY
obstacles:OBSTACLE_VECTOR
temporal:elapsed=Mm Ss dist_home=Dm CARDINAL phase=PHASE
```

> [!NOTE]
> The Wingman aggregator does **not** include a `wingman_pos` line — that line in the Lead's situation refers to the Wingman's position as seen by the Lead. The Wingman publishes its own full state in `/drone_1/situation`.

### Complete implementation

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_sensor_aggregator_node.py
#!/usr/bin/env python3
"""
wingman_sensor_aggregator_node.py
Fuses Wingman (DRONE_1) telemetry into a structured situation string
published on /drone_1/situation at 1 Hz.

Subscribes to /px4_1/fmu/out/... topics.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    BatteryStatus,
    VehicleLocalPosition,
    VehicleStatus,
)
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_DETECTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_FLIGHT_MODES: dict[int, str] = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    6: "ACRO",
    8: "STABILIZED",
    10: "OFFBOARD",
    14: "AUTO_LAND",
    17: "AUTO_TAKEOFF",
}

_CARDINALS = [
    "N", "NE", "E", "SE", "S", "SW", "W", "NW",
]


def _bearing_to_cardinal(bearing_deg: float) -> str:
    idx = int((bearing_deg + 22.5) / 45.0) % 8
    return _CARDINALS[idx]


def _flight_mode_name(nav_state: int) -> str:
    return _FLIGHT_MODES.get(nav_state, f"MODE_{nav_state}")


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class WingmanSensorAggregatorNode(Node):
    """
    Aggregates Wingman (DRONE_1) sensor data into a structured situation string
    published on /drone_1/situation at 1 Hz.
    """

    def __init__(self) -> None:
        super().__init__("wingman_sensor_aggregator_node")

        # ------------------------------------------------------------------ #
        # State
        # ------------------------------------------------------------------ #
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0
        self._pos_z: float = 0.0
        self._heading_deg: float = 0.0
        self._speed_ms: float = 0.0
        self._battery_pct: float = 100.0
        self._nav_state: int = 4
        self._arming_state: int = 1
        self._gps_ok: bool = False
        self._vx: float = 0.0
        self._vy: float = 0.0

        # Home position
        self._home_x: Optional[float] = None
        self._home_y: Optional[float] = None
        self._home_set_ts: Optional[float] = None

        # Camera / obstacle state
        self._camera_summary: str = "none"
        self._obstacle_vector: str = "clear"

        # Mission phase
        self._mission_phase: str = "idle"

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self._situation_pub = self.create_publisher(
            String, "/drone_1/situation", RELIABLE_QOS
        )

        # ------------------------------------------------------------------ #
        # Subscribers – Wingman telemetry (/px4_1/ namespace)
        # ------------------------------------------------------------------ #
        self.create_subscription(
            VehicleLocalPosition,
            "/px4_1/fmu/out/vehicle_local_position",
            self._on_position,
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            VehicleStatus,
            "/px4_1/fmu/out/vehicle_status_v1",
            self._on_vehicle_status,
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            BatteryStatus,
            "/px4_1/fmu/out/battery_status_v1",
            self._on_battery,
            BEST_EFFORT_QOS,
        )

        # Camera topics
        self.create_subscription(
            String,
            "/camera_1/detections",
            self._on_detections,
            _DETECTION_QOS,
        )
        self.create_subscription(
            String,
            "/camera_1/obstacle_vector",
            self._on_obstacle_vector,
            _DETECTION_QOS,
        )

        # Mission control topics (both drones receive the same mission)
        self.create_subscription(
            String,
            "/mission/plan",
            self._on_mission_plan,
            _DETECTION_QOS,
        )
        self.create_subscription(
            String,
            "/mission/step_assessment",
            self._on_step_assessment,
            _DETECTION_QOS,
        )

        # ------------------------------------------------------------------ #
        # 1 Hz publish timer
        # ------------------------------------------------------------------ #
        self.create_timer(1.0, self._publish_situation)

        self.get_logger().info("WingmanSensorAggregatorNode started.")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        self._pos_x = float(msg.x)
        self._pos_y = float(msg.y)
        self._pos_z = float(msg.z)
        self._vx = float(msg.vx)
        self._vy = float(msg.vy)
        self._speed_ms = math.sqrt(self._vx ** 2 + self._vy ** 2)

        if self._speed_ms > 0.1:
            self._heading_deg = math.degrees(math.atan2(self._vy, self._vx)) % 360.0

        if self._home_x is None and msg.xy_valid:
            self._home_x = self._pos_x
            self._home_y = self._pos_y
            self._home_set_ts = time.monotonic()
            self.get_logger().info(
                f"Wingman home set: ({self._home_x:.1f}, {self._home_y:.1f})"
            )

        self._gps_ok = bool(msg.xy_valid)

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        self._nav_state = int(msg.nav_state)
        self._arming_state = int(msg.arming_state)

    def _on_battery(self, msg: BatteryStatus) -> None:
        self._battery_pct = float(msg.remaining) * 100.0

    def _on_detections(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                label = data.get("label", "unknown")
                count = data.get("count", 1)
                confidence = data.get("confidence", 0.0)
                self._camera_summary = (
                    f"{count}x {label} ({confidence:.0%} conf)"
                )
            else:
                self._camera_summary = msg.data[:80]
        except (json.JSONDecodeError, KeyError):
            self._camera_summary = msg.data[:80]

    def _on_obstacle_vector(self, msg: String) -> None:
        self._obstacle_vector = msg.data.strip() if msg.data.strip() else "clear"

    def _on_mission_plan(self, msg: String) -> None:
        self._mission_phase = "active"
        self.get_logger().info("Wingman: mission plan received — phase: active")

    def _on_step_assessment(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            phase = data.get("phase", "active")
            self._mission_phase = str(phase)
        except (json.JSONDecodeError, KeyError):
            self._mission_phase = "step_complete"

    # ------------------------------------------------------------------ #
    # Situation publish
    # ------------------------------------------------------------------ #

    def _publish_situation(self) -> None:
        situation = self._build_situation()
        msg = String()
        msg.data = situation
        self._situation_pub.publish(msg)

    def _build_situation(self) -> str:
        # -- Line 1: primary state ----------------------------------------
        alt_m = -self._pos_z
        mode_str = _flight_mode_name(self._nav_state)
        armed_str = "ARMED" if self._arming_state == 2 else "DISARMED"
        gps_str = "OK" if self._gps_ok else "LOST"

        line1 = (
            f"pos:({self._pos_x:.1f},{self._pos_y:.1f}) "
            f"alt:{alt_m:.1f}m "
            f"hdg:{self._heading_deg:.0f}° "
            f"spd:{self._speed_ms:.1f}m/s "
            f"bat:{self._battery_pct:.0f}% "
            f"mode:{mode_str} {armed_str} "
            f"gps:{gps_str}"
        )

        # -- Line 2: camera -----------------------------------------------
        line2 = f"camera:{self._camera_summary}"

        # -- Line 3: obstacles --------------------------------------------
        line3 = f"obstacles:{self._obstacle_vector}"

        # -- Line 4: temporal ---------------------------------------------
        elapsed_str = "0m 0s"
        dist_home_str = "0m"
        cardinal_str = "N"

        if self._home_set_ts is not None:
            elapsed_sec = time.monotonic() - self._home_set_ts
            elapsed_min = int(elapsed_sec // 60)
            elapsed_rem = int(elapsed_sec % 60)
            elapsed_str = f"{elapsed_min}m {elapsed_rem}s"

        if self._home_x is not None and self._home_y is not None:
            dx = self._pos_x - self._home_x
            dy = self._pos_y - self._home_y
            dist = math.sqrt(dx * dx + dy * dy)
            dist_home_str = f"{dist:.0f}m"
            if dist > 0.5:
                bear = math.degrees(math.atan2(dy, dx)) % 360.0
                cardinal_str = _bearing_to_cardinal(bear)

        line4 = (
            f"temporal:elapsed={elapsed_str} "
            f"dist_home={dist_home_str} {cardinal_str} "
            f"phase={self._mission_phase}"
        )

        return "\n".join([line1, line2, line3, line4])


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WingmanSensorAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
EOF
```

---

## 6.6 Build and Verification

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install 2>&1 | tail -5
source install/setup.bash

# Verify entry points registered
ros2 pkg executables major_project | grep -E "aggregator|safety"
# Expected:
#   major_project lead_sensor_aggregator
#   major_project safety_monitor
#   major_project wingman_sensor_aggregator
```

## 6.7 Topic & QoS Summary

| Topic | Message Type | QoS | Direction |
|-------|-------------|-----|-----------|
| `/fmu/out/battery_status` | `BatteryStatus` | BEST_EFFORT | → SafetyMonitor, LeadAggregator |
| `/px4_1/fmu/out/battery_status` | `BatteryStatus` | BEST_EFFORT | → SafetyMonitor, WingmanAggregator |
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | BEST_EFFORT | → SafetyMonitor, LeadAggregator |
| `/px4_1/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | BEST_EFFORT | → SafetyMonitor, LeadAggregator (wingman_pos), WingmanAggregator |
| `/fmu/out/vehicle_status` | `VehicleStatus` | BEST_EFFORT | → SafetyMonitor, LeadAggregator |
| `/px4_1/fmu/out/vehicle_status` | `VehicleStatus` | BEST_EFFORT | → SafetyMonitor, WingmanAggregator |
| `/emergency_stop` | `Bool` | RELIABLE | → SafetyMonitor |
| `/safety/event` | `String (JSON)` | RELIABLE | SafetyMonitor → SLM agents |
| `/drone_0/situation` | `String` | RELIABLE | LeadAggregator → Lead SLM |
| `/drone_1/situation` | `String` | RELIABLE | WingmanAggregator → Wingman SLM |
| `/fmu/in/vehicle_command` | `VehicleCommand` | RELIABLE | SafetyMonitor → PX4 (Drone-0) |
| `/px4_1/fmu/in/vehicle_command` | `VehicleCommand` | RELIABLE | SafetyMonitor → PX4 (Drone-1) |

---

## 6.7 Integration Checklist

- [ ] `safety_monitor_node` is launched **first** (or simultaneously) so it catches early faults
- [ ] Both `target_system` values (`1` for Drone-0, `2` for Drone-1) match your PX4 `MAV_SYS_ID` params
- [ ] `/emergency_stop` is on RELIABLE QoS on **both** publisher and subscriber sides
- [ ] `lead_sensor_aggregator` is running **before** the Lead SLM agent so `/drone_0/situation` is populated on first LLM call
- [ ] Camera nodes publish JSON to `/camera_0/detections` matching `{"label": "...", "count": N, "confidence": 0.X}`
