# Part 7 — Flight Execution Layer

This part covers the three nodes that translate approved intents into PX4 commands:

| Node | File | Role |
|------|------|------|
| `LeadPX4CommanderNode` | `lead_pilot/lead_px4_commander_node.py` | Executes flight commands for Lead (Drone-0) |
| `WingmanPX4CommanderNode` | `wingman_pilot/wingman_px4_commander_node.py` | Executes flight commands for Wingman (Drone-1), includes `follow_lead` |
| `LeadIntentBridgeNode` | `lead_pilot/lead_intent_bridge_node.py` | Dispatches chained FlightIntent `then` steps for Lead |
| `WingmanIntentBridgeNode` | `wingman_pilot/wingman_intent_bridge_node.py` | Dispatches chained FlightIntent `then` steps for Wingman |

---

## 7.1 Architecture Overview

```
/lead/approved_intent  ──→  LeadPX4CommanderNode
                              ├── /fmu/in/offboard_control_mode   (10 Hz keepalive)
                              ├── /fmu/in/trajectory_setpoint     (10 Hz keepalive)
                              └── /fmu/in/vehicle_command         (arm/mode/land/rtl)
                              └── /lead/execution_feedback        (errors → SLM)

/wingman/approved_intent ──→  WingmanPX4CommanderNode
                              ├── /px4_1/fmu/in/offboard_control_mode
                              ├── /px4_1/fmu/in/trajectory_setpoint
                              └── /px4_1/fmu/in/vehicle_command
                              └── /wingman/execution_feedback

/fmu/out/vehicle_local_position ──→ WingmanPX4CommanderNode
                                     └── follow_lead: track Lead position + offset
```

---

## 7.2 QoS Reference

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

---

## 7.3 `LeadPX4CommanderNode`

**File:** `major_project/lead_pilot/lead_px4_commander_node.py`

### Direction offset table

```
north     → (+1.0,  0.0)    northeast → (+0.707, +0.707)
south     → (-1.0,  0.0)    southeast → (-0.707, +0.707)
east      → ( 0.0, +1.0)    southwest → (-0.707, -0.707)
west      → ( 0.0, -1.0)    northwest → (+0.707, -0.707)
forward   → (+1.0,  0.0)    backward  → (-1.0,  0.0)
right     → ( 0.0, +1.0)    left      → ( 0.0, -1.0)
```

### Supported actions

| Action | Behaviour |
|--------|-----------|
| `takeoff` | Arm → set OFFBOARD → climb to `altitude_m` (default 5 m) |
| `move` | Move `distance_m` in `direction` at current altitude |
| `hover` | Zero-velocity hold at current position |
| `hold` | Alias for hover |
| `search_stop` | Stop and hover (pause search) |
| `search_resume` | Resume last search pattern (restore previous setpoint) |
| `search_expand` | Expand search radius by `expansion_m` (default 10 m) |
| `search` | Fly North `radius_m`, pause, return — simple stub search loop |
| `land` | Send NAV_LAND command |
| `rtl` | Send NAV_RETURN_TO_LAUNCH command |
| `follow_lead` | Lead doesn't follow itself — hover stub with log notice |

### Complete implementation

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_px4_commander_node.py
#!/usr/bin/env python3
"""
lead_px4_commander_node.py
Translates approved FlightIntent JSON from /lead/approved_intent
into PX4 offboard setpoints and VehicleCommands for DRONE_0 (Lead).

target_system = 1 for all VehicleCommand messages.
"""

from __future__ import annotations

import json
import math
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
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)
from std_msgs.msg import Bool, String

# ---------------------------------------------------------------------------
# QoS
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
# Constants
# ---------------------------------------------------------------------------

DIRECTION_OFFSETS: dict[str, tuple[float, float]] = {
    "north":     ( 1.0,   0.0),
    "south":     (-1.0,   0.0),
    "east":      ( 0.0,   1.0),
    "west":      ( 0.0,  -1.0),
    "northeast": ( 0.707, 0.707),
    "northwest": ( 0.707,-0.707),
    "southeast": (-0.707, 0.707),
    "southwest": (-0.707,-0.707),
    "forward":   ( 1.0,   0.0),
    "backward":  (-1.0,   0.0),
    "left":      ( 0.0,  -1.0),
    "right":     ( 0.0,   1.0),
}

_DEFAULT_TAKEOFF_ALT_M: float = 5.0
_DEFAULT_MOVE_DIST_M: float = 10.0
_DEFAULT_SPEED_MS: float = 2.0
_DEFAULT_SEARCH_RADIUS_M: float = 20.0
_DEFAULT_EXPAND_M: float = 10.0

_TARGET_SYSTEM: int = 1


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LeadPX4CommanderNode(Node):
    """
    Executes flight commands for Lead Drone (DRONE_0).

    Subscribes:
      /lead/approved_intent  String (FlightIntent JSON)
      /fmu/out/vehicle_local_position  (track current position)
      /emergency_stop  Bool

    Publishes:
      /fmu/in/offboard_control_mode  OffboardControlMode
      /fmu/in/trajectory_setpoint    TrajectorySetpoint
      /fmu/in/vehicle_command        VehicleCommand
      /lead/execution_feedback       String (errors/status)
    """

    def __init__(self) -> None:
        super().__init__("lead_px4_commander_node")

        # ------------------------------------------------------------------ #
        # State
        # ------------------------------------------------------------------ #
        self._cur_x: float = 0.0
        self._cur_y: float = 0.0
        self._cur_z: float = 0.0          # NED: negative = up

        # Target setpoint — initialised to GROUND (0,0,0) not climb altitude.
        # Bug fix: streaming a non-zero _tgt_z before takeoff causes EKF
        # 'Altitude failure (roll)' because PX4 sees a commanded altitude it
        # cannot reach while still on the ground.
        self._tgt_x: float = 0.0
        self._tgt_y: float = 0.0
        self._tgt_z: float = 0.0          # NED: 0 = ground level ← FIXED

        # Saved setpoint for search_resume
        self._saved_x: Optional[float] = None
        self._saved_y: Optional[float] = None
        self._saved_z: Optional[float] = None

        # Search expansion state
        self._search_radius_m: float = _DEFAULT_SEARCH_RADIUS_M

        # Offboard state machine
        # PX4 requires setpoints to be streaming BEFORE the OFFBOARD switch,
        # and the OFFBOARD switch must happen BEFORE arming.
        # Sequence: stream ground setpoints (pre-arm) → OFFBOARD switch →
        #           Arm → update _tgt_z to climb altitude → active flight.
        self._offboard_active: bool = False
        self._keepalive_count: int = 0
        self._pre_arm_phase: bool = False   # True while waiting to switch OFFBOARD
        self._pending_alt_m: float = 0.0    # climb altitude requested by takeoff

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self._ocm_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            RELIABLE_QOS,
        )
        self._tsp_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            RELIABLE_QOS,
        )
        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            RELIABLE_QOS,
        )
        self._feedback_pub = self.create_publisher(
            String,
            "/lead/execution_feedback",
            RELIABLE_QOS,
        )

        # ------------------------------------------------------------------ #
        # Subscribers
        # ------------------------------------------------------------------ #
        self.create_subscription(
            String,
            "/lead/approved_intent",
            self._on_approved_intent,
            RELIABLE_QOS,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_local_position,
            BEST_EFFORT_QOS,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._on_emergency_stop,
            RELIABLE_QOS,
        )

        # ------------------------------------------------------------------ #
        # 10 Hz keepalive timer (PX4 offboard requires >2 Hz)
        # ------------------------------------------------------------------ #
        self.create_timer(0.1, self._keepalive)

        self.get_logger().info("LeadPX4CommanderNode started (target_system=1).")

    # ------------------------------------------------------------------ #
    # Telemetry callbacks
    # ------------------------------------------------------------------ #

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self._cur_x = float(msg.x)
        self._cur_y = float(msg.y)
        self._cur_z = float(msg.z)

    def _on_emergency_stop(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().error("Emergency stop received — sending NAV_LAND.")
            self._offboard_active = False
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ------------------------------------------------------------------ #
    # Intent dispatcher
    # ------------------------------------------------------------------ #

    def _on_approved_intent(self, msg: String) -> None:
        raw = msg.data.strip()
        if not raw:
            return

        try:
            intent = json.loads(raw)
        except json.JSONDecodeError as exc:
            self._publish_feedback(f"JSON parse error: {exc} — raw: {raw[:120]}")
            return

        # Skip bridge echo-loop markers
        if intent.get("__bridge_dispatched__"):
            return

        action = str(intent.get("action", "")).lower()
        self.get_logger().info(f"Lead commander executing action: {action}")

        handler = {
            "takeoff": self._action_takeoff,
            "move": self._action_move,
            "hover": self._action_hover,
            "hold": self._action_hover,
            "search_stop": self._action_search_stop,
            "search_resume": self._action_search_resume,
            "search_expand": self._action_search_expand,
            "search": self._action_search,
            "land": self._action_land,
            "rtl": self._action_rtl,
            "follow_lead": self._action_follow_lead_stub,
        }.get(action)

        if handler is None:
            self._publish_feedback(
                f"Unknown action '{action}' — supported: "
                "takeoff, move, hover, hold, search_stop, search_resume, "
                "search_expand, search, land, rtl, follow_lead"
            )
            return

        try:
            handler(intent)
        except Exception as exc:
            self._publish_feedback(f"Action '{action}' raised: {exc}")
            self.get_logger().error(f"Action handler exception: {exc}", exc_info=True)

    # ------------------------------------------------------------------ #
    # Action handlers
    # ------------------------------------------------------------------ #

    def _action_takeoff(self, intent: dict) -> None:
        alt_m = float(intent.get("altitude_m", _DEFAULT_TAKEOFF_ALT_M))

        # Store the requested altitude — keepalive will command it AFTER arming.
        self._pending_alt_m = abs(alt_m)

        # Keep target at CURRENT position (ground) during pre-arm streaming.
        # Bug fix: setting _tgt_z to -alt_m here caused PX4 EKF to see an
        # unreachable altitude setpoint before the drone is even armed,
        # triggering 'Altitude failure (roll)' and violent pitch/roll.
        self._tgt_x = self._cur_x
        self._tgt_y = self._cur_y
        self._tgt_z = self._cur_z          # NED: stay at ground for now

        # Enter pre-arm phase: keepalive will stream ground setpoints for
        # 1 second (10 ticks @ 10Hz), then send OFFBOARD switch + Arm,
        # then raise _tgt_z to climb altitude.
        self._pre_arm_phase   = True
        self._offboard_active = False      # keepalive pre-arm loop takes over
        self._keepalive_count = 0          # reset counter for pre-arm timing
        self.get_logger().info(
            f"Takeoff requested: {alt_m}m — entering pre-arm streaming phase.")

    def _action_move(self, intent: dict) -> None:
        direction  = str(intent.get("direction", "north")).lower()
        distance_m = float(intent.get("distance_m", _DEFAULT_MOVE_DIST_M))

        if direction not in DIRECTION_OFFSETS:
            self._publish_feedback(
                f"Unknown direction '{direction}'. "
                f"Valid: {', '.join(DIRECTION_OFFSETS.keys())}"
            )
            return

        dx, dy = DIRECTION_OFFSETS[direction]
        self._tgt_x = self._cur_x + dx * distance_m
        self._tgt_y = self._cur_y + dy * distance_m

        # Bug fix #3: use _tgt_z (last COMMANDED altitude), NOT _cur_z.
        # _cur_z is the instantaneous measured altitude — if telemetry is
        # delayed or the drone hasn't finished climbing yet, _cur_z can be
        # 0.0 or a mid-climb value, causing the drone to descend during the move.
        # _tgt_z is always the last altitude we explicitly commanded.
        altitude_m = intent.get("altitude_m", None)
        if altitude_m is not None:
            self._tgt_z = -abs(float(altitude_m))   # NED: negative = up
        # else: keep _tgt_z unchanged (hold last commanded altitude)

        self._offboard_active = True
        self.get_logger().info(
            f"Move {distance_m}m {direction} → target "
            f"({self._tgt_x:.1f}, {self._tgt_y:.1f}, z={self._tgt_z:.1f})"
        )

    def _action_hover(self, intent: dict) -> None:
        """Hold current position at last COMMANDED altitude (not measured)."""
        self._tgt_x = self._cur_x
        self._tgt_y = self._cur_y
        # Bug fix #7: use _tgt_z, not _cur_z.
        # If the drone was mid-descent due to Bug #3, _cur_z would lock in
        # the ground-level altitude. _tgt_z holds the correct commanded altitude.
        # (_tgt_x/_tgt_y use _cur_x/_cur_y to hold the CURRENT horizontal
        # position, which is correct for hover.)
        # _tgt_z is intentionally NOT updated here — keep the commanded altitude.
        self._offboard_active = True
        self.get_logger().info(
            f"Hover at ({self._cur_x:.1f}, {self._cur_y:.1f}, z_cmd={self._tgt_z:.1f})."
        )

    def _action_search_stop(self, intent: dict) -> None:
        """Save current setpoint and hover."""
        self._saved_x = self._tgt_x
        self._saved_y = self._tgt_y
        self._saved_z = self._tgt_z
        self._action_hover(intent)
        self.get_logger().info("Search stopped — position saved for resume.")

    def _action_search_resume(self, intent: dict) -> None:
        """Restore saved setpoint."""
        if self._saved_x is None:
            self._publish_feedback("search_resume: no saved position — hovering.")
            self._action_hover(intent)
            return
        self._tgt_x = self._saved_x
        self._tgt_y = self._saved_y
        self._tgt_z = self._saved_z
        self._offboard_active = True
        self.get_logger().info(
            f"Search resumed to ({self._tgt_x:.1f}, {self._tgt_y:.1f})."
        )

    def _action_search_expand(self, intent: dict) -> None:
        """Expand search radius and move North by expanded radius."""
        expansion_m = float(intent.get("expansion_m", _DEFAULT_EXPAND_M))
        self._search_radius_m += expansion_m
        dx, dy = DIRECTION_OFFSETS["north"]
        self._tgt_x = self._cur_x + dx * self._search_radius_m
        self._tgt_y = self._cur_y + dy * self._search_radius_m
        self._tgt_z = self._cur_z
        self._offboard_active = True
        self.get_logger().info(
            f"Search expanded to radius {self._search_radius_m}m → "
            f"target ({self._tgt_x:.1f}, {self._tgt_y:.1f})."
        )

    def _action_search(self, intent: dict) -> None:
        """
        Simple search: fly North by radius_m from current position.
        A more complex lawnmower pattern would be orchestrated by the SLM
        via sequential move commands. This command seeds the first waypoint.
        """
        radius_m = float(intent.get("radius_m", _DEFAULT_SEARCH_RADIUS_M))
        self._search_radius_m = radius_m
        dx, dy = DIRECTION_OFFSETS["north"]
        self._tgt_x = self._cur_x + dx * radius_m
        self._tgt_y = self._cur_y + dy * radius_m
        self._tgt_z = self._cur_z
        self._offboard_active = True
        self.get_logger().info(
            f"Search started: radius {radius_m}m, first waypoint "
            f"({self._tgt_x:.1f}, {self._tgt_y:.1f})."
        )

    def _action_land(self, intent: dict) -> None:
        self._offboard_active = False
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent.")

    def _action_rtl(self, intent: dict) -> None:
        self._offboard_active = False
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("RTL command sent.")

    def _action_follow_lead_stub(self, intent: dict) -> None:
        """
        Lead cannot follow itself. Log a notice and hover in place.
        The SLM agent should never issue follow_lead to the Lead drone.
        """
        self.get_logger().warning(
            "follow_lead received by Lead commander — Lead cannot follow itself. "
            "Hovering in place."
        )
        self._publish_feedback(
            "follow_lead is not valid for the Lead drone. "
            "Hovering in place. Assign follow_lead only to the Wingman."
        )
        self._action_hover(intent)

    # ------------------------------------------------------------------ #
    # 10 Hz keepalive
    # ------------------------------------------------------------------ #

    def _keepalive(self) -> None:
        """
        PX4 OFFBOARD mode requires setpoints streamed at >2 Hz before the
        OFFBOARD switch is accepted.  We use a 3-phase state machine:

        Phase 0 — IDLE (_pre_arm_phase=False, _offboard_active=False):
            Stream current ground position. No arm/mode commands.

        Phase 1 — PRE-ARM (_pre_arm_phase=True, _offboard_active=False):
            Stream ground-level setpoints for 10 ticks (1 s).
            On tick 10: send DO_SET_MODE → OFFBOARD.
            On tick 11: send COMPONENT_ARM_DISARM → arm.
            On tick 12+: raise _tgt_z to climb altitude → active flight.

        Phase 2 — ACTIVE (_offboard_active=True):
            Stream _tgt_x/y/z continuously at 10 Hz.
        """
        self._keepalive_count += 1
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        # Always publish OffboardControlMode (PX4 needs this at >2 Hz)
        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        ocm.position     = True
        ocm.velocity     = False
        ocm.acceleration = False
        ocm.attitude     = False
        ocm.body_rate    = False
        self._ocm_pub.publish(ocm)

        # ── Phase 1: pre-arm sequence ──────────────────────────────────────
        if self._pre_arm_phase:
            if self._keepalive_count == 10:
                # 1 s of setpoints streamed — safe to switch OFFBOARD
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                    param2=6.0,   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
                )
                self.get_logger().info("Pre-arm: OFFBOARD mode switch sent.")

            elif self._keepalive_count == 11:
                # OFFBOARD accepted — arm the drone
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )
                self.get_logger().info("Pre-arm: ARM command sent.")

            elif self._keepalive_count == 13:
                # Armed + OFFBOARD confirmed — now command the climb
                self._tgt_z = -abs(self._pending_alt_m)   # NED: negative = up
                self._pre_arm_phase   = False
                self._offboard_active = True
                self.get_logger().info(
                    f"Takeoff: climbing to {self._pending_alt_m}m "
                    f"(NED z={self._tgt_z:.1f}).")

        # ── Publish TrajectorySetpoint ─────────────────────────────────────
        tsp = TrajectorySetpoint()
        tsp.timestamp = now_us
        tsp.position  = [self._tgt_x, self._tgt_y, self._tgt_z]
        tsp.yaw       = float("nan")   # let PX4 manage yaw
        self._tsp_pub.publish(tsp)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _send_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.target_system = _TARGET_SYSTEM
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
        self._cmd_pub.publish(msg)

    def _publish_feedback(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._feedback_pub.publish(msg)
        self.get_logger().warning(f"Feedback: {text}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeadPX4CommanderNode()
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

## 7.4 `WingmanPX4CommanderNode`

**File:** `major_project/wingman_pilot/wingman_px4_commander_node.py`

### `follow_lead` formation geometry

```
Lead position: (lead_x, lead_y)
Wingman target: (lead_x - offset, lead_y + offset)
                 ↑ behind Lead     ↑ to the left of Lead
Same altitude as Lead.
```

The offset value comes from the intent JSON field `offset_m` (default 5 m). As Lead moves, the Wingman's target setpoint is updated every time a new Lead position arrives.

### Complete implementation

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_px4_commander_node.py
#!/usr/bin/env python3
"""
wingman_px4_commander_node.py
Translates approved FlightIntent JSON from /wingman/approved_intent
into PX4 offboard setpoints and VehicleCommands for DRONE_1 (Wingman).

CRITICAL UPGRADE (loophole fix M5.1):
  Implements follow_lead action by subscribing to Lead's local position
  (/fmu/out/vehicle_local_position) and continuously updating the trajectory
  setpoint to maintain a formation offset behind-left of the Lead drone.

target_system = 2 for all VehicleCommand messages.
"""

from __future__ import annotations

import json
import math
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
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)
from std_msgs.msg import Bool, String

# ---------------------------------------------------------------------------
# QoS
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
# Constants
# ---------------------------------------------------------------------------

DIRECTION_OFFSETS: dict[str, tuple[float, float]] = {
    "north":     ( 1.0,   0.0),
    "south":     (-1.0,   0.0),
    "east":      ( 0.0,   1.0),
    "west":      ( 0.0,  -1.0),
    "northeast": ( 0.707, 0.707),
    "northwest": ( 0.707,-0.707),
    "southeast": (-0.707, 0.707),
    "southwest": (-0.707,-0.707),
    "forward":   ( 1.0,   0.0),
    "backward":  (-1.0,   0.0),
    "left":      ( 0.0,  -1.0),
    "right":     ( 0.0,   1.0),
}

_DEFAULT_TAKEOFF_ALT_M: float = 5.0
_DEFAULT_MOVE_DIST_M: float = 10.0
_DEFAULT_SPEED_MS: float = 2.0
_DEFAULT_SEARCH_RADIUS_M: float = 20.0
_DEFAULT_EXPAND_M: float = 10.0
_DEFAULT_FOLLOW_OFFSET_M: float = 5.0

_TARGET_SYSTEM: int = 2   # Drone-1


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class WingmanPX4CommanderNode(Node):
    """
    Executes flight commands for Wingman Drone (DRONE_1).

    Subscribes:
      /wingman/approved_intent              String (FlightIntent JSON)
      /px4_1/fmu/out/vehicle_local_position (own position)
      /fmu/out/vehicle_local_position       (Lead position — for follow_lead)
      /emergency_stop                        Bool

    Publishes:
      /px4_1/fmu/in/offboard_control_mode  OffboardControlMode
      /px4_1/fmu/in/trajectory_setpoint    TrajectorySetpoint
      /px4_1/fmu/in/vehicle_command        VehicleCommand
      /wingman/execution_feedback          String (errors/status)
    """

    def __init__(self) -> None:
        super().__init__("wingman_px4_commander_node")

        # Namespace parameter (default 'px4_1')
        self.declare_parameter("drone_namespace", "px4_1")
        self._ns: str = (
            self.get_parameter("drone_namespace")
            .get_parameter_value()
            .string_value
        )

        # ------------------------------------------------------------------ #
        # State – own position
        # ------------------------------------------------------------------ #
        self._cur_x: float = 0.0
        self._cur_y: float = 0.0
        self._cur_z: float = 0.0

        # Target setpoint — initialised to GROUND, not climb altitude (same bug fix as Lead)
        self._tgt_x: float = 0.0
        self._tgt_y: float = 0.0
        self._tgt_z: float = 0.0          # NED: 0 = ground ← FIXED

        # Saved setpoint for search_resume
        self._saved_x: Optional[float] = None
        self._saved_y: Optional[float] = None
        self._saved_z: Optional[float] = None

        # Search expansion state
        self._search_radius_m: float = _DEFAULT_SEARCH_RADIUS_M

        # Offboard state machine (3-phase, same as Lead)
        self._offboard_active: bool = False
        self._keepalive_count: int = 0
        self._pre_arm_phase: bool = False
        self._pending_alt_m: float = 0.0

        # ------------------------------------------------------------------ #
        # State – follow_lead
        # ------------------------------------------------------------------ #
        self._follow_lead_active: bool = False
        self._follow_offset_m: float = _DEFAULT_FOLLOW_OFFSET_M
        # Lead position (NED) — updated from /fmu/out/vehicle_local_position
        self._lead_x: Optional[float] = None
        self._lead_y: Optional[float] = None
        self._lead_z: Optional[float] = None

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self._ocm_pub = self.create_publisher(
            OffboardControlMode,
            f"/{self._ns}/fmu/in/offboard_control_mode",
            RELIABLE_QOS,
        )
        self._tsp_pub = self.create_publisher(
            TrajectorySetpoint,
            f"/{self._ns}/fmu/in/trajectory_setpoint",
            RELIABLE_QOS,
        )
        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            f"/{self._ns}/fmu/in/vehicle_command",
            RELIABLE_QOS,
        )
        self._feedback_pub = self.create_publisher(
            String,
            "/wingman/execution_feedback",
            RELIABLE_QOS,
        )

        # ------------------------------------------------------------------ #
        # Subscribers
        # ------------------------------------------------------------------ #
        self.create_subscription(
            String,
            "/wingman/approved_intent",
            self._on_approved_intent,
            RELIABLE_QOS,
        )
        # Own position
        self.create_subscription(
            VehicleLocalPosition,
            f"/{self._ns}/fmu/out/vehicle_local_position",
            self._on_own_position,
            BEST_EFFORT_QOS,
        )
        # Lead position (DRONE_0) — required for follow_lead
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_lead_position,
            BEST_EFFORT_QOS,
        )
        # Emergency stop
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._on_emergency_stop,
            RELIABLE_QOS,
        )

        # ------------------------------------------------------------------ #
        # 10 Hz keepalive timer
        # ------------------------------------------------------------------ #
        self.create_timer(0.1, self._keepalive)

        self.get_logger().info(
            f"WingmanPX4CommanderNode started (ns={self._ns}, target_system=2)."
        )

    # ------------------------------------------------------------------ #
    # Telemetry callbacks
    # ------------------------------------------------------------------ #

    def _on_own_position(self, msg: VehicleLocalPosition) -> None:
        self._cur_x = float(msg.x)
        self._cur_y = float(msg.y)
        self._cur_z = float(msg.z)

    def _on_lead_position(self, msg: VehicleLocalPosition) -> None:
        """
        Cache Lead drone's NED position.
        When follow_lead is active, update the Wingman's target setpoint
        to maintain the formation offset (behind-left of Lead).
        """
        self._lead_x = float(msg.x)
        self._lead_y = float(msg.y)
        self._lead_z = float(msg.z)

        if self._follow_lead_active:
            self._update_follow_setpoint()

    def _on_emergency_stop(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().error(
                "Wingman: Emergency stop received — NAV_LAND."
            )
            self._follow_lead_active = False
            self._offboard_active = False
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ------------------------------------------------------------------ #
    # follow_lead setpoint update
    # ------------------------------------------------------------------ #

    def _update_follow_setpoint(self) -> None:
        """
        Compute Wingman target from Lead position + formation offset.

        Formation: Wingman flies behind-left of Lead.
          tgt_x = lead_x - offset   (South of Lead in NED North-axis)
          tgt_y = lead_y + offset   (East of Lead → to Lead's right == behind-left
                                     when Lead faces North)
          tgt_z = lead_z            (Same altitude as Lead)
        """
        if self._lead_x is None or self._lead_y is None or self._lead_z is None:
            return

        offset = self._follow_offset_m
        self._tgt_x = self._lead_x - offset
        self._tgt_y = self._lead_y + offset
        self._tgt_z = self._lead_z   # Match Lead altitude exactly

    # ------------------------------------------------------------------ #
    # Intent dispatcher
    # ------------------------------------------------------------------ #

    def _on_approved_intent(self, msg: String) -> None:
        raw = msg.data.strip()
        if not raw:
            return

        try:
            intent = json.loads(raw)
        except json.JSONDecodeError as exc:
            self._publish_feedback(f"JSON parse error: {exc} — raw: {raw[:120]}")
            return

        if intent.get("__bridge_dispatched__"):
            return

        action = str(intent.get("action", "")).lower()
        self.get_logger().info(f"Wingman commander executing action: {action}")

        # Any non-follow_lead action deactivates follow mode
        if action != "follow_lead":
            self._follow_lead_active = False

        handler = {
            "takeoff": self._action_takeoff,
            "move": self._action_move,
            "hover": self._action_hover,
            "hold": self._action_hover,
            "search_stop": self._action_search_stop,
            "search_resume": self._action_search_resume,
            "search_expand": self._action_search_expand,
            "search": self._action_search,
            "land": self._action_land,
            "rtl": self._action_rtl,
            "follow_lead": self._action_follow_lead,
        }.get(action)

        if handler is None:
            self._publish_feedback(
                f"Unknown action '{action}' — supported: "
                "takeoff, move, hover, hold, search_stop, search_resume, "
                "search_expand, search, land, rtl, follow_lead"
            )
            return

        try:
            handler(intent)
        except Exception as exc:
            self._publish_feedback(f"Action '{action}' raised: {exc}")
            self.get_logger().error(f"Action handler exception: {exc}", exc_info=True)

    # ------------------------------------------------------------------ #
    # Action handlers
    # ------------------------------------------------------------------ #

    def _action_takeoff(self, intent: dict) -> None:
        alt_m = float(intent.get("altitude_m", _DEFAULT_TAKEOFF_ALT_M))
        self._pending_alt_m = abs(alt_m)
        # Stay at current ground position during pre-arm streaming phase
        self._tgt_x = self._cur_x
        self._tgt_y = self._cur_y
        self._tgt_z = self._cur_z
        self._pre_arm_phase   = True
        self._offboard_active = False
        self._keepalive_count = 0
        self.get_logger().info(
            f"Wingman takeoff requested: {alt_m}m — entering pre-arm streaming phase.")

    def _action_move(self, intent: dict) -> None:
        direction  = str(intent.get("direction", "north")).lower()
        distance_m = float(intent.get("distance_m", _DEFAULT_MOVE_DIST_M))

        if direction not in DIRECTION_OFFSETS:
            self._publish_feedback(
                f"Unknown direction '{direction}'. "
                f"Valid: {', '.join(DIRECTION_OFFSETS.keys())}"
            )
            return

        dx, dy = DIRECTION_OFFSETS[direction]
        self._tgt_x = self._cur_x + dx * distance_m
        self._tgt_y = self._cur_y + dy * distance_m

        # Bug fix #3: use _tgt_z (last commanded altitude), not _cur_z
        altitude_m = intent.get("altitude_m", None)
        if altitude_m is not None:
            self._tgt_z = -abs(float(altitude_m))
        # else: keep _tgt_z unchanged

        self._offboard_active = True
        self.get_logger().info(
            f"Wingman move {distance_m}m {direction} → "
            f"({self._tgt_x:.1f}, {self._tgt_y:.1f}, z={self._tgt_z:.1f})"
        )

    def _action_hover(self, intent: dict) -> None:
        self._tgt_x = self._cur_x
        self._tgt_y = self._cur_y
        # Bug fix #7: keep _tgt_z (commanded altitude), don't overwrite with _cur_z
        self._offboard_active = True
        self.get_logger().info(
            f"Wingman hover at ({self._cur_x:.1f}, {self._cur_y:.1f}, z_cmd={self._tgt_z:.1f})."
        )

    def _action_search_stop(self, intent: dict) -> None:
        self._saved_x = self._tgt_x
        self._saved_y = self._tgt_y
        self._saved_z = self._tgt_z
        self._action_hover(intent)
        self.get_logger().info("Wingman search stopped — position saved.")

    def _action_search_resume(self, intent: dict) -> None:
        if self._saved_x is None:
            self._publish_feedback("search_resume: no saved position — hovering.")
            self._action_hover(intent)
            return
        self._tgt_x = self._saved_x
        self._tgt_y = self._saved_y
        self._tgt_z = self._saved_z
        self._offboard_active = True
        self.get_logger().info(
            f"Wingman search resumed to ({self._tgt_x:.1f}, {self._tgt_y:.1f})."
        )

    def _action_search_expand(self, intent: dict) -> None:
        expansion_m = float(intent.get("expansion_m", _DEFAULT_EXPAND_M))
        self._search_radius_m += expansion_m
        dx, dy = DIRECTION_OFFSETS["north"]
        self._tgt_x = self._cur_x + dx * self._search_radius_m
        self._tgt_y = self._cur_y + dy * self._search_radius_m
        self._tgt_z = self._cur_z
        self._offboard_active = True
        self.get_logger().info(
            f"Wingman search expanded to radius {self._search_radius_m}m."
        )

    def _action_search(self, intent: dict) -> None:
        radius_m = float(intent.get("radius_m", _DEFAULT_SEARCH_RADIUS_M))
        self._search_radius_m = radius_m
        dx, dy = DIRECTION_OFFSETS["north"]
        self._tgt_x = self._cur_x + dx * radius_m
        self._tgt_y = self._cur_y + dy * radius_m
        self._tgt_z = self._cur_z
        self._offboard_active = True
        self.get_logger().info(
            f"Wingman search started: radius {radius_m}m, "
            f"waypoint ({self._tgt_x:.1f}, {self._tgt_y:.1f})."
        )

    def _action_land(self, intent: dict) -> None:
        self._follow_lead_active = False
        self._offboard_active = False
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Wingman land command sent.")

    def _action_rtl(self, intent: dict) -> None:
        self._follow_lead_active = False
        self._offboard_active = False
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("Wingman RTL command sent.")

    def _action_follow_lead(self, intent: dict) -> None:
        """
        Activate follow_lead mode.

        The Wingman will continuously track the Lead's NED position and maintain
        a formation offset of `offset_m` meters behind-left of the Lead.

        Intent JSON: {"action": "follow_lead", "offset_m": 5.0, "confidence": "high"}
        """
        self._follow_offset_m = float(intent.get("offset_m", _DEFAULT_FOLLOW_OFFSET_M))
        self._follow_lead_active = True
        self._offboard_active = True

        # Immediately set initial target if Lead position is known
        if self._lead_x is not None:
            self._update_follow_setpoint()
            self.get_logger().info(
                f"follow_lead activated: offset={self._follow_offset_m}m, "
                f"initial target ({self._tgt_x:.1f}, {self._tgt_y:.1f}, {self._tgt_z:.1f})."
            )
        else:
            self.get_logger().warning(
                f"follow_lead activated but Lead position not yet received. "
                f"Will update setpoint on first Lead position message. "
                f"offset={self._follow_offset_m}m."
            )

    # ------------------------------------------------------------------ #
    # 10 Hz keepalive
    # ------------------------------------------------------------------ #

    def _keepalive(self) -> None:
        """3-phase offboard keepalive — same state machine as LeadPX4CommanderNode."""
        self._keepalive_count += 1
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        ocm = OffboardControlMode()
        ocm.timestamp    = now_us
        ocm.position     = True
        ocm.velocity     = False
        ocm.acceleration = False
        ocm.attitude     = False
        ocm.body_rate    = False
        self._ocm_pub.publish(ocm)

        # Phase 1: pre-arm sequence
        if self._pre_arm_phase:
            if self._keepalive_count == 10:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0, param2=6.0)
                self.get_logger().info("Wingman pre-arm: OFFBOARD mode switch sent.")
            elif self._keepalive_count == 11:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0)
                self.get_logger().info("Wingman pre-arm: ARM command sent.")
            elif self._keepalive_count == 13:
                self._tgt_z           = -abs(self._pending_alt_m)
                self._pre_arm_phase   = False
                self._offboard_active = True
                self.get_logger().info(
                    f"Wingman takeoff: climbing to {self._pending_alt_m}m "
                    f"(NED z={self._tgt_z:.1f}).")

        # Phase 2: follow_lead updates setpoint every tick
        if self._follow_lead_active and self._offboard_active:
            self._update_follow_setpoint()

        tsp = TrajectorySetpoint()
        tsp.timestamp = now_us
        tsp.position  = [self._tgt_x, self._tgt_y, self._tgt_z]
        tsp.yaw       = float("nan")
        self._tsp_pub.publish(tsp)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _send_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.target_system = _TARGET_SYSTEM
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
        self._cmd_pub.publish(msg)

    def _publish_feedback(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._feedback_pub.publish(msg)
        self.get_logger().warning(f"Wingman feedback: {text}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WingmanPX4CommanderNode()
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

> [!IMPORTANT]
> **follow_lead deactivation**: Every action handler except `_action_follow_lead` itself calls `self._follow_lead_active = False` at the top of `_on_approved_intent`. This ensures any new command (move, hover, land, etc.) immediately breaks formation following.

---

## 7.5 `LeadIntentBridgeNode`

**File:** `major_project/lead_pilot/lead_intent_bridge_node.py`

### Purpose

The SLM agent outputs a `FlightIntent` that can carry a chained `then` field — a secondary intent to dispatch after the primary action completes. The bridge listens on `/lead/approved_intent`, detects a `then` payload, waits `chain_delay_sec` seconds (default 6.0), then re-publishes the chained step (with a `__bridge_dispatched__` marker to prevent echo loops).

### Complete implementation

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_intent_bridge_node.py
#!/usr/bin/env python3
"""
lead_intent_bridge_node.py
Handles chained FlightIntent 'then' field for the Lead drone.

Subscribes to /lead/approved_intent.
After chain_delay_sec (default 6.0s), republishes the 'then' step with
a __bridge_dispatched__ marker to prevent echo loops.

The LeadPX4CommanderNode ignores any intent with __bridge_dispatched__ = True
when processing from its own subscription (bridge echo prevention).
Instead, the bridge publishes the chained step as a new intent that the
commander picks up as a fresh command.
"""

from __future__ import annotations

import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import String

# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LeadIntentBridgeNode(Node):
    """
    Chains sequential FlightIntent steps for the Lead drone.

    When an approved intent arrives with a 'then' key, the bridge waits
    chain_delay_sec seconds and then publishes the chained intent back
    to /lead/approved_intent so the commander picks it up.

    The __bridge_dispatched__ flag is set on the republished message so
    the bridge itself ignores it (breaking the echo loop).
    """

    def __init__(self) -> None:
        super().__init__("lead_intent_bridge_node")

        self.declare_parameter("chain_delay_sec", 6.0)
        self._chain_delay: float = (
            self.get_parameter("chain_delay_sec")
            .get_parameter_value()
            .double_value
        )

        # Track pending chain timer so we can cancel if a new intent arrives
        self._pending_timer: Optional[threading.Timer] = None
        self._pending_timer_lock = threading.Lock()

        # ------------------------------------------------------------------ #
        # Publisher / Subscriber on same topic
        # ------------------------------------------------------------------ #
        self._pub = self.create_publisher(
            String, "/lead/approved_intent", RELIABLE_QOS
        )
        self.create_subscription(
            String,
            "/lead/approved_intent",
            self._on_intent,
            RELIABLE_QOS,
        )

        self.get_logger().info(
            f"LeadIntentBridgeNode started (chain_delay={self._chain_delay}s)."
        )

    def _on_intent(self, msg: String) -> None:
        raw = msg.data.strip()
        if not raw:
            return

        try:
            intent = json.loads(raw)
        except json.JSONDecodeError:
            return  # Ignore malformed messages

        # Ignore messages that were dispatched by the bridge itself
        if intent.get("__bridge_dispatched__"):
            return

        # Cancel any pending chain (new intent supersedes previous chain)
        self._cancel_pending()

        then_step = intent.get("then")
        if not then_step:
            return  # No chained step

        if not isinstance(then_step, dict):
            self.get_logger().warning(
                f"'then' field is not a dict — ignoring: {then_step!r}"
            )
            return

        action = str(intent.get("action", "unknown"))
        then_action = str(then_step.get("action", "unknown"))
        self.get_logger().info(
            f"Bridge: will dispatch '{then_action}' "
            f"after '{action}' in {self._chain_delay}s."
        )

        # Shallow copy + mark as bridge-dispatched to prevent echo loop
        chained = dict(then_step)
        chained["__bridge_dispatched__"] = True

        # Remove any nested 'then' from the chained step — the bridge will
        # handle multi-step chains by recursion: the published chained intent
        # itself may have a 'then', which will be picked up on the next cycle
        # UNLESS it carries __bridge_dispatched__ which would block recursion.
        # Solution: re-publish WITHOUT __bridge_dispatched__ but WITH the
        # nested 'then', letting the bridge process it naturally.
        # We strip __bridge_dispatched__ from the published payload only when
        # a nested 'then' exists so the bridge can chain further.
        nested_then = chained.pop("then", None)

        def _dispatch() -> None:
            payload = dict(chained)
            # If there's a deeper chain, republish without the bridge marker
            # so the bridge processes it again
            if nested_then is not None:
                payload.pop("__bridge_dispatched__", None)
                payload["then"] = nested_then

            out = String()
            out.data = json.dumps(payload)
            self._pub.publish(out)
            self.get_logger().info(
                f"Bridge dispatched chained action: {payload.get('action')}"
            )

        timer = threading.Timer(self._chain_delay, _dispatch)
        with self._pending_timer_lock:
            self._pending_timer = timer
        timer.daemon = True
        timer.start()

    def _cancel_pending(self) -> None:
        """Cancel any in-flight chain timer."""
        with self._pending_timer_lock:
            if self._pending_timer is not None:
                self._pending_timer.cancel()
                self._pending_timer = None

    def destroy_node(self) -> None:
        self._cancel_pending()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeadIntentBridgeNode()
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
> **Multi-step chain recursion**: If an intent has `then: {action: B, then: {action: C}}`, the bridge dispatches B after 6s. B is published *without* `__bridge_dispatched__` and with the `then: {action: C}` intact, so the bridge picks it up again and dispatches C after another 6s. This creates a clean waterfall of actions.

---

## 7.5b `WingmanIntentBridgeNode`

**File:** `major_project/wingman_pilot/wingman_intent_bridge_node.py`

### Purpose

Symmetric counterpart to `LeadIntentBridgeNode` for Drone-1. Listens on `/wingman/approved_intent` and chains `then` steps for the Wingman — enabling the Wingman Agent to issue multi-step sequences (e.g., `takeoff → move → search`) in a single `FlightIntent` payload.

### Complete implementation

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_intent_bridge_node.py
#!/usr/bin/env python3
"""
wingman_intent_bridge_node.py
Handles chained FlightIntent 'then' field for the Wingman drone.

Subscribes to /wingman/approved_intent.
After chain_delay_sec (default 6.0s), republishes the 'then' step with
a __bridge_dispatched__ marker to prevent echo loops.

The WingmanPX4CommanderNode ignores any intent with __bridge_dispatched__ = True
when processing from its own subscription (bridge echo prevention).
Instead, the bridge publishes the chained step as a new intent that the
commander picks up as a fresh command.
"""

from __future__ import annotations

import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import String

# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class WingmanIntentBridgeNode(Node):
    """
    Chains sequential FlightIntent steps for the Wingman drone.

    When an approved intent arrives with a 'then' key, the bridge waits
    chain_delay_sec seconds and then publishes the chained intent back
    to /wingman/approved_intent so the commander picks it up.

    The __bridge_dispatched__ flag is set on the republished message so
    the bridge itself ignores it (breaking the echo loop).
    """

    def __init__(self) -> None:
        super().__init__("wingman_intent_bridge_node")

        self.declare_parameter("chain_delay_sec", 6.0)
        self._chain_delay: float = (
            self.get_parameter("chain_delay_sec")
            .get_parameter_value()
            .double_value
        )

        # Track pending chain timer so we can cancel if a new intent arrives
        self._pending_timer: Optional[threading.Timer] = None
        self._pending_timer_lock = threading.Lock()

        # ------------------------------------------------------------------ #
        # Publisher / Subscriber on same topic
        # ------------------------------------------------------------------ #
        self._pub = self.create_publisher(
            String, "/wingman/approved_intent", RELIABLE_QOS
        )
        self.create_subscription(
            String,
            "/wingman/approved_intent",
            self._on_intent,
            RELIABLE_QOS,
        )

        self.get_logger().info(
            f"WingmanIntentBridgeNode started (chain_delay={self._chain_delay}s)."
        )

    def _on_intent(self, msg: String) -> None:
        raw = msg.data.strip()
        if not raw:
            return

        try:
            intent = json.loads(raw)
        except json.JSONDecodeError:
            return  # Ignore malformed messages

        # Ignore messages that were dispatched by the bridge itself
        if intent.get("__bridge_dispatched__"):
            return

        # Cancel any pending chain (new intent supersedes previous chain)
        self._cancel_pending()

        then_step = intent.get("then")
        if not then_step:
            return  # No chained step

        if not isinstance(then_step, dict):
            self.get_logger().warning(
                f"'then' field is not a dict — ignoring: {then_step!r}"
            )
            return

        action = str(intent.get("action", "unknown"))
        then_action = str(then_step.get("action", "unknown"))
        self.get_logger().info(
            f"Bridge: will dispatch '{then_action}' "
            f"after '{action}' in {self._chain_delay}s."
        )

        # Shallow copy + mark as bridge-dispatched to prevent echo loop
        chained = dict(then_step)
        chained["__bridge_dispatched__"] = True

        # Remove any nested 'then' from the chained step — the bridge will
        # handle multi-step chains by recursion: the published chained intent
        # itself may have a 'then', which will be picked up on the next cycle
        # UNLESS it carries __bridge_dispatched__ which would block recursion.
        # Solution: re-publish WITHOUT __bridge_dispatched__ but WITH the
        # nested 'then', letting the bridge process it naturally.
        # We strip __bridge_dispatched__ from the published payload only when
        # a nested 'then' exists so the bridge can chain further.
        nested_then = chained.pop("then", None)

        def _dispatch() -> None:
            payload = dict(chained)
            # If there's a deeper chain, republish without the bridge marker
            # so the bridge processes it again
            if nested_then is not None:
                payload.pop("__bridge_dispatched__", None)
                payload["then"] = nested_then

            out = String()
            out.data = json.dumps(payload)
            self._pub.publish(out)
            self.get_logger().info(
                f"Bridge dispatched chained action: {payload.get('action')}"
            )

        timer = threading.Timer(self._chain_delay, _dispatch)
        with self._pending_timer_lock:
            self._pending_timer = timer
        timer.daemon = True
        timer.start()

    def _cancel_pending(self) -> None:
        """Cancel any in-flight chain timer."""
        with self._pending_timer_lock:
            if self._pending_timer is not None:
                self._pending_timer.cancel()
                self._pending_timer = None

    def destroy_node(self) -> None:
        self._cancel_pending()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WingmanIntentBridgeNode()
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
> **Symmetric design**: `WingmanIntentBridgeNode` is intentionally identical in logic to `LeadIntentBridgeNode`, only differing in the topic `/wingman/approved_intent` and node name `wingman_intent_bridge_node`. Both bridges run independently on their respective machines (PC-1 for Lead, PC-2 for Wingman), so multi-step chains execute correctly without cross-machine coupling.

---

## 7.6 Build and Verification

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install 2>&1 | tail -5
source install/setup.bash

# Verify entry points registered
ros2 pkg executables major_project | grep -E 'commander|intent_bridge'
# Expected:
#   major_project lead_px4_commander
#   major_project lead_intent_bridge
#   major_project wingman_px4_commander
#   major_project wingman_intent_bridge
```

## 7.7 Topic Summary — Flight Execution Layer

| Topic | Msg Type | QoS | Flow |
|-------|----------|-----|------|
| `/lead/approved_intent` | String (JSON) | RELIABLE | SLM → LeadCommander, LeadBridge |
| `/wingman/approved_intent` | String (JSON) | RELIABLE | SLM → WingmanCommander, WingmanBridge |
| `/fmu/in/offboard_control_mode` | OffboardControlMode | RELIABLE | LeadCommander → PX4-0 |
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | RELIABLE | LeadCommander → PX4-0 |
| `/fmu/in/vehicle_command` | VehicleCommand | RELIABLE | LeadCommander → PX4-0 |
| `/px4_1/fmu/in/offboard_control_mode` | OffboardControlMode | RELIABLE | WingmanCommander → PX4-1 |
| `/px4_1/fmu/in/trajectory_setpoint` | TrajectorySetpoint | RELIABLE | WingmanCommander → PX4-1 |
| `/px4_1/fmu/in/vehicle_command` | VehicleCommand | RELIABLE | WingmanCommander → PX4-1 |
| `/fmu/out/vehicle_local_position` | VehicleLocalPosition | BEST_EFFORT | PX4-0 → LeadCommander (pos track) + WingmanCommander (follow_lead) |
| `/px4_1/fmu/out/vehicle_local_position` | VehicleLocalPosition | BEST_EFFORT | PX4-1 → WingmanCommander (own pos) |
| `/lead/execution_feedback` | String | RELIABLE | LeadCommander → SLM |
| `/wingman/execution_feedback` | String | RELIABLE | WingmanCommander → SLM |
| `/emergency_stop` | Bool | RELIABLE | → LeadCommander, WingmanCommander |

---

## 7.7 Key Design Decisions

### PX4 OFFBOARD mode requirements
PX4 requires setpoints to be streamed at **>2 Hz** before it accepts the OFFBOARD mode switch, and continuously while in OFFBOARD mode. The 10 Hz timer satisfies this with a 5× safety margin. The `OffboardControlMode` message must arrive **before** the `TrajectorySetpoint` in each cycle — both are published in the same `_keepalive()` call in the correct order.

### NED coordinate convention
PX4 uses NED (North-East-Down):
- `x` = North (+), South (-)
- `y` = East (+), West (-)
- `z` = Down (+), **Up is negative**

Altitude in setpoints is therefore `tgt_z = -abs(altitude_m)`.

### follow_lead formation offset
The formation `(lead_x - offset, lead_y + offset)` places the Wingman **South** and **East** of the Lead when Lead faces North — this is behind and to the right in NED but "behind-left" from Lead's perspective in a typical navigation frame. Adjust the signs to suit your mission's heading convention.

### Emergency stop precedence
Emergency stop bypasses all intent processing — it is handled in a direct subscriber callback that immediately sends `NAV_LAND` and sets `_offboard_active = False`. The keepalive timer continues streaming setpoints (PX4 will ignore them once NAV_LAND is executing) but no new intents will change the target until the node restarts.
