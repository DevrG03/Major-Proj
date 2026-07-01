# Part 3: Common Modules

> **Target:** PC-1 (then synced to PC-2)
> **Prerequisites:** Part 2 complete — package scaffold exists at `~/major_ws/src/major_project/`

These 7 modules are the foundation all agent nodes import. Every loophole fix from the architectural audit is applied here.

---

## 3.1 Pydantic Schemas (`common/schemas.py`)

Adds `AgentMessage` typed inter-agent envelope to the original schemas.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/schemas.py
"""
All Pydantic v2 schemas for the multi-drone SLM pilot system.

New in this version:
- AgentMessage: typed envelope for ALL inter-agent communications.
  Prevents status messages from being mistaken as new task orders (Loophole #7 fix).
- make_agent_msg(): convenience serialiser.
"""
from __future__ import annotations
from typing import Optional, Literal
from pydantic import BaseModel, Field, field_validator
from dataclasses import dataclass
import json


# ─────────────────────────────────────────────────────────────────
# Flight Intent (unchanged from minor project)
# ─────────────────────────────────────────────────────────────────

class FlightIntent(BaseModel):
    action: Literal[
        "takeoff", "move", "hover", "land", "rtl",
        "search", "search_stop", "search_resume", "search_expand",
        "hold", "follow_lead"
    ]
    altitude: Optional[float] = Field(None, ge=0.5, le=50.0)
    distance: Optional[float] = Field(None, ge=0.1, le=100.0)
    direction: Optional[str] = None
    speed: Optional[float] = Field(None, ge=0.1, le=10.0)
    heading: Optional[float] = Field(None, ge=0.0, le=360.0)
    offset_m: Optional[float] = Field(None, ge=1.0, le=30.0)   # for follow_lead
    then: Optional[FlightIntent] = None
    confidence: Literal["high", "medium", "low"]
    clarification_question: Optional[str] = None

    @field_validator('direction')
    @classmethod
    def validate_direction(cls, v):
        if v is None:
            return v
        valid = {
            'north', 'south', 'east', 'west',
            'northeast', 'northwest', 'southeast', 'southwest',
            'forward', 'backward', 'left', 'right', 'up', 'down'
        }
        if v.lower() not in valid:
            return None
        return v.lower()

FlightIntent.model_rebuild()   # required for self-referential 'then' field


# ─────────────────────────────────────────────────────────────────
# Drone position and situational awareness
# ─────────────────────────────────────────────────────────────────

class DronePosition(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0       # NED frame: negative = up
    heading: float = 0.0  # degrees, 0 = north
    speed: float = 0.0    # m/s


class SituationalAwareness(BaseModel):
    drone_id: str
    position: DronePosition
    battery_pct: float = Field(ge=0.0, le=100.0)
    flight_mode: str
    gps_fix: bool = True
    altitude_baro: float = 0.0
    camera_summary: str = "No camera data"

    def to_prompt_block(self) -> str:
        pos = self.position
        return (
            f"[DRONE | {self.drone_id}] "
            f"pos:({pos.x:.1f},{pos.y:.1f}) alt:{self.altitude_baro:.1f}m "
            f"hdg:{pos.heading:.0f}° spd:{pos.speed:.1f}m/s "
            f"bat:{self.battery_pct:.0f}% mode:{self.flight_mode} "
            f"gps:{'OK' if self.gps_fix else 'NO'}\n"
            f"[CAMERA | {self.drone_id}] {self.camera_summary}"
        )


# ─────────────────────────────────────────────────────────────────
# Multi-drone coordination schemas
# ─────────────────────────────────────────────────────────────────

class WingmanOrder(BaseModel):
    order_id: Optional[str] = None
    mission_context: str
    intent: FlightIntent
    priority: Literal["routine", "urgent", "emergency"]
    lead_position: Optional[DronePosition] = None
    confidence: Literal["high", "medium", "low"]
    clarification_question: Optional[str] = None

    def to_prompt_block(self) -> str:
        intent_str = self.intent.model_dump_json(exclude_none=True)
        return (
            f"[ORDER from LEAD | id:{self.order_id} priority:{self.priority.upper()}]\n"
            f"Context: {self.mission_context}\n"
            f"Command: {intent_str}\n"
            f"Lead confidence: {self.confidence}"
            + (f"\nLead asks: {self.clarification_question}"
               if self.clarification_question else "")
        )


class StatusReport(BaseModel):
    order_id: str
    status: Literal[
        "acknowledged", "executing", "completed", "failed", "needs_clarification"
    ]
    drone_position: DronePosition
    battery_pct: float = Field(ge=0.0, le=100.0)
    obstacle_detected: bool = False
    obstacle_description: Optional[str] = None
    situation_summary: str
    clarification_question: Optional[str] = None
    confidence: Literal["high", "medium", "low"]

    def to_prompt_block(self) -> str:
        return (
            f"[WINGMAN REPORT | order:{self.order_id} status:{self.status.upper()}]\n"
            f"{self.situation_summary}"
            + (f"\nWingman asks: {self.clarification_question}"
               if self.clarification_question else "")
            + (f"\nObstacle: {self.obstacle_description}"
               if self.obstacle_detected else "")
        )


class LeadOutput(BaseModel):
    my_intent: Optional[FlightIntent] = None
    wingman_order: Optional[WingmanOrder] = None
    confidence: Literal["high", "medium", "low"]
    situation_report: str
    clarification_question: Optional[str] = None


class WingmanOutput(BaseModel):
    intent: Optional[FlightIntent] = None
    confidence: Literal["high", "medium", "low"]
    situation_summary: str
    clarification_question: Optional[str] = None


# ─────────────────────────────────────────────────────────────────
# Safety event
# ─────────────────────────────────────────────────────────────────

@dataclass
class SafetyEvent:
    drone_id: str        # "DRONE_0", "DRONE_1", "BOTH"
    event_type: str      # "low_battery", "battery_rtl", "gps_lost",
                         # "emergency_stop", "proximity_warning"
    severity: str        # "warning", "critical"
    message: str
    value: float = 0.0   # numeric value (e.g. battery%, distance)


# ─────────────────────────────────────────────────────────────────
# Inter-agent typed message envelope  (Loophole #7 fix)
#
# ALL messages between Lead and Wingman agents MUST use this envelope.
# This prevents status messages from being mistaken as task orders.
# ─────────────────────────────────────────────────────────────────

class AgentMessage(BaseModel):
    """
    Typed envelope for all /agent/lead_to_wingman and /agent/wingman_to_lead messages.

    type meanings:
      task     — new mission goal for the receiver (triggers _assign_goal)
      status   — informational update (does NOT trigger new mission)
      reply    — answer to a previous query
      query    — question requiring a reply
      abort    — command receiver to abort current mission immediately
      position — position update (for situational awareness only)
    """
    type: Literal["task", "status", "reply", "query", "abort", "position"]
    sender: Literal["LEAD", "WINGMAN"]
    content: str
    order_id: Optional[str] = None


def make_agent_msg(
    type: str,
    sender: str,
    content: str,
    order_id: Optional[str] = None
) -> str:
    """Serialize an AgentMessage to a JSON string for ROS2 String message data."""
    msg = AgentMessage(
        type=type,          # type: ignore[arg-type]
        sender=sender,      # type: ignore[arg-type]
        content=content,
        order_id=order_id
    )
    return msg.model_dump_json()


# ─────────────────────────────────────────────────────────────────
# Compact value expansion (token reduction, backward-compatible)
# ─────────────────────────────────────────────────────────────────

_CONF_EXPAND:  dict[str, str] = {"H": "high", "M": "medium", "L": "low"}
_DIR_EXPAND: dict[str, str] = {
    "N": "north", "S": "south", "E": "east", "W": "west",
    "NE": "northeast", "NW": "northwest", "SE": "southeast", "SW": "southwest",
    "FWD": "forward", "BCK": "backward", "L": "left", "R": "right",
    "UP": "up", "DN": "down",
}
_PRI_EXPAND: dict[str, str] = {"R": "routine", "U": "urgent", "E": "emergency"}


def expand_compact_values(data):
    """Expand abbreviated SLM output values before Pydantic validation."""
    if isinstance(data, list):
        return [expand_compact_values(item) for item in data]
    if not isinstance(data, dict):
        return data
    out = {}
    for k, v in data.items():
        if isinstance(v, (dict, list)):
            out[k] = expand_compact_values(v)
        elif k == "confidence" and isinstance(v, str):
            out[k] = _CONF_EXPAND.get(v, v)
        elif k == "direction" and isinstance(v, str):
            out[k] = _DIR_EXPAND.get(v, v)
        elif k == "priority" and isinstance(v, str):
            out[k] = _PRI_EXPAND.get(v, v)
        else:
            out[k] = v
    return out


def parse_lead_output(raw_json: str) -> Optional[LeadOutput]:
    try:
        data = expand_compact_values(json.loads(raw_json))
        return LeadOutput(**data)
    except Exception:
        return None


def parse_wingman_output(raw_json: str) -> Optional[WingmanOutput]:
    try:
        data = expand_compact_values(json.loads(raw_json))
        return WingmanOutput(**data)
    except Exception:
        return None
EOF
```

---

## 3.2 Ollama Client (`common/ollama_client.py`)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/ollama_client.py
"""
Thin wrapper around the Ollama REST API.
Handles timeout, retry, and JSON extraction from response.
"""
import requests
import time
import logging

logger = logging.getLogger(__name__)


class OllamaClient:

    def __init__(
        self,
        host: str = "localhost",
        port: int = 11434,
        model: str = "qwen2.5-coder:3b",
        num_ctx: int = 2048,
        max_retries: int = 3,
        timeout: float = 15.0,
    ):
        self.url = f"http://{host}:{port}/api/generate"
        self.model = model
        self.num_ctx = num_ctx
        self.max_retries = max_retries
        self.timeout = timeout

    def infer(self, prompt: str, system: str) -> tuple[str | None, float]:
        """
        Run inference. Returns (json_string_or_None, latency_seconds).
        Retries up to max_retries on failure.
        """
        payload = {
            "model":  self.model,
            "prompt": prompt,
            "system": system,
            "stream": False,
            "format": "json",
            "options": {
                "num_ctx":        self.num_ctx,
                "temperature":    0,
                "top_p":          1.0,
                "repeat_penalty": 1.0,
            },
        }

        for attempt in range(self.max_retries):
            t_start = time.perf_counter()
            try:
                response = requests.post(
                    self.url, json=payload, timeout=self.timeout)
                latency = time.perf_counter() - t_start

                if response.status_code == 200:
                    raw = response.json().get("response", "").strip()
                    if raw.startswith("{"):
                        return raw, latency
                    # Try to extract JSON object from response
                    start = raw.find("{")
                    end   = raw.rfind("}") + 1
                    if start >= 0 and end > start:
                        return raw[start:end], latency
                    logger.warning(f"No JSON in response: {raw[:80]}")
                else:
                    logger.warning(f"Ollama HTTP {response.status_code}")

            except requests.Timeout:
                logger.warning(
                    f"Ollama timeout (attempt {attempt + 1}/{self.max_retries})")
            except requests.ConnectionError:
                logger.warning(
                    f"Ollama connection error (attempt {attempt + 1}/{self.max_retries})")
            except Exception as e:
                logger.warning(f"Ollama error: {e} (attempt {attempt + 1})")

            if attempt < self.max_retries - 1:
                time.sleep(0.5)

        return None, 0.0
EOF
```

---

## 3.3 Confidence Gate (`common/confidence_gate.py`)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/confidence_gate.py
"""
Confidence gate policy for Lead and Wingman pilots.
Implements the two-level cascade described in the architecture document.
"""
from enum import Enum


class LeadAction(Enum):
    EXECUTE                = "execute"
    EXECUTE_WITH_WARNING   = "execute_with_warning"
    WITHHOLD_CLARIFY_HUMAN = "withhold_clarify_human"


class WingmanAction(Enum):
    EXECUTE               = "execute"
    EXECUTE_WITH_WARNING  = "execute_with_warning"
    CLARIFY_LEAD          = "clarify_lead"   # Wingman never contacts human directly


def gate_lead(confidence: str) -> LeadAction:
    """
    Lead Pilot gate:
      high   → execute intent + send wingman order immediately
      medium → execute + warn GCS, send wingman order with warning flag
      low    → withhold, request clarification from Human Commander
    """
    if confidence == "high":
        return LeadAction.EXECUTE
    elif confidence == "medium":
        return LeadAction.EXECUTE_WITH_WARNING
    else:
        return LeadAction.WITHHOLD_CLARIFY_HUMAN


def gate_wingman(confidence: str) -> WingmanAction:
    """
    Wingman gate:
      high   → execute immediately, report to Lead
      medium → execute with assumption, flag in status report
      low    → withhold, send clarification request to Lead
    """
    if confidence == "high":
        return WingmanAction.EXECUTE
    elif confidence == "medium":
        return WingmanAction.EXECUTE_WITH_WARNING
    else:
        return WingmanAction.CLARIFY_LEAD
EOF
```

---

## 3.4 Normaliser (`common/normaliser.py`)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/normaliser.py
"""
Normalises SLM output to canonical action names.
The SLM often uses variant spellings; this maps them to schema values.
Applied BEFORE Pydantic validation.
"""

ACTION_ALIASES: dict[str, str] = {
    # takeoff
    "take_off": "takeoff", "take off": "takeoff", "launch": "takeoff",
    "liftoff": "takeoff", "lift_off": "takeoff", "ascend": "takeoff",
    "go up": "takeoff", "fly up": "takeoff",
    # move
    "fly": "move", "go": "move", "navigate": "move", "travel": "move",
    "proceed": "move", "advance": "move", "translate": "move",
    # hover
    "stop": "hover", "halt": "hover", "stay": "hover", "wait_action": "hover",
    "hold_position": "hover", "pause": "hover", "maintain": "hover",
    # land
    "landing": "land", "touch down": "land", "touchdown": "land",
    "descend and land": "land", "set down": "land",
    # rtl
    "return": "rtl", "come back": "rtl", "go home": "rtl",
    "return to home": "rtl", "return to launch": "rtl", "rth": "rtl",
    # search
    "scan": "search", "survey": "search", "look": "search",
    "inspect": "search", "investigate": "search", "patrol": "search",
    "recon": "search", "reconnaissance": "search",
    # hold (wingman)
    "hold position": "hold", "stay in place": "hold", "remain": "hold",
    # follow_lead
    "follow": "follow_lead", "trail": "follow_lead", "shadow": "follow_lead",
}

DIRECTION_ALIASES: dict[str, str] = {
    "n": "north", "s": "south", "e": "east", "w": "west",
    "ne": "northeast", "nw": "northwest", "se": "southeast", "sw": "southwest",
    "fwd": "forward", "back": "backward", "bwd": "backward",
    "ahead": "forward", "behind": "backward",
}

_CANONICAL_ACTIONS = frozenset({
    "takeoff", "move", "hover", "land", "rtl",
    "search", "search_stop", "search_resume", "search_expand",
    "hold", "follow_lead",
})


def normalise_action(action: str) -> str:
    if action is None:
        return "hover"
    cleaned = action.strip().lower().replace("-", " ").replace("_", " ")
    # Direct canonical check
    cleaned_underscore = cleaned.replace(" ", "_")
    if cleaned_underscore in _CANONICAL_ACTIONS:
        return cleaned_underscore
    # Alias lookup (try both space and underscore forms)
    return ACTION_ALIASES.get(cleaned,
           ACTION_ALIASES.get(cleaned_underscore, "hover"))


def normalise_direction(direction: str) -> str:
    if direction is None:
        return None
    cleaned = direction.strip().lower()
    return DIRECTION_ALIASES.get(cleaned, cleaned)


def normalise_parsed(data: dict) -> dict:
    """
    Apply all normalisations to a raw SLM output dict before Pydantic validation.
    Call this BEFORE FlightIntent(**data).
    """
    if "action" in data:
        data["action"] = normalise_action(data["action"])
    if "direction" in data:
        data["direction"] = normalise_direction(data["direction"])
    # Recursively normalise nested 'then' chain
    if "then" in data and isinstance(data["then"], dict):
        data["then"] = normalise_parsed(data["then"])
    return data
EOF
```

---

## 3.5 Tool Registry (`common/tool_registry.py`)

> **All critical loophole fixes are in this file:**
> - #1 fix: `_wait()` and `_search()` are interruptible (check `_abort_event` every 0.5s / 2s)
> - #3 fix: `_ask_human()` and `_ask_lead()` return `PENDING_*` immediately — zero blocking
> - Loophole #6: `get_wingman_situation()` added to `LeadToolRegistry`
> - Loophole #7: All inter-agent messages use `AgentMessage` JSON envelope
> - Loophole #11: `follow_lead` tool added to `WingmanToolRegistry` instruction set

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/tool_registry.py
"""
Tool Registry — all tools available to Lead and Wingman agent loops.

Design rules (loophole fixes applied):
  - NO time.sleep() in tool execute methods that blocks > 2s continuously.
  - _wait() and _search() use a 0.5s / 2s polling loop that checks _abort_event.
  - _ask_human() / _ask_lead() NEVER block. They return a PENDING sentinel string.
    The agent loop monitors _waiting_for_human / _waiting_for_lead flags and
    performs passive get_situation() checks until the response arrives.
  - All inter-agent messages use AgentMessage JSON envelope.
"""
from __future__ import annotations
import json
import time
import threading
from dataclasses import dataclass, field
from typing import Callable


# ─────────────────────────────────────────────────────────────────
# Tool dataclass
# ─────────────────────────────────────────────────────────────────

@dataclass
class Tool:
    description: str
    params: dict[str, str]
    execute: Callable


# ─────────────────────────────────────────────────────────────────
# Base Tool Registry (shared by Lead and Wingman)
# ─────────────────────────────────────────────────────────────────

class BaseToolRegistry:
    """
    All tools shared between Lead and Wingman.
    ros_iface must expose:
        .lock (threading.Lock)
        .own_situation (str)
        .camera_summary (str)
        .obstacle_vector (str)
        .battery_pct (float)
        .agent_memory (AgentMemory)
        .pub_intent (Publisher<String>)
        ._mission_done (bool)
        ._mission_report (str)
        ._abort_event (threading.Event)
    """

    # Direction → (dx_NED_x, dy_NED_y) unit vectors
    _DIR_OFFSETS: dict[str, tuple[float, float]] = {
        'north':     ( 1.0,  0.0), 'south':     (-1.0,  0.0),
        'east':      ( 0.0,  1.0), 'west':      ( 0.0, -1.0),
        'northeast': ( 0.707,  0.707), 'northwest': ( 0.707, -0.707),
        'southeast': (-0.707,  0.707), 'southwest': (-0.707, -0.707),
        'forward':   ( 1.0,  0.0), 'backward':  (-1.0,  0.0),
        'left':      ( 0.0, -1.0), 'right':     ( 0.0,  1.0),
    }

    def __init__(self, ros_iface):
        self.ros = ros_iface
        self.tools: dict[str, Tool] = {}
        self._register_base_tools()

    # ── Registration ──────────────────────────────────────────────

    def _register_base_tools(self):
        self.tools.update({
            "takeoff": Tool(
                description="Arm and ascend to altitude metres (1–30). Returns immediately with ETA.",
                params={"altitude": "float: target altitude in metres (1–30)"},
                execute=self._takeoff),

            "move": Tool(
                description=(
                    "Fly in direction for distance metres. Optional altitude change. "
                    "Returns immediately with ETA. Call wait(ETA) then get_situation()."),
                params={
                    "direction": "str: N S E W NE NW SE SW forward backward left right",
                    "distance":  "float: metres (1–100)",
                    "altitude":  "float: optional new altitude in metres"},
                execute=self._move),

            "hover": Tool(
                description="Hold current position.",
                params={},
                execute=self._hover),

            "search": Tool(
                description=(
                    "Hover and scan camera for duration_sec seconds (5–60). "
                    "Accumulates all object detections. Returns summary when done."),
                params={"duration_sec": "int: scan duration seconds (5–60)"},
                execute=self._search),

            "land": Tool(
                description="Land drone at current position.",
                params={},
                execute=self._land),

            "rtl": Tool(
                description="Return to launch point and land.",
                params={},
                execute=self._rtl),

            "get_situation": Tool(
                description=(
                    "Read full sensor state: position, altitude, battery, GPS, "
                    "flight mode, camera, wingman position."),
                params={},
                execute=self._get_situation),

            "scan_camera": Tool(
                description="Get current camera detections with direction and distance.",
                params={},
                execute=self._scan_camera),

            "get_battery": Tool(
                description="Get own drone battery percentage.",
                params={},
                execute=self._get_battery),

            "remember": Tool(
                description="Store a fact in long-term persistent memory.",
                params={"fact": "str: the fact to store"},
                execute=self._remember),

            "recall": Tool(
                description="Retrieve stored facts matching a keyword.",
                params={"query": "str: keyword to search memory"},
                execute=self._recall),

            "wait": Tool(
                description=(
                    "Pause for seconds (1–30) while monitoring. "
                    "Abortable if a new goal arrives."),
                params={"seconds": "int: seconds to wait (1–30)"},
                execute=self._wait),

            "mission_complete": Tool(
                description="Declare mission accomplished and end the agent loop.",
                params={"report": "str: full mission completion summary"},
                execute=self._mission_complete),
        })

    # ── Publish helpers ────────────────────────────────────────────

    def _publish_intent(self, action_dict: dict):
        """Publish a FlightIntent JSON to the drone's approved_intent topic."""
        from std_msgs.msg import String as _String
        msg = _String()
        msg.data = json.dumps(action_dict)
        self.ros.pub_intent.publish(msg)

    # ── Flight tools ──────────────────────────────────────────────

    def _takeoff(self, params: dict) -> str:
        altitude = float(params.get('altitude', 5.0))
        altitude = max(1.0, min(30.0, altitude))
        # Bug fix #1: publish 'altitude_m' — commander reads intent.get('altitude_m')
        self._publish_intent({
            'action': 'takeoff', 'altitude_m': altitude, 'confidence': 'high'})
        
        # ECSM Fix: Pause agent thread for 2.0s to allow physical pre-arm sequence 
        # (1.3s) to complete before SLM can issue its next tool call.
        import time
        time.sleep(2.0)
        
        eta = int(altitude * 1.8) + 6
        return (
            f"Takeoff initiated. Ascending to {altitude}m. "
            f"ETA ~{eta}s. Call wait({eta}) then get_situation().")

    def _move(self, params: dict) -> str:
        # Normalise direction abbreviations
        _abbrev = {
            'N': 'north', 'S': 'south', 'E': 'east', 'W': 'west',
            'NE': 'northeast', 'NW': 'northwest',
            'SE': 'southeast', 'SW': 'southwest',
            'FWD': 'forward', 'BCK': 'backward',
            'L': 'left', 'R': 'right',
        }
        raw_dir   = str(params.get('direction', 'N')).upper().strip()
        direction = _abbrev.get(raw_dir, raw_dir.lower())
        distance  = float(params.get('distance', 10.0))
        distance  = max(1.0, min(100.0, distance))
        altitude  = params.get('altitude', None)

        # Bug fix #2: publish 'distance_m' — commander reads intent.get('distance_m')
        # Bug fix #5: publish 'altitude_m' — commander reads intent.get('altitude_m')
        intent: dict = {
            'action': 'move', 'direction': direction,
            'distance_m': distance, 'confidence': 'high'}
        if altitude is not None:
            intent['altitude_m'] = float(altitude)

        self._publish_intent(intent)
        eta = max(8, int(distance / 2.0) + 4)
        alt_note = f" Changing altitude to {altitude}m." if altitude is not None else ""
        return (
            f"Moving {direction} {distance}m.{alt_note} "
            f"ETA ~{eta}s. Call wait({eta}) then get_situation().")

    def _hover(self, params: dict) -> str:
        self._publish_intent({'action': 'hover', 'confidence': 'high'})
        return "Hovering at current position."

    def _search(self, params: dict) -> str:
        """
        Hover and accumulate camera detections for duration_sec seconds.
        The loop checks _abort_event every 2 seconds (interruptible).
        """
        duration = int(params.get('duration_sec', 15))
        duration = max(5, min(60, duration))

        # Hold position during search
        self._publish_intent({'action': 'hover', 'confidence': 'high'})
        time.sleep(1.0)   # 1s stabilisation (intentional, short, unavoidable)

        observations: list[str] = []
        deadline = time.time() + duration

        while time.time() < deadline:
            # ── Abort check (Loophole #4 fix) ──────────────────
            abort = getattr(self.ros, '_abort_event', None)
            if abort and abort.is_set():
                elapsed = duration - max(0.0, deadline - time.time())
                break

            # ── Collect camera data ─────────────────────────────
            with self.ros.lock:
                cam = self.ros.camera_summary
                obs = self.ros.obstacle_vector

            # Record distinct non-empty, non-clear detections
            if cam and 'not available' not in cam.lower() and \
               'no detection' not in cam.lower() and 'clear' not in cam.lower():
                entry = cam + (f" [{obs}]" if obs else "")
                if entry not in observations:
                    observations.append(entry[:150])

            time.sleep(2.0)   # check every 2 seconds

        elapsed = int(min(duration, duration - max(0.0, deadline - time.time())))

        if observations:
            combined = " | ".join(observations[:5])
            return f"Search complete ({elapsed}s of {duration}s). Detected: {combined}"
        return f"Search complete ({duration}s). Area clear — no objects detected."

    def _land(self, params: dict) -> str:
        self._publish_intent({'action': 'land', 'confidence': 'high'})
        return "Land command sent. Allow ~15s to touch down."

    def _rtl(self, params: dict) -> str:
        self._publish_intent({'action': 'rtl', 'confidence': 'high'})
        return "RTL initiated. Drone returning to launch. Allow ~40s."

    # ── Sensing tools ──────────────────────────────────────────────

    def _get_situation(self, params: dict) -> str:
        with self.ros.lock:
            sit = self.ros.own_situation
        if not sit:
            return "No situation data yet — telemetry initialising. Try again in 2s."
        return sit

    def _scan_camera(self, params: dict) -> str:
        with self.ros.lock:
            cam = self.ros.camera_summary
            obs = self.ros.obstacle_vector
        if not cam:
            return "Camera not available."
        result = cam
        if obs:
            result += f"\nObstacle vectors: {obs}"
        return result

    def _get_battery(self, params: dict) -> str:
        with self.ros.lock:
            pct = self.ros.battery_pct
        return f"Own drone battery: {pct:.0f}%"

    # ── Memory tools ───────────────────────────────────────────────

    def _remember(self, params: dict) -> str:
        fact = str(params.get('fact', '')).strip()
        if not fact:
            return "Error: 'fact' parameter is required."
        self.ros.agent_memory.remember(fact)
        return f"Remembered: '{fact[:100]}'"

    def _recall(self, params: dict) -> str:
        query = str(params.get('query', '')).strip()
        facts = self.ros.agent_memory.recall(query, limit=5)
        if not facts:
            return f"No memories found matching '{query}'."
        return "Recalled:\n" + "\n".join(f"  • {f}" for f in facts)

    # ── Timing tool ────────────────────────────────────────────────

    def _wait(self, params: dict) -> str:
        """
        Pause for seconds. Checks _abort_event every 0.5s.
        Never blocks more than 0.5s between abort checks (Loophole #2 fix).
        """
        secs = int(params.get('seconds', 5))
        # Bug fix #8: raised from 30 to 60 — takeoff ETA can be up to 33s (alt=15m),
        # and move ETA for 100m is 54s. Capping at 30 caused premature resumption.
        secs = max(1, min(60, secs))
        deadline = time.time() + secs

        while time.time() < deadline:
            abort = getattr(self.ros, '_abort_event', None)
            if abort and abort.is_set():
                elapsed = int(secs - max(0.0, deadline - time.time()))
                return f"Wait interrupted after ~{elapsed}s (new goal received)."
            time.sleep(0.5)

        return f"Waited {secs}s."

    # ── Mission control ────────────────────────────────────────────

    def _mission_complete(self, params: dict) -> str:
        report = str(params.get('report', 'Mission accomplished.')).strip()
        self.ros._mission_done   = True
        self.ros._mission_report = report
        return f"MISSION COMPLETE: {report}"

    # ── Registry helpers ───────────────────────────────────────────

    def is_valid(self, tool_name: str) -> bool:
        return tool_name in self.tools

    def execute(self, tool_name: str, params: dict) -> str:
        if tool_name not in self.tools:
            valid = sorted(self.tools.keys())
            return f"Unknown tool '{tool_name}'. Valid: {valid}"
        try:
            return self.tools[tool_name].execute(params)
        except Exception as exc:
            return f"Tool '{tool_name}' execution error: {str(exc)[:150]}"


# ─────────────────────────────────────────────────────────────────
# Lead Tool Registry
# ─────────────────────────────────────────────────────────────────

class LeadToolRegistry(BaseToolRegistry):
    """
    Lead Pilot exclusive tools.
    Adds: human comms, wingman coordination, wingman position query.

    Additional ros_iface requirements:
        .pub_wingman_msg  (Publisher<String> → /agent/lead_to_wingman)
        .pub_clarification (Publisher<String> → /clarification_request)
        ._waiting_for_human (bool) — set True by this tool, cleared by agent loop
        ._human_response (str | None) — filled by _on_voice callback
    """

    def __init__(self, ros_iface):
        super().__init__(ros_iface)
        self._register_lead_tools()

    def _register_lead_tools(self):
        self.tools.update({
            "get_wingman_situation": Tool(
                description=(
                    "Get Wingman drone's last known position and state from "
                    "the situation block (reads wingman_pos line)."),
                params={},
                execute=self._get_wingman_situation),

            "message_wingman": Tool(
                description=(
                    "Send a typed message to Wingman. "
                    "msg_type: 'task' (new mission), 'status' (info only), "
                    "'reply' (answer to Wingman query), 'abort' (stop Wingman mission)."),
                params={
                    "message":  "str: message content",
                    "msg_type": "str: task|status|reply|abort (default: task)"},
                execute=self._message_wingman),

            "ask_human": Tool(
                description=(
                    "Ask Ground Commander a question. NON-BLOCKING — returns immediately. "
                    "Agent continues monitoring situation. When human answers (via voice), "
                    "[HUMAN ANSWERED] appears in context. "
                    "Use ONLY for: safety decisions, scope expansion, genuine uncertainty."),
                params={"question": "str: the question for the human commander"},
                execute=self._ask_human),

            "notify_human": Tool(
                description="Send a one-way status message to GCS. No reply expected.",
                params={"message": "str: status message for GCS"},
                execute=self._notify_human),
        })

    def _get_wingman_situation(self, params: dict) -> str:
        with self.ros.lock:
            sit = self.ros.own_situation
        if not sit:
            return "Wingman position: unknown (situation block not available yet)"
        for line in sit.split('\n'):
            if 'wingman_pos' in line:
                return line.strip()
        return "Wingman position: not in situation block (check lead sensor aggregator)"

    def _message_wingman(self, params: dict) -> str:
        from std_msgs.msg import String as _String
        msg_type = str(params.get('msg_type', 'task')).lower()
        if msg_type not in ('task', 'status', 'reply', 'query', 'abort', 'position'):
            msg_type = 'task'
        content = str(params.get('message', '')).strip()
        if not content:
            return "Error: 'message' parameter is required."
        payload = json.dumps({
            "type": msg_type, "sender": "LEAD",
            "content": content, "order_id": None})
        msg = _String()
        msg.data = payload
        self.ros.pub_wingman_msg.publish(msg)
        return f"Sent [{msg_type}] to Wingman: '{content[:80]}'"

    def _ask_human(self, params: dict) -> str:
        """
        NON-BLOCKING ask_human (Loophole #3 fix).
        Sets _waiting_for_human = True. Agent loop will:
          - Skip SLM inference while waiting
          - Call get_situation() every 3s
          - When _on_voice sets _human_response, inject [HUMAN ANSWERED] into context
          - Timeout after HUMAN_WAIT_TIMEOUT_CYCLES (120s) and continue
        """
        from std_msgs.msg import String as _String
        question = str(params.get('question', 'Please advise.')).strip()
        q_msg = _String()
        q_msg.data = question
        self.ros.pub_clarification.publish(q_msg)
        # Signal agent loop to enter monitoring-wait mode
        self.ros._human_response    = None
        self.ros._waiting_for_human = True
        return (
            f"PENDING_HUMAN_RESPONSE: Question sent to GCS: '{question[:80]}'. "
            f"Monitoring situation while awaiting answer. "
            f"[HUMAN ANSWERED] will appear in context when response received.")

    def _notify_human(self, params: dict) -> str:
        from std_msgs.msg import String as _String
        message = str(params.get('message', '')).strip()
        if not message:
            return "Error: 'message' parameter is required."
        msg = _String()
        msg.data = f"[LEAD] {message}"
        self.ros.pub_clarification.publish(msg)
        return f"GCS notified: '{message[:80]}'"


# ─────────────────────────────────────────────────────────────────
# Wingman Tool Registry
# ─────────────────────────────────────────────────────────────────

class WingmanToolRegistry(BaseToolRegistry):
    """
    Wingman Pilot exclusive tools.
    Wingman never contacts human directly — all comms go through Lead.

    Additional ros_iface requirements:
        .pub_lead_msg  (Publisher<String> → /agent/wingman_to_lead)
        ._waiting_for_lead (bool)
        ._lead_response (str | None)
    """

    def __init__(self, ros_iface):
        super().__init__(ros_iface)
        self._register_wingman_tools()

    def _register_wingman_tools(self):
        self.tools.update({
            "message_lead": Tool(
                description=(
                    "Send a typed message to Lead agent. "
                    "msg_type: 'status' (progress update), 'reply' (answer to Lead query), "
                    "'query' (ask Lead a question)."),
                params={
                    "message":  "str: message content",
                    "msg_type": "str: status|reply|query (default: status)"},
                execute=self._message_lead),

            "ask_lead": Tool(
                description=(
                    "Ask Lead agent a question. NON-BLOCKING — returns immediately. "
                    "Agent monitors situation while waiting. "
                    "[LEAD ANSWERED] appears in context when Lead replies."),
                params={"question": "str: question for Lead agent"},
                execute=self._ask_lead),

            "notify_lead": Tool(
                description="Send a one-way status update to Lead agent.",
                params={"message": "str: status message for Lead"},
                execute=self._notify_lead),

            "follow_lead": Tool(
                description="Fly in formation behind Lead (Drone-0) with a specified horizontal separation.",
                params={"offset_m": "float: formation separation in metres (1–30)"},
                execute=self._follow_lead),
        })

    def _message_lead(self, params: dict) -> str:
        from std_msgs.msg import String as _String
        msg_type = str(params.get('msg_type', 'status')).lower()
        if msg_type not in ('status', 'reply', 'query', 'task', 'position'):
            msg_type = 'status'
        content = str(params.get('message', '')).strip()
        if not content:
            return "Error: 'message' parameter is required."
        payload = json.dumps({
            "type": msg_type, "sender": "WINGMAN",
            "content": content, "order_id": None})
        msg = _String()
        msg.data = payload
        self.ros.pub_lead_msg.publish(msg)
        return f"Sent [{msg_type}] to Lead: '{content[:80]}'"

    def _ask_lead(self, params: dict) -> str:
        """
        NON-BLOCKING ask_lead (Loophole #3 fix).
        Same pattern as ask_human: sets flag, returns PENDING sentinel,
        agent loop monitors and injects [LEAD ANSWERED] when response arrives.
        """
        from std_msgs.msg import String as _String
        question = str(params.get('question', '')).strip()
        if not question:
            return "Error: 'question' parameter is required."
        payload = json.dumps({
            "type": "query", "sender": "WINGMAN",
            "content": question, "order_id": None})
        msg = _String()
        msg.data = payload
        self.ros.pub_lead_msg.publish(msg)
        # Signal agent loop to enter monitoring-wait mode
        self.ros._lead_response    = None
        self.ros._waiting_for_lead = True
        return (
            f"PENDING_LEAD_RESPONSE: Question sent to Lead: '{question[:80]}'. "
            f"Monitoring situation. [LEAD ANSWERED] will appear when Lead replies.")

    def _notify_lead(self, params: dict) -> str:
        from std_msgs.msg import String as _String
        message = str(params.get('message', '')).strip()
        if not message:
            return "Error: 'message' parameter is required."
        payload = json.dumps({
            "type": "status", "sender": "WINGMAN",
            "content": message, "order_id": None})
        msg = _String()
        msg.data = payload
        self.ros.pub_lead_msg.publish(msg)
        return f"Lead notified: '{message[:80]}'"

    def _follow_lead(self, params: dict) -> str:
        offset = float(params.get('offset_m', 5.0))
        offset = max(1.0, min(30.0, offset))
        self._publish_intent({
            'action': 'follow_lead', 'offset_m': offset, 'confidence': 'high'})
        return f"Formation follow activated. Tracking Lead with {offset}m offset."
EOF
```

---

## 3.6 Context Manager (`common/context_manager.py`)

> **Loophole #8 fixes applied:**
> - `MAX_HISTORY` raised to 12 (was 8)
> - `COMPRESS_BATCH` raised to 6 (was 4)
> - Compression result truncation raised to 100 chars (was 40)
> - Critical detections auto-flagged to `memory_block`

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/context_manager.py
"""
Context Manager — keeps the SLM context window bounded.

Token budget (2048 context):
  [MISSION GOAL]        ~30 tokens
  [CURRENT SITUATION]   ~120 tokens (includes wingman_pos)
  [MEMORY]              ~150 tokens (compressed history + critical flags)
  [MESSAGES]            ~60 tokens
  [RECENT ACTIONS]      12 entries × ~60 tok = ~720 tokens
  [NEXT ACTION]         ~10 tokens
  System prompt:        ~300 tokens
  Output:               ~50 tokens
  Total:                ~1440 tokens — fits comfortably in 2048

When RECENT ACTIONS exceeds MAX_HISTORY entries, the oldest COMPRESS_BATCH
entries are condensed into a one-line "Earlier: ..." summary and moved to
[MEMORY]. Critical detections are also flagged to [MEMORY] automatically.
"""

MAX_HISTORY    = 12   # max tool-call entries before compression (was 8)
COMPRESS_BATCH =  6   # how many to compress at once (was 4)

# Results containing these keywords are auto-copied to memory_block
# so they survive context compression (Loophole #8 fix)
CRITICAL_KEYWORDS = (
    'person', 'detected', 'obstacle', 'SAFETY', 'CRITICAL',
    'battery', 'low battery', 'GPS NO', 'error', 'collision',
    'PENDING_HUMAN_RESPONSE', 'PENDING_LEAD_RESPONSE',
)


class ContextManager:

    def __init__(self):
        self.goal         = ""    # current mission goal string
        self.situation    = ""    # latest situation block
        self.memory_block = ""    # compressed history + recalled facts
        self.inter_agent  = []    # recent inter-agent messages (max 5)
        self.history      = []    # list of {tool, params_str, result} dicts

    # ── Update methods ────────────────────────────────────────────

    def set_goal(self, goal: str):
        self.goal = goal

    def update_situation(self, situation: str):
        self.situation = situation

    def add_inter_agent_message(self, source: str, content: str):
        self.inter_agent.append(f"[{source}] {content}")
        if len(self.inter_agent) > 5:
            self.inter_agent.pop(0)

    def add_memory_note(self, note: str):
        """Inject a recalled fact or important event into the memory block."""
        self.memory_block = f"{note}\n{self.memory_block}".strip()
        if len(self.memory_block) > 800:
            self.memory_block = self.memory_block[:800] + "…"

    def add_tool_result(self, tool: str, params: dict, result: str):
        """Record a completed tool call. Compress if history is full."""
        params_str = ", ".join(
            f"{k}={str(v)[:20]}" for k, v in params.items()
        ) if params else ""

        # Auto-flag critical results to memory block so they survive compression
        if any(kw in result for kw in CRITICAL_KEYWORDS):
            self.add_memory_note(f"[CRITICAL] {tool}→{result[:120]}")

        self.history.append({
            "tool":       tool,
            "params_str": params_str,
            "result":     result[:150],   # cap each result to 150 chars
        })

        if len(self.history) > MAX_HISTORY:
            self.compress_history()

    def clear_history(self):
        """Call when a new mission starts."""
        self.history      = []
        self.memory_block = ""
        self.inter_agent  = []
        self.goal         = ""

    # ── Compression ───────────────────────────────────────────────

    def compress_history(self):
        """Condense the oldest COMPRESS_BATCH entries into a one-line summary."""
        batch = self.history[:COMPRESS_BATCH]
        self.history = self.history[COMPRESS_BATCH:]
        parts = []
        for e in batch:
            r = e['result'][:100].replace('\n', ' ')   # 100 chars (was 40)
            if e['params_str']:
                parts.append(f"{e['tool']}({e['params_str'][:30]})→{r}")
            else:
                parts.append(f"{e['tool']}()→{r}")
        compressed = "Earlier: " + " | ".join(parts)
        self.memory_block = (compressed + "\n" + self.memory_block).strip()
        if len(self.memory_block) > 900:
            self.memory_block = self.memory_block[:900] + "…"

    # ── Prompt building ───────────────────────────────────────────

    def build_prompt(self) -> str:
        """Assemble the complete context prompt for the next SLM inference."""
        parts = []

        if self.goal:
            parts.append(f"[MISSION GOAL]\n{self.goal}")

        if self.situation:
            parts.append(f"[CURRENT SITUATION]\n{self.situation}")

        if self.memory_block:
            parts.append(f"[MEMORY]\n{self.memory_block}")

        if self.inter_agent:
            parts.append(
                "[MESSAGES FROM OTHER AGENT]\n" + "\n".join(self.inter_agent))

        if self.history:
            lines = []
            for e in self.history:
                call_str = (
                    f"→ {e['tool']}({e['params_str']})"
                    if e['params_str']
                    else f"→ {e['tool']}()")
                lines.append(call_str)
                lines.append(f"← {e['result']}")
            parts.append("[RECENT ACTIONS]\n" + "\n".join(lines))

        parts.append("[NEXT ACTION] Output exactly one JSON tool call:")
        return "\n\n".join(parts)
EOF
```

---

## 3.7 Agent Memory (`common/agent_memory.py`)

> **Loophole #14 fix applied:** Persistent SQLite connection with WAL mode instead of opening/closing per query.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/agent_memory.py
"""
Agent Memory — SQLite-backed long-term remember/recall.

Fixes applied (Loophole #14):
  - Persistent connection (self.conn) — opened once, closed on __del__
  - WAL journal mode for concurrent reads without blocking writes
  - Thread-safe with threading.Lock

One DB file per drone role (lead vs wingman), persisted at ~/.ros/<db_name>.
Survives node restarts.
"""
import sqlite3
import threading
import time
import os


class AgentMemory:

    def __init__(self, db_name: str = "lead_agent_memory.db"):
        db_dir = os.path.expanduser("~/.ros")
        os.makedirs(db_dir, exist_ok=True)
        self.db_path = os.path.join(db_dir, db_name)
        self._lock = threading.Lock()

        # Persistent connection — opened once (Loophole #14 fix)
        self.conn = sqlite3.connect(
            self.db_path, check_same_thread=False, timeout=10.0)
        self.conn.execute("PRAGMA journal_mode=WAL")      # concurrent read-write
        self.conn.execute("PRAGMA synchronous=NORMAL")    # balanced durability
        self._init_db()

    def _init_db(self):
        with self._lock:
            self.conn.execute("""
                CREATE TABLE IF NOT EXISTS memory (
                    id        INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    fact      TEXT NOT NULL
                )
            """)
            self.conn.commit()

    def remember(self, fact: str):
        """Store a timestamped fact."""
        with self._lock:
            self.conn.execute(
                "INSERT INTO memory (timestamp, fact) VALUES (?, ?)",
                (time.time(), fact.strip()))
            self.conn.commit()

    def recall(self, query: str = "", limit: int = 6) -> list[str]:
        """Return up to limit facts, newest first. Optionally filter by keyword."""
        with self._lock:
            if query:
                rows = self.conn.execute(
                    "SELECT fact FROM memory WHERE fact LIKE ? "
                    "ORDER BY timestamp DESC LIMIT ?",
                    (f"%{query}%", limit)
                ).fetchall()
            else:
                rows = self.conn.execute(
                    "SELECT fact FROM memory ORDER BY timestamp DESC LIMIT ?",
                    (limit,)
                ).fetchall()
        return [r[0] for r in rows]

    def get_recent(self, n: int = 5) -> list[str]:
        return self.recall(query="", limit=n)

    def clear(self):
        """Wipe all memories — use only for testing."""
        with self._lock:
            self.conn.execute("DELETE FROM memory")
            self.conn.commit()

    def __del__(self):
        """Close DB connection on garbage collection."""
        try:
            if hasattr(self, 'conn') and self.conn:
                self.conn.close()
        except Exception:
            pass
EOF
```

---

## 3.8 Verify Common Modules

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install 2>&1 | tail -5
source install/setup.bash

python3 - << 'PYEOF'
import sys, os
sys.path.insert(0, os.path.expanduser('~/major_ws/src/major_project'))

# ── Schemas ────────────────────────────────────────────────────────
from major_project.common.schemas import (
    FlightIntent, AgentMessage, make_agent_msg,
    DronePosition, WingmanOrder, SafetyEvent
)
fi = FlightIntent(action='move', direction='north', distance=50.0, confidence='high')
assert fi.action == 'move'
print("FlightIntent OK")

am = AgentMessage(type='task', sender='LEAD', content='go north')
assert am.type == 'task'
j = make_agent_msg('status', 'WINGMAN', 'I am at N50m')
import json; d = json.loads(j)
assert d['type'] == 'status' and d['sender'] == 'WINGMAN'
print("AgentMessage OK")

# ── Context Manager ────────────────────────────────────────────────
from major_project.common.context_manager import ContextManager, MAX_HISTORY
ctx = ContextManager()
ctx.set_goal("find the football")
ctx.update_situation("bat:90% alt:0m mode:MANUAL gps:OK\nwingman_pos:(5,0) alt:0.0m")
ctx.add_tool_result("get_situation", {}, "bat:90% alt:0m mode:MANUAL")
ctx.add_tool_result("takeoff", {"altitude": 10}, "Takeoff initiated. ETA ~25s.")
prompt = ctx.build_prompt()
assert "[MISSION GOAL]" in prompt
assert "find the football" in prompt
assert "[CURRENT SITUATION]" in prompt
assert "[RECENT ACTIONS]" in prompt
print("ContextManager build_prompt OK")

# Test compression: add MAX_HISTORY+4 entries
for i in range(MAX_HISTORY + 4):
    ctx.add_tool_result(f"tool_{i}", {"x": i}, f"result_{i}")
assert len(ctx.history) <= MAX_HISTORY, f"History too long: {len(ctx.history)}"
assert "Earlier" in ctx.memory_block, "Compression not running"
print(f"Context compression OK (history={len(ctx.history)}, max={MAX_HISTORY})")

# Test critical keyword auto-flag
ctx2 = ContextManager()
ctx2.add_tool_result("search", {}, "Detected: person(91%) ahead very_close [person:ahead:very_close]")
assert "CRITICAL" in ctx2.memory_block, "Critical keyword not auto-flagged"
print("Critical auto-flag OK")

# ── Agent Memory ───────────────────────────────────────────────────
from major_project.common.agent_memory import AgentMemory
mem = AgentMemory(db_name="smoke_test.db")
mem.clear()
mem.remember("football found at N50m E0m")
mem.remember("car parked at E30m")
results = mem.recall("football")
assert len(results) == 1 and "football" in results[0]
recent = mem.get_recent(5)
assert len(recent) == 2
mem.clear()
import os as _os; _os.remove(_os.path.expanduser("~/.ros/smoke_test.db"))
print("AgentMemory OK (persistent conn + WAL)")

# ── Normaliser ─────────────────────────────────────────────────────
from major_project.common.normaliser import normalise_action, normalise_direction
assert normalise_action("take off") == "takeoff"
assert normalise_action("scan") == "search"
assert normalise_action("go home") == "rtl"
assert normalise_direction("n") == "north"
assert normalise_direction("NE") == "northeast"
print("Normaliser OK")

# ── OllamaClient import ─────────────────────────────────────────────
from major_project.common.ollama_client import OllamaClient
client = OllamaClient()
print("OllamaClient import OK")

print("\n✅ ALL COMMON MODULE TESTS PASSED")
PYEOF
```

**Expected output:**
```
FlightIntent OK
AgentMessage OK
ContextManager build_prompt OK
Context compression OK (history=12, max=12)
Critical auto-flag OK
AgentMemory OK (persistent conn + WAL)
Normaliser OK
OllamaClient import OK

✅ ALL COMMON MODULE TESTS PASSED
```
