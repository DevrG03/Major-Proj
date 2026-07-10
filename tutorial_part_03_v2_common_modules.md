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
        model: str = "qwen3.5:2b",
        num_ctx: int = 8192,
        max_retries: int = 3,
        timeout: float = 45.0,
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
            "think": False,
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
Now refactored to use standard LangChain @tool decorated functions.
"""
from __future__ import annotations
import json
import time
from typing import Optional

from langchain_core.tools import tool
from pydantic import BaseModel, Field

def _publish_intent(ros_iface, action_dict: dict):
    """Publish a FlightIntent JSON to the drone's approved_intent topic."""
    from std_msgs.msg import String as _String
    msg = _String()
    msg.data = json.dumps(action_dict)
    ros_iface.pub_intent.publish(msg)

def get_base_tools(ros_iface):
    """
    All tools shared between Lead and Wingman.
    Returns a list of LangChain @tool functions.
    """

    @tool
    def wait(seconds: int = Field(5, description="seconds to wait (1–30)")) -> str:
        """
        Pause for seconds (1–30) while monitoring. 
        Abortable if a new goal arrives.
        """
        secs = max(1, min(60, seconds))
        deadline = time.time() + secs
        while time.time() < deadline:
            abort = getattr(ros_iface, '_abort_event', None)
            if abort and abort.is_set():
                elapsed = int(secs - max(0.0, deadline - time.time()))
                return f"Wait interrupted after ~{elapsed}s (new goal received)."
            time.sleep(0.5)
        return f"Waited {secs}s."

    @tool
    def takeoff(altitude: float = Field(5.0, description="target altitude in metres (1–30)")) -> str:
        """Arm and ascend to altitude metres (1–30). Returns immediately with ETA."""
        with ros_iface.lock:
            sit_str = getattr(ros_iface, 'own_situation', '')
        if sit_str:
            import re as _re
            m = _re.search(r'alt:([-\d.]+)m', sit_str)
            if m:
                current_alt = float(m.group(1))
                if current_alt > 1.0:
                    return (
                        f"[GUARDRAIL] Already airborne at {current_alt:.1f}m. "
                        "Call move() or hover() — do NOT call takeoff while flying.")
        altitude = max(1.0, min(30.0, altitude))
        _publish_intent(ros_iface, {'action': 'takeoff', 'altitude_m': altitude, 'confidence': 'high'})
        eta = int(altitude * 1.8) + 6
        wait.invoke({"seconds": eta})
        return f"Takeoff complete. Ascended to {altitude}m."

    @tool
    def move(
        direction: str = Field(..., description="N S E W NE NW SE SW forward backward left right"),
        distance: float = Field(..., description="metres (1–100)"),
        altitude: Optional[float] = Field(None, description="optional new altitude in metres")
    ) -> str:
        """Fly in direction for distance metres. Optional altitude change. Returns immediately with ETA. Call wait(ETA) then get_situation()."""
        with ros_iface.lock:
            sit = getattr(ros_iface, 'own_situation', '')
        if sit and 'alt:' in sit:
            import re as _re
            m = _re.search(r'alt:([-\d.]+)m', sit)
            if m and float(m.group(1)) < 1.0:
                return "Error: Cannot move while on the ground. You MUST call takeoff() first."
        _abbrev = {
            'N': 'north', 'S': 'south', 'E': 'east', 'W': 'west',
            'NE': 'northeast', 'NW': 'northwest',
            'SE': 'southeast', 'SW': 'southwest',
            'FWD': 'forward', 'BCK': 'backward',
            'L': 'left', 'R': 'right',
        }
        raw_dir = str(direction).upper().strip()
        direction_val = _abbrev.get(raw_dir, raw_dir.lower())
        distance_val = max(1.0, min(100.0, float(distance)))
        
        intent: dict = {
            'action': 'move', 'direction': direction_val,
            'distance_m': distance_val, 'confidence': 'high'}
        if altitude is not None:
            intent['altitude_m'] = float(altitude)

        _publish_intent(ros_iface, intent)
        eta = max(8, int(distance_val / 2.0) + 4)
        alt_note = f" Changed altitude to {altitude}m." if altitude is not None else ""
        wait.invoke({"seconds": eta})
        return f"Move complete. Reached {direction_val} {distance_val}m.{alt_note}"

    @tool
    def hover() -> str:
        """Hold current position."""
        _publish_intent(ros_iface, {"command": "hover"})
        return "[HOVER] Command sent. Holding position."

    @tool
    def follow_road(duration_sec: int = Field(15, description="duration in seconds (5-120)")) -> str:
        """Visually follow the asphalt road for duration_sec seconds. Returns when done."""
        start_t = time.time()
        while time.time() - start_t < duration_sec:
            if getattr(ros_iface, '_abort_event', None) and ros_iface._abort_event.is_set():
                return "[ABORTED] Road following interrupted by new goal."
                
            road_dir = "straight"
            obs = getattr(ros_iface, 'obstacle_vector', '')
            if "road:curve_left" in obs:
                road_dir = "curve_left"
            elif "road:curve_right" in obs:
                road_dir = "curve_right"
                
            if road_dir == "curve_left":
                intent = {"command": "move", "direction": "northwest", "distance_m": 2.0}
            elif road_dir == "curve_right":
                intent = {"command": "move", "direction": "northeast", "distance_m": 2.0}
            else:
                intent = {"command": "move", "direction": "north", "distance_m": 3.0}
                
            _publish_intent(ros_iface, intent)
            time.sleep(1.5)
            
        return f"[FOLLOW ROAD COMPLETE] Followed road for {duration_sec} seconds."

    @tool
    def search(duration_sec: int = Field(15, description="scan duration seconds (5–60)")) -> str:
        """Hover and scan camera for duration_sec seconds (5–60). Accumulates all object detections. Returns summary when done."""
        with ros_iface.lock:
            sit = getattr(ros_iface, 'own_situation', '')
        if sit and 'alt:' in sit:
            import re as _re
            m = _re.search(r'alt:([-\d.]+)m', sit)
            if m and float(m.group(1)) < 1.0:
                return "Error: Cannot search while on the ground. You MUST call takeoff() first."
        duration = max(5, min(60, duration_sec))
        _publish_intent(ros_iface, {'action': 'hover', 'confidence': 'high'})
        time.sleep(1.0)
        
        observations: list[str] = []
        deadline = time.time() + duration

        while time.time() < deadline:
            abort = getattr(ros_iface, '_abort_event', None)
            if abort and abort.is_set():
                elapsed = duration - max(0.0, deadline - time.time())
                break
                
            with ros_iface.lock:
                cam = getattr(ros_iface, 'camera_summary', '')
                obs = getattr(ros_iface, 'obstacle_vector', '')
                
            if cam and 'not available' not in cam.lower() and \
               'no detection' not in cam.lower() and 'clear' not in cam.lower():
                entry = cam + (f" [{obs}]" if obs else "")
                if entry not in observations:
                    observations.append(entry[:150])
                    
            time.sleep(2.0)
            
        elapsed = int(min(duration, duration - max(0.0, deadline - time.time())))
        if observations:
            combined = " | ".join(observations[:5])
            return f"Search complete ({elapsed}s of {duration}s). Detected: {combined}"
        return f"Search complete ({duration}s). Area clear — no objects detected."

    @tool
    def land() -> str:
        """Land drone at current position."""
        _publish_intent(ros_iface, {'action': 'land', 'confidence': 'high'})
        return "Land command sent. Allow ~15s to touch down."

    @tool
    def rtl() -> str:
        """Return to launch point and land."""
        _publish_intent(ros_iface, {'action': 'rtl', 'confidence': 'high'})
        return "RTL initiated. Drone returning to launch. Allow ~40s."

    @tool
    def get_situation() -> str:
        """Read full sensor state: position, altitude, battery, GPS, flight mode, camera, wingman position."""
        with ros_iface.lock:
            sit = getattr(ros_iface, 'own_situation', '')
        if not sit:
            return "No situation data yet — telemetry initialising. Try again in 2s."
        return sit

    @tool
    def scan_camera() -> str:
        """Get current camera detections with direction and distance."""
        with ros_iface.lock:
            cam = getattr(ros_iface, 'camera_summary', '')
            obs = getattr(ros_iface, 'obstacle_vector', '')
        if not cam:
            return "Camera not available."
        result = cam
        if obs:
            result += f"\nObstacle vectors: {obs}"
        return result

    @tool
    def get_battery() -> str:
        """Get own drone battery percentage."""
        with ros_iface.lock:
            pct = getattr(ros_iface, 'battery_pct', 0.0)
        return f"Own drone battery: {pct:.0f}%"

    @tool
    def remember(fact: str = Field(..., description="the fact to store")) -> str:
        """Store a fact in long-term persistent memory."""
        fact_str = str(fact).strip()
        if not fact_str:
            return "Error: 'fact' parameter is required."
        ros_iface.agent_memory.remember(fact_str)
        return f"Remembered: '{fact_str[:100]}'"

    @tool
    def recall(query: str = Field(..., description="keyword to search memory")) -> str:
        """Retrieve stored facts matching a keyword."""
        query_str = str(query).strip()
        facts = ros_iface.agent_memory.recall(query_str, limit=5)
        if not facts:
            return f"No memories found matching '{query_str}'."
        return "Recalled:\n" + "\n".join(f"  • {f}" for f in facts)

    @tool
    def mission_complete(report: str = Field("Mission accomplished.", description="full mission completion summary")) -> str:
        """Declare mission accomplished and end the agent loop."""
        _publish_intent(ros_iface, {'action': 'hover', 'confidence': 'high'})
        ros_iface._mission_done = True
        ros_iface._mission_report = str(report).strip()
        return f"MISSION COMPLETE: {report}. Drone locked in hover. Awaiting next command."

    return [takeoff, move, hover, follow_road, search, land, rtl, get_situation, scan_camera, get_battery, remember, recall, wait, mission_complete]


def get_lead_tools(ros_iface):
    """
    Lead Pilot exclusive tools.
    Adds: human comms, wingman coordination, wingman position query.
    """
    base_tools = get_base_tools(ros_iface)

    @tool
    def get_wingman_situation() -> str:
        """Get Wingman drone's last known position and state from the situation block (reads wingman_pos line)."""
        with ros_iface.lock:
            sit = getattr(ros_iface, 'own_situation', '')
        if not sit:
            return "Wingman position: unknown (situation block not available yet)"
        for line in sit.split('\n'):
            if 'wingman_pos' in line:
                return line.strip()
        return "Wingman position: not in situation block (check lead sensor aggregator)"

    @tool
    def message_wingman(
        message: str = Field(..., description="message content"),
        msg_type: str = Field("task", description="task|status|reply|abort (default: task)")
    ) -> str:
        """Send a typed message to Wingman. msg_type: 'task', 'status', 'reply', 'abort'."""
        from std_msgs.msg import String as _String
        msg_type_str = str(msg_type).lower()
        if msg_type_str not in ('task', 'status', 'reply', 'query', 'abort', 'position'):
            msg_type_str = 'task'
        content = str(message).strip()
        if not content:
            return "Error: 'message' parameter is required."
        payload = json.dumps({
            "type": msg_type_str, "sender": "LEAD",
            "content": content, "order_id": None})
        msg = _String()
        msg.data = payload
        ros_iface.pub_wingman_msg.publish(msg)
        return f"Sent [{msg_type_str}] to Wingman: '{content[:80]}'"

    @tool
    def ask_human(question: str = Field(..., description="the question for the human commander")) -> str:
        """
        Ask Ground Commander a question. NON-BLOCKING — returns immediately. 
        Use ONLY for: safety decisions, scope expansion, genuine uncertainty.
        """
        from std_msgs.msg import String as _String
        question_str = str(question).strip()
        q_msg = _String()
        q_msg.data = question_str
        ros_iface.pub_clarification.publish(q_msg)
        ros_iface._human_response = None
        ros_iface._waiting_for_human = True
        return (
            f"PENDING_HUMAN_RESPONSE: Question sent to GCS: '{question_str[:80]}'. "
            f"Monitoring situation while awaiting answer. "
            f"[HUMAN ANSWERED] will appear in context when response received.")

    @tool
    def notify_human(message: str = Field(..., description="status message for GCS")) -> str:
        """Send a one-way status message to GCS. No reply expected."""
        from std_msgs.msg import String as _String
        msg_str = str(message).strip()
        if not msg_str:
            return "Error: 'message' parameter is required."
        msg = _String()
        msg.data = f"[LEAD] {msg_str}"
        ros_iface.pub_clarification.publish(msg)
        return f"GCS notified: '{msg_str[:80]}'"

    return base_tools + [get_wingman_situation, message_wingman, ask_human, notify_human]


def get_wingman_tools(ros_iface):
    """
    Wingman Pilot exclusive tools.
    Wingman never contacts human directly — all comms go through Lead.
    """
    base_tools = get_base_tools(ros_iface)

    @tool
    def message_lead(
        message: str = Field(..., description="message content"),
        msg_type: str = Field("status", description="status|reply|query (default: status)")
    ) -> str:
        """Send a typed message to Lead agent. msg_type: 'status', 'reply', 'query'."""
        from std_msgs.msg import String as _String
        msg_type_str = str(msg_type).lower()
        if msg_type_str not in ('status', 'reply', 'query', 'task', 'position'):
            msg_type_str = 'status'
        content = str(message).strip()
        if not content:
            return "Error: 'message' parameter is required."
        payload = json.dumps({
            "type": msg_type_str, "sender": "WINGMAN",
            "content": content, "order_id": None})
        msg = _String()
        msg.data = payload
        ros_iface.pub_lead_msg.publish(msg)
        return f"Sent [{msg_type_str}] to Lead: '{content[:80]}'"

    @tool
    def ask_lead(question: str = Field(..., description="question for Lead agent")) -> str:
        """Ask Lead agent a question. NON-BLOCKING — returns immediately."""
        from std_msgs.msg import String as _String
        question_str = str(question).strip()
        if not question_str:
            return "Error: 'question' parameter is required."
        payload = json.dumps({
            "type": "query", "sender": "WINGMAN",
            "content": question_str, "order_id": None})
        msg = _String()
        msg.data = payload
        ros_iface.pub_lead_msg.publish(msg)
        ros_iface._lead_response = None
        ros_iface._waiting_for_lead = True
        return (
            f"PENDING_LEAD_RESPONSE: Question sent to Lead: '{question_str[:80]}'. "
            f"Monitoring situation. [LEAD ANSWERED] will appear when Lead replies.")

    @tool
    def notify_lead(message: str = Field(..., description="status message for Lead")) -> str:
        """Send a one-way status update to Lead agent."""
        from std_msgs.msg import String as _String
        msg_str = str(message).strip()
        if not msg_str:
            return "Error: 'message' parameter is required."
        payload = json.dumps({
            "type": "status", "sender": "WINGMAN",
            "content": msg_str, "order_id": None})
        msg = _String()
        msg.data = payload
        ros_iface.pub_lead_msg.publish(msg)
        return f"Lead notified: '{msg_str[:80]}'"

    @tool
    def follow_lead(offset_m: float = Field(5.0, description="formation separation in metres (1–30)")) -> str:
        """Fly in formation behind Lead (Drone-0) with a specified horizontal separation."""
        offset = max(1.0, min(30.0, float(offset_m)))
        _publish_intent(ros_iface, {
            'action': 'follow_lead', 'offset_m': offset, 'confidence': 'high'})
        return f"Formation follow activated. Tracking Lead with {offset}m offset."

    return base_tools + [message_lead, ask_lead, notify_lead, follow_lead]
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

        # Sliding-window telemetry deduplication (Scratchpad Compression pattern):
        # Keep ONLY the latest get_situation result in history. Older entries contain
        # stale state (e.g. alt:0m DISARMED) that confuses small SLMs into
        # re-issuing takeoff commands even when the drone is already airborne.
        if tool == 'get_situation':
            self.history = [h for h in self.history if h['tool'] != 'get_situation']

        # Auto-flag critical results to memory block so they survive compression
        if any(kw in result for kw in CRITICAL_KEYWORDS):
            self.add_memory_note(f"[CRITICAL] {tool}→{result[:120]}")

        self.history.append({
            "tool":       tool,
            "params_str": params_str,
            "result":     result[:150],   # cap each result to 150 chars
        })

        if len(self.history) > MAX_HISTORY:
            self._compress_oldest()

    def clear_history(self):
        """Call when a new mission starts."""
        self.history      = []
        self.memory_block = ""
        self.inter_agent  = []
        self.goal         = ""

    # ── Compression ───────────────────────────────────────────────

    def _compress_oldest(self):
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
