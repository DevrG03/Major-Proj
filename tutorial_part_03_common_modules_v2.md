# Part 3: Common Modules (V2 - LangGraph Upgrade)

> **Target:** PC-1 (then synced to PC-2)
> **Prerequisites:** Part 2 complete — package scaffold exists at `~/major_ws/src/major_project/`

This V2 tutorial upgrades our agent foundation to use **LangGraph**. We replace custom `while` loops and manual context windows with deterministic State Graphs and LangChain-compatible Tools.

---

## 3.1 Pydantic Schemas & Graph State (`common/schemas.py`)

We add `AgentState`, the core memory structure that LangGraph passes between nodes.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/schemas.py
"""
All Pydantic v2 schemas and LangGraph State definitions.
"""
from __future__ import annotations
from typing import Optional, Literal, TypedDict, Annotated, Sequence
from pydantic import BaseModel, Field, field_validator
import operator
import json
from langchain_core.messages import BaseMessage


# ─────────────────────────────────────────────────────────────────
# LangGraph Agent State
# ─────────────────────────────────────────────────────────────────

class AgentState(TypedDict):
    """
    The state dictionary passed between LangGraph nodes.
    """
    messages: Annotated[Sequence[BaseMessage], operator.add] # Chat history / Short-term memory
    checklist: list[dict]                                    # The hierarchical task list
    current_task_index: int                                  # Which checklist item we are on
    sensor_telemetry: str                                    # Latest get_situation() output
    mission_goal: str                                        # High-level mission goal


# ─────────────────────────────────────────────────────────────────
# Flight Intent
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
    offset_m: Optional[float] = Field(None, ge=1.0, le=30.0)
    then: Optional[FlightIntent] = None
    confidence: Literal["high", "medium", "low"]
    clarification_question: Optional[str] = None

    @field_validator('direction')
    @classmethod
    def validate_direction(cls, v):
        if v is None: return v
        valid = {'north', 'south', 'east', 'west', 'northeast', 'northwest', 'southeast', 'southwest', 'forward', 'backward', 'left', 'right', 'up', 'down'}
        if v.lower() not in valid: return None
        return v.lower()

FlightIntent.model_rebuild()


# ─────────────────────────────────────────────────────────────────
# Drone position and situational awareness
# ─────────────────────────────────────────────────────────────────

class DronePosition(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0
    speed: float = 0.0


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
# Inter-agent typed message envelope
# ─────────────────────────────────────────────────────────────────

class AgentMessage(BaseModel):
    type: Literal["task", "status", "reply", "query", "abort", "position"]
    sender: Literal["LEAD", "WINGMAN"]
    content: str
    order_id: Optional[str] = None

def make_agent_msg(type: str, sender: str, content: str, order_id: Optional[str] = None) -> str:
    msg = AgentMessage(type=type, sender=sender, content=content, order_id=order_id)
    return msg.model_dump_json()

EOF
```

---

## 3.2 Ollama Client (`common/ollama_client.py`)

*(Unchanged. We keep our robust REST wrapper for local Qwen3.5:2b calls).*

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/ollama_client.py
"""
Thin wrapper around the Ollama REST API.
"""
import requests
import time
import logging

logger = logging.getLogger(__name__)

class OllamaClient:
    def __init__(self, model: str = 'qwen3.5:2b', url: str = 'http://localhost:11434/api/generate', num_ctx: int = 4096):
        self.model = model
        self.url = url
        self.num_ctx = num_ctx
        self.timeout = 180.0
        self.max_retries = 3

    def infer(self, prompt: str, system: str, schema: dict | None = None) -> tuple[str | None, float]:
        payload = {
            "model": self.model, "prompt": prompt, "system": system,
            "stream": False, "think": False,
            "options": {"num_ctx": self.num_ctx, "temperature": 0}
        }
        if schema:
            payload["format"] = schema
            
        for attempt in range(self.max_retries):
            t_start = time.perf_counter()
            try:
                response = requests.post(self.url, json=payload, timeout=self.timeout)
                latency = time.perf_counter() - t_start
                if response.status_code == 200:
                    raw = response.json().get("response", "").strip()
                    if raw:
                        return raw, latency
            except Exception as e:
                print(f"[OllamaClient] Inference attempt {attempt+1} failed: {e}")
            time.sleep(0.5)
        return None, 0.0
EOF
```

---

## 3.3 Confidence Gate (`common/confidence_gate.py`)

*(Unchanged)*

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/confidence_gate.py
from enum import Enum

class LeadAction(Enum):
    EXECUTE = "execute"
    EXECUTE_WITH_WARNING = "execute_with_warning"
    WITHHOLD_CLARIFY_HUMAN = "withhold_clarify_human"

class WingmanAction(Enum):
    EXECUTE = "execute"
    EXECUTE_WITH_WARNING = "execute_with_warning"
    CLARIFY_LEAD = "clarify_lead"

def gate_lead(confidence: str) -> LeadAction:
    if confidence == "high": return LeadAction.EXECUTE
    elif confidence == "medium": return LeadAction.EXECUTE_WITH_WARNING
    else: return LeadAction.WITHHOLD_CLARIFY_HUMAN

def gate_wingman(confidence: str) -> WingmanAction:
    if confidence == "high": return WingmanAction.EXECUTE
    elif confidence == "medium": return WingmanAction.EXECUTE_WITH_WARNING
    else: return WingmanAction.CLARIFY_LEAD
EOF
```

---

## 3.4 Agent Memory (`common/agent_memory.py`)

*(Unchanged. SQLite WAL mode memory for Long-Term storage).*

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/agent_memory.py
"""
Agent Memory — SQLite-backed long-term remember/recall.
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
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False, timeout=10.0)
        self.conn.execute("PRAGMA journal_mode=WAL")
        self._init_db()

    def _init_db(self):
        with self._lock:
            self.conn.execute("""
                CREATE TABLE IF NOT EXISTS memory (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL, fact TEXT NOT NULL)
            """)
            self.conn.commit()

    def remember(self, fact: str):
        with self._lock:
            self.conn.execute("INSERT INTO memory (timestamp, fact) VALUES (?, ?)", (time.time(), fact.strip()))
            self.conn.commit()

    def recall(self, query: str = "", limit: int = 6) -> list[str]:
        with self._lock:
            if query:
                rows = self.conn.execute("SELECT fact FROM memory WHERE fact LIKE ? ORDER BY timestamp DESC LIMIT ?", (f"%{query}%", limit)).fetchall()
            else:
                rows = self.conn.execute("SELECT fact FROM memory ORDER BY timestamp DESC LIMIT ?", (limit,)).fetchall()
        return [r[0] for r in rows]

    def clear(self):
        with self._lock:
            self.conn.execute("DELETE FROM memory")
            self.conn.commit()
EOF
```

---

## 3.5 Tool Registry (`common/tool_registry.py`)

**MAJOR UPDATE**: All tools are now converted to LangChain `StructuredTool` objects. This allows LangGraph executors to seamlessly call them based on the Planner's JSON output.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/tool_registry.py
"""
Tool Registry — Upgraded to LangChain StructuredTools for LangGraph integration.
"""
from __future__ import annotations
import json
import time
from typing import Callable, Any
from pydantic import BaseModel, Field
from langchain_core.tools import StructuredTool

# ── Parameter Schemas ──────────────────────────────────────────

class TakeoffInput(BaseModel):
    altitude: float = Field(..., description="Target altitude in metres (1-30)")

class MoveInput(BaseModel):
    direction: str = Field(..., description="N, S, E, W, NE, NW, SE, SW, forward, backward, left, right")
    distance: float = Field(..., description="Distance in metres (1-100)")
    altitude: float | None = Field(None, description="Optional new altitude")

class FollowRoadInput(BaseModel):
    duration_sec: int = Field(15, description="Duration to follow the road in seconds (5-120)")

class SearchInput(BaseModel):
    duration_sec: int = Field(15, description="Scan duration in seconds (5-60)")

class NotifyInput(BaseModel):
    message: str = Field(..., description="Message content")

class MemoryInput(BaseModel):
    fact: str = Field(..., description="Fact to remember")

class EmptyInput(BaseModel):
    pass


# ─────────────────────────────────────────────────────────────────
# Base Tool Registry
# ─────────────────────────────────────────────────────────────────

class BaseToolRegistry:
    def __init__(self, ros_iface):
        self.ros = ros_iface
        self.tools_list: list[StructuredTool] = []
        self._register_base_tools()

    def _register_base_tools(self):
        self.tools_list.extend([
            StructuredTool.from_function(func=self._takeoff, name="takeoff", description="Arm and ascend.", args_schema=TakeoffInput),
            StructuredTool.from_function(func=self._move, name="move", description="Fly in a direction.", args_schema=MoveInput),
            StructuredTool.from_function(func=self._hover, name="hover", description="Hold position.", args_schema=EmptyInput),
            StructuredTool.from_function(func=self._follow_road, name="follow_road", description="Visually follow road.", args_schema=FollowRoadInput),
            StructuredTool.from_function(func=self._search, name="search", description="Hover and scan camera.", args_schema=SearchInput),
            StructuredTool.from_function(func=self._land, name="land", description="Land drone.", args_schema=EmptyInput),
            StructuredTool.from_function(func=self._rtl, name="rtl", description="Return to launch.", args_schema=EmptyInput),
            StructuredTool.from_function(func=self._get_situation, name="get_situation", description="Read sensors.", args_schema=EmptyInput),
            StructuredTool.from_function(func=self._remember, name="remember", description="Store a fact in memory.", args_schema=MemoryInput),
            StructuredTool.from_function(func=self._mission_complete, name="mission_complete", description="End mission.", args_schema=NotifyInput),
        ])

    def get_tools(self) -> list[StructuredTool]:
        return self.tools_list

    def _publish_intent(self, action_dict: dict):
        from std_msgs.msg import String as _String
        msg = _String()
        msg.data = json.dumps(action_dict)
        self.ros.pub_intent.publish(msg)

    def _takeoff(self, altitude: float) -> str:
        altitude = max(1.0, min(30.0, altitude))
        self._publish_intent({'action': 'takeoff', 'altitude_m': altitude, 'confidence': 'high'})
        time.sleep(4)
        return f"Takeoff complete. Ascended to {altitude}m."

    def _move(self, direction: str, distance: float, altitude: float = None) -> str:
        dir_map = {
            'n': 'north', 's': 'south', 'e': 'east', 'w': 'west',
            'ne': 'northeast', 'nw': 'northwest', 'se': 'southeast', 'sw': 'southwest'
        }
        d_lower = direction.lower()
        d_norm = dir_map.get(d_lower, d_lower)

        intent = {'action': 'move', 'direction': d_norm, 'distance_m': distance, 'confidence': 'high'}
        if altitude is not None:
            intent['altitude_m'] = float(altitude)
        self._publish_intent(intent)
        time.sleep(5)
        return f"Move complete. Reached {d_norm} {distance}m."

    def _hover(self) -> str:
        self._publish_intent({"action": "hover", "confidence": "high"})
        return "[HOVER] Command sent. Holding position."

    def _follow_road(self, duration_sec: int) -> str:
        start_t = time.time()
        while time.time() - start_t < duration_sec:
            if getattr(self.ros, '_abort_event', None) and self.ros._abort_event.is_set():
                return "[ABORTED] Road following interrupted."
            road_dir = "straight"
            if "road:curve_left" in self.ros.obstacle_vector: road_dir = "curve_left"
            elif "road:curve_right" in self.ros.obstacle_vector: road_dir = "curve_right"
            
            if road_dir == "curve_left": intent = {"action": "move", "direction": "northwest", "distance_m": 2.0}
            elif road_dir == "curve_right": intent = {"action": "move", "direction": "northeast", "distance_m": 2.0}
            else: intent = {"action": "move", "direction": "north", "distance_m": 3.0}
                
            self._publish_intent(intent)
            time.sleep(1.5)
        return f"[FOLLOW ROAD COMPLETE] Followed road for {duration_sec} seconds."

    def _search(self, duration_sec: int) -> str:
        self._publish_intent({'action': 'hover', 'confidence': 'high'})
        time.sleep(1.0)
        observations = []
        deadline = time.time() + duration_sec
        aborted = False
        while time.time() < deadline:
            if getattr(self.ros, '_abort_event', None) and self.ros._abort_event.is_set():
                aborted = True
                break
            with self.ros.lock:
                cam = self.ros.camera_summary
                obs = self.ros.obstacle_vector
            if cam and 'clear' not in cam.lower():
                entry = cam + (f" [{obs}]" if obs else "")
                if entry not in observations: observations.append(entry[:150])
            time.sleep(2.0)
        if aborted:
            return "[ABORTED] Search interrupted."
        if observations:
            return f"Search complete. Detected: {' | '.join(observations[:5])}"
        return "Search complete. Area clear."

    def _land(self) -> str:
        self._publish_intent({'action': 'land', 'confidence': 'high'})
        return "Land command sent."

    def _rtl(self) -> str:
        self._publish_intent({'action': 'rtl', 'confidence': 'high'})
        return "RTL initiated."

    def _get_situation(self) -> str:
        with self.ros.lock: return self.ros.own_situation or "No data."

    def _remember(self, fact: str) -> str:
        self.ros.agent_memory.remember(fact)
        return f"Remembered: '{fact}'"

    def _mission_complete(self, message: str) -> str:
        self._publish_intent({'action': 'hover', 'confidence': 'high'})
        self.ros._mission_done = True
        return f"MISSION COMPLETE: {message}"


# ── Parameter Schemas for Lead/Wingman ─────────────────────────

class MessageWingmanInput(BaseModel):
    message: str = Field(..., description="Message content")
    msg_type: str = Field("task", description="task|status|reply|abort")

class AskHumanInput(BaseModel):
    question: str = Field(..., description="Question for GCS")

class MessageLeadInput(BaseModel):
    message: str = Field(..., description="Message content")
    msg_type: str = Field("status", description="status|reply|query")

class AskLeadInput(BaseModel):
    question: str = Field(..., description="Question for Lead agent")

class FollowLeadInput(BaseModel):
    offset_m: float = Field(5.0, description="Formation separation in metres (1-30)")


class LeadToolRegistry(BaseToolRegistry):
    def _register_base_tools(self):
        super()._register_base_tools()
        self.tools_list.extend([
            StructuredTool.from_function(func=self._get_wingman_situation, name="get_wingman_situation", description="Get Wingman state.", args_schema=EmptyInput),
            StructuredTool.from_function(func=self._message_wingman, name="message_wingman", description="Send msg to Wingman.", args_schema=MessageWingmanInput),
            StructuredTool.from_function(func=self._ask_human, name="ask_human", description="Ask GCS a question.", args_schema=AskHumanInput),
            StructuredTool.from_function(func=self._notify_human, name="notify_human", description="Notify GCS.", args_schema=NotifyInput),
        ])

    def _get_wingman_situation(self) -> str:
        with self.ros.lock:
            sit = self.ros.own_situation
        if sit:
            for line in sit.split('\n'):
                if 'wingman_pos' in line: return line.strip()
        return "Wingman position: unknown."

    def _message_wingman(self, message: str, msg_type: str = "task") -> str:
        from std_msgs.msg import String as _String
        payload = json.dumps({"type": msg_type, "sender": "LEAD", "content": message, "order_id": None})
        msg = _String()
        msg.data = payload
        self.ros.pub_wingman_msg.publish(msg)
        return f"Sent [{msg_type}] to Wingman."

    def _ask_human(self, question: str) -> str:
        from std_msgs.msg import String as _String
        q_msg = _String()
        q_msg.data = question
        self.ros.pub_clarification.publish(q_msg)
        self.ros._human_response = None
        self.ros._waiting_for_human = True
        return f"PENDING_HUMAN_RESPONSE: Question sent to GCS."

    def _notify_human(self, message: str) -> str:
        from std_msgs.msg import String as _String
        msg = _String()
        msg.data = f"[LEAD] {message}"
        self.ros.pub_clarification.publish(msg)
        return f"GCS notified."


class WingmanToolRegistry(BaseToolRegistry):
    def _register_base_tools(self):
        super()._register_base_tools()
        self.tools_list.extend([
            StructuredTool.from_function(func=self._message_lead, name="message_lead", description="Send msg to Lead.", args_schema=MessageLeadInput),
            StructuredTool.from_function(func=self._ask_lead, name="ask_lead", description="Ask Lead a question.", args_schema=AskLeadInput),
            StructuredTool.from_function(func=self._notify_lead, name="notify_lead", description="Notify Lead.", args_schema=NotifyInput),
            StructuredTool.from_function(func=self._follow_lead, name="follow_lead", description="Follow Lead.", args_schema=FollowLeadInput),
        ])

    def _message_lead(self, message: str, msg_type: str = "status") -> str:
        from std_msgs.msg import String as _String
        payload = json.dumps({"type": msg_type, "sender": "WINGMAN", "content": message, "order_id": None})
        msg = _String()
        msg.data = payload
        self.ros.pub_lead_msg.publish(msg)
        return f"Sent [{msg_type}] to Lead."

    def _ask_lead(self, question: str) -> str:
        from std_msgs.msg import String as _String
        payload = json.dumps({"type": "query", "sender": "WINGMAN", "content": question, "order_id": None})
        msg = _String()
        msg.data = payload
        self.ros.pub_lead_msg.publish(msg)
        self.ros._lead_response = None
        self.ros._waiting_for_lead = True
        return f"PENDING_LEAD_RESPONSE: Question sent to Lead."

    def _notify_lead(self, message: str) -> str:
        from std_msgs.msg import String as _String
        payload = json.dumps({"type": "status", "sender": "WINGMAN", "content": message, "order_id": None})
        msg = _String()
        msg.data = payload
        self.ros.pub_lead_msg.publish(msg)
        return f"Lead notified."

    def _follow_lead(self, offset_m: float) -> str:
        offset_m = max(1.0, min(30.0, offset_m))
        self._publish_intent({'action': 'follow_lead', 'offset_m': offset_m, 'confidence': 'high'})
        return f"Formation follow activated with {offset_m}m offset."
EOF
```

---

## 3.6 Verify Common Modules V2

Let's ensure the new LangGraph-ready tools load properly.

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install 2>&1 | tail -5
source install/setup.bash

python3 - << 'PYEOF'
import sys, os
sys.path.insert(0, os.path.expanduser('~/major_ws/src/major_project'))

from major_project.common.tool_registry import BaseToolRegistry

# Create dummy mock ROS interface
class DummyRos:
    import threading
    lock = threading.Lock()
    own_situation = "Mock situation"
    camera_summary = ""
    obstacle_vector = ""

ros = DummyRos()
registry = BaseToolRegistry(ros)

# Verify LangChain Tool Generation
tools = registry.get_tools()
assert any(t.name == "takeoff" for t in tools)
assert any(t.name == "follow_road" for t in tools)

print("\n✅ LANGGRAPH V2 COMMON MODULES PASSED")
PYEOF
```
