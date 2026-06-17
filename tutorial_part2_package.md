# Multi-Drone SLM Pilot System — Tutorial Part 2
## ROS2 Package, Schemas, and Common Utilities

> Continues from Part 1. Both PCs have Ubuntu 26 + ROS2 Lyrical + PX4 + Ollama running.

---

## PART 7: Create the ROS2 Package

### 7.1 Scaffold the Package

```bash
# [PC-1]
cd ~/major_ws/src
ros2 pkg create major_project \
  --build-type ament_python \
  --dependencies rclpy px4_msgs std_msgs sensor_msgs geometry_msgs
```

### 7.2 Create Directory Structure

```bash
# [PC-1]
cd ~/major_ws/src/major_project/major_project

mkdir -p common
mkdir -p lead_pilot
mkdir -p wingman_pilot
mkdir -p gcs
mkdir -p benchmark

# Create __init__.py in every directory
touch common/__init__.py
touch lead_pilot/__init__.py
touch wingman_pilot/__init__.py
touch gcs/__init__.py

# Create placeholder files
touch common/schemas.py
touch common/normaliser.py
touch common/confidence_gate.py
touch common/ollama_client.py
touch lead_pilot/lead_nlu_node.py
touch lead_pilot/lead_px4_commander_node.py
touch lead_pilot/lead_sensor_aggregator_node.py
touch lead_pilot/lead_intent_bridge_node.py
touch wingman_pilot/wingman_nlu_node.py
touch wingman_pilot/wingman_px4_commander_node.py
touch wingman_pilot/wingman_sensor_aggregator_node.py
touch gcs/stt_node.py
touch gcs/clarification_speaker_node.py
touch gcs/mission_monitor_node.py
touch gcs/emergency_stop_node.py
touch gcs/camera_detection_node.py

cd ~/major_ws/src/major_project
mkdir -p launch config
touch launch/lead_pilot.launch.py
touch launch/wingman_pilot.launch.py
touch config/lead_config.yaml
touch config/wingman_config.yaml
```

### 7.3 Edit setup.py

```bash
# [PC-1]
cat << 'EOF' > ~/major_ws/src/major_project/setup.py
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'major_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Devrajsinh Gohil',
    maintainer_email='202511004@dau.ac.in',
    description='Rank-based multi-SLM drone pilot system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stt_node = major_project.gcs.stt_node:main',
            'clarification_speaker = major_project.gcs.clarification_speaker_node:main',
            'mission_monitor = major_project.gcs.mission_monitor_node:main',
            'emergency_stop = major_project.gcs.emergency_stop_node:main',
            'camera_detection = major_project.gcs.camera_detection_node:main',
            'lead_nlu = major_project.lead_pilot.lead_nlu_node:main',
            'lead_px4_commander = major_project.lead_pilot.lead_px4_commander_node:main',
            'lead_sensor_aggregator = major_project.lead_pilot.lead_sensor_aggregator_node:main',
            'lead_intent_bridge = major_project.lead_pilot.lead_intent_bridge_node:main',
            'wingman_nlu = major_project.wingman_pilot.wingman_nlu_node:main',
            'wingman_px4_commander = major_project.wingman_pilot.wingman_px4_commander_node:main',
            'wingman_sensor_aggregator = major_project.wingman_pilot.wingman_sensor_aggregator_node:main',
        ],
    },
)
EOF
```

---

## PART 8: Common Schemas (Pydantic v2)

### 8.1 Write schemas.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/schemas.py
"""
All Pydantic v2 schemas for the multi-drone SLM pilot system.
FlightIntent: ported from minor project (unchanged).
WingmanOrder, StatusReport, SituationalAwareness: new for major project.
"""
from __future__ import annotations
from typing import Optional, Literal
from pydantic import BaseModel, Field, field_validator
import json


# ─────────────────────────────────────────────
# Existing schema from minor project
# ─────────────────────────────────────────────

class FlightIntent(BaseModel):
    action: Literal[
        "takeoff", "move", "hover", "land", "rtl",
        "search", "search_stop", "search_resume", "search_expand",
        "hold", "follow_lead"
    ]
    altitude: Optional[float] = Field(None, ge=0.5, le=50.0,
        description="Target altitude in metres (0.5–50)")
    distance: Optional[float] = Field(None, ge=0.1, le=100.0,
        description="Distance to travel in metres (0.1–100)")
    direction: Optional[str] = None
    speed: Optional[float] = Field(None, ge=0.1, le=10.0,
        description="Speed in m/s (0.1–10)")
    heading: Optional[float] = Field(None, ge=0.0, le=360.0,
        description="Target heading in degrees (0–360)")
    then: Optional[FlightIntent] = None
    confidence: Literal["high", "medium", "low"]
    clarification_question: Optional[str] = None

    @field_validator('direction')
    @classmethod
    def validate_direction(cls, v):
        if v is None:
            return v
        valid = {'north','south','east','west','northeast','northwest',
                 'southeast','southwest','forward','backward','left','right','up','down'}
        if v.lower() not in valid:
            return None
        return v.lower()


# Pydantic v2 requires model_rebuild() for self-referential types (then: Optional["FlightIntent"])
FlightIntent.model_rebuild()


# ─────────────────────────────────────────────
# New schemas for multi-drone major project
# ─────────────────────────────────────────────

class DronePosition(BaseModel):
    x: float = 0.0          # metres, NED frame
    y: float = 0.0
    z: float = 0.0           # negative = up in NED
    heading: float = 0.0     # degrees, 0=north
    speed: float = 0.0       # m/s


class SituationalAwareness(BaseModel):
    """Structured sensor state per drone. Injected into SLM prompt as text."""
    drone_id: str            # "LEAD" or "WINGMAN"
    position: DronePosition
    battery_pct: float = Field(ge=0.0, le=100.0)
    flight_mode: str         # PX4 mode string
    gps_fix: bool = True
    altitude_baro: float = 0.0
    camera_summary: str = "No camera data"

    def to_prompt_block(self) -> str:
        pos = self.position
        return (
            f"[DRONE | {self.drone_id}] "
            f"pos:({pos.x:.1f},{pos.y:.1f},{pos.z:.1f}m) "
            f"hdg:{pos.heading:.0f}° spd:{pos.speed:.1f}m/s "
            f"bat:{self.battery_pct:.0f}% mode:{self.flight_mode} "
            f"baro:{self.altitude_baro:.1f}m gps:{'OK' if self.gps_fix else 'NO'}\n"
            f"[CAMERA | {self.drone_id}] {self.camera_summary}"
        )


class WingmanOrder(BaseModel):
    """Lead Pilot → Wingman. Schema-validated command. Never free-form NL only."""
    order_id: Optional[str] = None  # SLM may omit; Lead NLU assigns if missing
    mission_context: str     # NL brief: WHY this order was issued
    intent: FlightIntent     # the actual structured command
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
            + (f"\nLead asks: {self.clarification_question}" if self.clarification_question else "")
        )


class StatusReport(BaseModel):
    """Wingman → Lead. Reports execution status and current situation."""
    order_id: str
    status: Literal[
        "acknowledged", "executing", "completed", "failed", "needs_clarification"
    ]
    drone_position: DronePosition
    battery_pct: float = Field(ge=0.0, le=100.0)
    obstacle_detected: bool = False
    obstacle_description: Optional[str] = None
    situation_summary: str   # NL summary for lead's context window
    clarification_question: Optional[str] = None
    confidence: Literal["high", "medium", "low"]

    def to_prompt_block(self) -> str:
        return (
            f"[WINGMAN REPORT | order:{self.order_id} status:{self.status.upper()}]\n"
            f"{self.situation_summary}"
            + (f"\nWingman asks: {self.clarification_question}" if self.clarification_question else "")
            + (f"\nObstacle: {self.obstacle_description}" if self.obstacle_detected else "")
        )


class LeadOutput(BaseModel):
    """Full JSON output from Lead SLM per inference cycle."""
    my_intent: Optional[FlightIntent] = None
    wingman_order: Optional[WingmanOrder] = None
    confidence: Literal["high", "medium", "low"]
    situation_report: str    # NL summary for GCS display (the "radio chatter")
    clarification_question: Optional[str] = None


class WingmanOutput(BaseModel):
    """Full JSON output from Wingman SLM per inference cycle."""
    intent: Optional[FlightIntent] = None
    confidence: Literal["high", "medium", "low"]
    situation_summary: str
    clarification_question: Optional[str] = None


# ─────────────────────────────────────────────
# Serialisation helpers
# ─────────────────────────────────────────────

def parse_lead_output(raw_json: str) -> Optional[LeadOutput]:
    """Parse and validate Lead SLM output. Returns None on failure."""
    try:
        data = json.loads(raw_json)
        return LeadOutput(**data)
    except Exception:
        return None

def parse_wingman_output(raw_json: str) -> Optional[WingmanOutput]:
    """Parse and validate Wingman SLM output. Returns None on failure."""
    try:
        data = json.loads(raw_json)
        return WingmanOutput(**data)
    except Exception:
        return None
EOF
```

### 8.2 Write normaliser.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/normaliser.py
"""
Normalises SLM output to canonical action names.
The SLM often uses variant spellings; this maps them all to schema values.
"""

ACTION_ALIASES: dict[str, str] = {
    # takeoff variants
    "take_off": "takeoff", "take off": "takeoff", "launch": "takeoff",
    "liftoff": "takeoff", "lift_off": "takeoff", "ascend": "takeoff",
    "go up": "takeoff", "fly up": "takeoff",
    # move variants
    "fly": "move", "go": "move", "navigate": "move", "travel": "move",
    "proceed": "move", "advance": "move", "translate": "move",
    # hover variants
    "stop": "hover", "halt": "hover", "stay": "hover", "wait": "hover",
    "hold": "hover", "pause": "hover", "maintain": "hover",
    # land variants
    "landing": "land", "touch down": "land", "touchdown": "land",
    "descend and land": "land", "set down": "land",
    # rtl variants
    "return": "rtl", "come back": "rtl", "go home": "rtl",
    "return to home": "rtl", "return to launch": "rtl", "rth": "rtl",
    # search variants
    "scan": "search", "survey": "search", "look": "search",
    "inspect": "search", "investigate": "search", "patrol": "search",
    "recon": "search", "reconnaissance": "search",
    # hold (wingman)
    "hold position": "hold", "hold_position": "hold",
    "stay in place": "hold", "remain": "hold",
    # follow
    "follow": "follow_lead", "trail": "follow_lead", "shadow": "follow_lead",
}

DIRECTION_ALIASES: dict[str, str] = {
    "n": "north", "s": "south", "e": "east", "w": "west",
    "ne": "northeast", "nw": "northwest", "se": "southeast", "sw": "southwest",
    "fwd": "forward", "back": "backward", "bwd": "backward",
    "ahead": "forward", "behind": "backward",
}

def normalise_action(action: str) -> str:
    """Map any action alias to its canonical schema value."""
    if action is None:
        return "hover"
    cleaned = action.strip().lower().replace("-", " ").replace("_", " ")
    # Direct canonical match
    canonical = {
        "takeoff", "move", "hover", "land", "rtl",
        "search", "search_stop", "search_resume", "search_expand",
        "hold", "follow_lead"
    }
    cleaned_underscore = cleaned.replace(" ", "_")
    if cleaned_underscore in canonical:
        return cleaned_underscore
    # Alias lookup
    return ACTION_ALIASES.get(cleaned, ACTION_ALIASES.get(cleaned_underscore, "hover"))

def normalise_direction(direction: str) -> str:
    if direction is None:
        return None
    cleaned = direction.strip().lower()
    return DIRECTION_ALIASES.get(cleaned, cleaned)

def normalise_parsed(data: dict) -> dict:
    """
    Apply all normalisations to a raw SLM output dict before Pydantic validation.
    Call this BEFORE passing to FlightIntent(**data).
    """
    if "action" in data:
        data["action"] = normalise_action(data["action"])
    if "direction" in data:
        data["direction"] = normalise_direction(data["direction"])
    # Normalise nested 'then' chain recursively
    if "then" in data and isinstance(data["then"], dict):
        data["then"] = normalise_parsed(data["then"])
    return data
EOF
```

### 8.3 Write confidence_gate.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/confidence_gate.py
"""
Confidence gate policy for both Lead and Wingman pilots.
Implements the two-level cascade described in the architecture doc.
"""
from enum import Enum

class LeadAction(Enum):
    EXECUTE = "execute"
    EXECUTE_WITH_WARNING = "execute_with_warning"
    WITHHOLD_CLARIFY_HUMAN = "withhold_clarify_human"

class WingmanAction(Enum):
    EXECUTE = "execute"
    EXECUTE_WITH_WARNING = "execute_with_warning"
    CLARIFY_LEAD = "clarify_lead"   # wingman never contacts human directly

def gate_lead(confidence: str) -> LeadAction:
    """
    Lead Pilot gate:
      high   → execute my_intent + send wingman_order immediately
      medium → execute + warn GCS, send wingman_order with warning
      low    → withhold, request clarification from Human Commander
    """
    if confidence == "high":
        return LeadAction.EXECUTE
    elif confidence == "medium":
        return LeadAction.EXECUTE_WITH_WARNING
    else:  # low
        return LeadAction.WITHHOLD_CLARIFY_HUMAN

def gate_wingman(confidence: str) -> WingmanAction:
    """
    Wingman gate:
      high   → execute immediately, report back to lead
      medium → execute with assumption, flag in status report
      low    → withhold, send clarification request to Lead
    """
    if confidence == "high":
        return WingmanAction.EXECUTE
    elif confidence == "medium":
        return WingmanAction.EXECUTE_WITH_WARNING
    else:  # low
        return WingmanAction.CLARIFY_LEAD
EOF
```

### 8.4 Write ollama_client.py

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/common/ollama_client.py
"""
Thin wrapper around Ollama REST API.
Handles timeout, retry, and JSON extraction.
"""
import requests
import json
import time
import logging

logger = logging.getLogger(__name__)


class OllamaClient:
    def __init__(self, host: str = "localhost", port: int = 11434,
                 model: str = "qwen2.5-coder:3b",
                 num_ctx: int = 2048, max_retries: int = 3,
                 timeout: float = 15.0):
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
            "model": self.model,
            "prompt": prompt,
            "system": system,
            "stream": False,
            "format": "json",
            "options": {
                "num_ctx": self.num_ctx,
                "temperature": 0,
                "top_p": 1.0,
                "repeat_penalty": 1.0,
            }
        }

        for attempt in range(self.max_retries):
            t_start = time.perf_counter()
            try:
                response = requests.post(
                    self.url, json=payload, timeout=self.timeout)
                latency = time.perf_counter() - t_start

                if response.status_code == 200:
                    raw = response.json().get("response", "")
                    # Ollama with format=json should return valid JSON
                    # but sometimes wraps it — try to extract
                    raw = raw.strip()
                    if raw.startswith("{"):
                        return raw, latency
                    # Try to find JSON object in response
                    start = raw.find("{")
                    end = raw.rfind("}") + 1
                    if start >= 0 and end > start:
                        return raw[start:end], latency
                    logger.warning(f"No JSON found in response: {raw[:100]}")
                else:
                    logger.warning(f"Ollama returned {response.status_code}")

            except requests.Timeout:
                logger.warning(f"Ollama timeout (attempt {attempt+1}/{self.max_retries})")
            except Exception as e:
                logger.warning(f"Ollama error: {e} (attempt {attempt+1}/{self.max_retries})")

            if attempt < self.max_retries - 1:
                time.sleep(0.5)

        return None, 0.0
EOF
```

### 8.5 Build and Test Common Package

```bash
# [PC-1]
cd ~/major_ws
colcon build --packages-select major_project
source install/setup.bash
```

```bash
# [PC-1] Run unit tests on schemas
python3 - << 'EOF'
import sys
sys.path.insert(0, '/root/major_ws/src/major_project')
# Adjust path if your username differs:
sys.path.insert(0, '/home/' + __import__('os').getenv('USER') + '/major_ws/src/major_project')

from major_project.common.schemas import (
    FlightIntent, WingmanOrder, StatusReport,
    SituationalAwareness, DronePosition, LeadOutput, WingmanOutput,
    parse_lead_output, parse_wingman_output
)
from major_project.common.normaliser import normalise_parsed, normalise_action
from major_project.common.confidence_gate import gate_lead, gate_wingman, LeadAction, WingmanAction
import json

print("Testing FlightIntent...")
fi = FlightIntent(action="takeoff", altitude=5.0, confidence="high")
assert fi.action == "takeoff"
assert fi.altitude == 5.0
print("  FlightIntent OK")

print("Testing normaliser...")
assert normalise_action("take off") == "takeoff"
assert normalise_action("launch") == "takeoff"
assert normalise_action("scan") == "search"
assert normalise_action("HOVER") == "hover"
print("  Normaliser OK")

print("Testing confidence gate...")
assert gate_lead("high") == LeadAction.EXECUTE
assert gate_lead("low") == LeadAction.WITHHOLD_CLARIFY_HUMAN
assert gate_wingman("low") == WingmanAction.CLARIFY_LEAD
print("  Confidence gate OK")

print("Testing WingmanOrder...")
import uuid
order = WingmanOrder(
    order_id=str(uuid.uuid4()),
    mission_context="Cover the south sector",
    intent=FlightIntent(action="search", confidence="high"),
    priority="routine",
    confidence="high"
)
print("  WingmanOrder:", order.to_prompt_block()[:60])
print("  WingmanOrder OK")

print("Testing SituationalAwareness...")
sa = SituationalAwareness(
    drone_id="LEAD",
    position=DronePosition(x=0, y=0, z=-50, heading=90, speed=2.0),
    battery_pct=87.0,
    flight_mode="OFFBOARD",
    camera_summary="Forward clear. No obstacles."
)
print("  SA block:", sa.to_prompt_block())
print("  SituationalAwareness OK")

print("\nAll tests passed!")
EOF
```

**Expected:** All tests passed!

---

## PART 9: GCS Nodes

### 9.1 STT Node (Faster-Whisper)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/stt_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
import queue
import threading
from faster_whisper import WhisperModel


class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.pub = self.create_publisher(String, '/voice_commands', 10)

        # Load Whisper model
        self.get_logger().info("Loading Faster-Whisper tiny.en (int8)...")
        self.model = WhisperModel("tiny.en", device="cpu",
                                   compute_type="int8")
        self.get_logger().info("STT model loaded.")

        self.audio_queue = queue.Queue()
        self.sample_rate = 16000
        self.chunk_duration = 0.5   # seconds per chunk
        self.chunk_samples = int(self.sample_rate * self.chunk_duration)
        self.buffer = []
        self.silence_threshold = 0.01
        self.min_speech_duration = 1.0   # seconds
        self.silence_after_speech = 1.5  # seconds of silence to end utterance

        self.is_speaking = False
        self.silence_chunks = 0
        self.silence_chunks_needed = int(
            self.silence_after_speech / self.chunk_duration)

        # Start audio capture thread
        self.capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        self.get_logger().info("Listening for voice commands... (speak clearly)")

    def _audio_callback(self, indata, frames, time_info, status):
        self.audio_queue.put(indata.copy())

    def _capture_loop(self):
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32',
            blocksize=self.chunk_samples,
            callback=self._audio_callback
        ):
            while rclpy.ok():
                try:
                    chunk = self.audio_queue.get(timeout=1.0)
                    rms = np.sqrt(np.mean(chunk**2))

                    if rms > self.silence_threshold:
                        self.is_speaking = True
                        self.silence_chunks = 0
                        self.buffer.append(chunk)
                    elif self.is_speaking:
                        self.buffer.append(chunk)
                        self.silence_chunks += 1
                        if self.silence_chunks >= self.silence_chunks_needed:
                            # Utterance complete — transcribe
                            audio = np.concatenate(self.buffer, axis=0).flatten()
                            duration = len(audio) / self.sample_rate
                            if duration >= self.min_speech_duration:
                                self._transcribe(audio)
                            self.buffer = []
                            self.is_speaking = False
                            self.silence_chunks = 0
                except queue.Empty:
                    pass

    def _transcribe(self, audio: np.ndarray):
        try:
            segments, info = self.model.transcribe(
                audio, beam_size=5, language="en",
                condition_on_previous_text=False)
            text = " ".join(seg.text.strip() for seg in segments).strip()
            if text and len(text) > 2:
                self.get_logger().info(f"Transcribed: '{text}'")
                msg = String()
                msg.data = text
                self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 9.2 Clarification Speaker Node

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/clarification_speaker_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os


class ClarificationSpeakerNode(Node):
    def __init__(self):
        super().__init__('clarification_speaker_node')
        self.sub = self.create_subscription(
            String, '/clarification_request', self.on_clarification, 10)
        self.use_tts = os.path.exists('/usr/bin/espeak-ng')
        self.get_logger().info("Clarification Speaker ready.")

    def on_clarification(self, msg: String):
        question = msg.data
        # Print clearly to terminal
        print("\n" + "="*60)
        print("  CLARIFICATION NEEDED FROM GROUND COMMANDER:")
        print(f"  {question}")
        print("="*60 + "\n")
        # Optionally speak it
        if self.use_tts:
            try:
                subprocess.Popen(
                    ['espeak-ng', '-s', '150', question],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ClarificationSpeakerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 9.3 Mission Monitor Node

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/mission_monitor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.panel import Panel
from rich.layout import Layout
from rich.text import Text
import threading
import time
import json


class MissionMonitorNode(Node):
    def __init__(self):
        super().__init__('mission_monitor_node')

        self.drone0_situation = "No data"
        self.drone1_situation = "No data"
        self.last_command = "—"
        self.lead_status = "—"
        self.wingman_status = "—"
        self.mission_log = []
        self.lock = threading.Lock()

        self.sub_d0 = self.create_subscription(
            String, '/drone_0/situation', self.on_drone0, 10)
        self.sub_d1 = self.create_subscription(
            String, '/drone_1/situation', self.on_drone1, 10)
        self.sub_cmd = self.create_subscription(
            String, '/voice_commands', self.on_command, 10)
        self.sub_mission = self.create_subscription(
            String, '/mission_status', self.on_mission_status, 10)

        # Start display thread
        self.display_thread = threading.Thread(
            target=self._display_loop, daemon=True)
        self.display_thread.start()

    def on_drone0(self, msg):
        with self.lock:
            self.drone0_situation = msg.data

    def on_drone1(self, msg):
        with self.lock:
            self.drone1_situation = msg.data

    def on_command(self, msg):
        with self.lock:
            self.last_command = msg.data
            self.mission_log.append(f"[CMD] {msg.data}")
            self.mission_log = self.mission_log[-10:]

    def on_mission_status(self, msg):
        with self.lock:
            try:
                data = json.loads(msg.data)
                self.lead_status = data.get('lead', '—')
                self.wingman_status = data.get('wingman', '—')
            except Exception:
                self.lead_status = msg.data

    def _display_loop(self):
        console = Console()
        with Live(console=console, refresh_per_second=2) as live:
            while rclpy.ok():
                with self.lock:
                    d0 = self.drone0_situation
                    d1 = self.drone1_situation
                    cmd = self.last_command
                    lead_s = self.lead_status
                    wingman_s = self.wingman_status
                    log = list(self.mission_log)

                table = Table(show_header=True, header_style="bold cyan",
                              title="MISSION CONTROL STATION",
                              title_style="bold white on blue")
                table.add_column("Field", style="bold", width=20)
                table.add_column("DRONE-0 (LEAD)", width=35)
                table.add_column("DRONE-1 (WINGMAN)", width=35)
                table.add_row("Situation", d0, d1)
                table.add_row("SLM Status", lead_s, wingman_s)
                table.add_row("Last Command", cmd, "—")

                log_text = "\n".join(log[-5:]) if log else "No commands yet"
                panel = Panel(table, subtitle=f"Log: {log_text[:100]}")
                live.update(panel)
                time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 9.4 Emergency Stop Node

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/emergency_stop_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import threading
import sys


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.pub_stop = self.create_publisher(Bool, '/emergency_stop', 10)
        # Also listen for voice emergency
        self.sub_cmd = self.create_subscription(
            String, '/voice_commands', self.on_voice, 10)

        print("\n[EMERGENCY STOP NODE]")
        print("  Type 'STOP' and press Enter to emergency-land ALL drones")
        print("  OR say 'emergency land' / 'abort all' via voice")
        print()

        self.keyboard_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()

    def _keyboard_loop(self):
        while True:
            try:
                line = input()
                if line.strip().upper() in ('STOP', 'ABORT', 'EMERGENCY', 'E'):
                    self._trigger_stop("keyboard")
            except EOFError:
                break

    def on_voice(self, msg: String):
        text = msg.data.lower()
        triggers = ['emergency', 'abort all', 'emergency land',
                    'stop all', 'all land', 'kill']
        if any(t in text for t in triggers):
            self._trigger_stop(f"voice: '{msg.data}'")

    def _trigger_stop(self, source: str):
        self.get_logger().error(f"EMERGENCY STOP triggered by {source}")
        msg = Bool()
        msg.data = True
        # Publish multiple times to ensure delivery
        for _ in range(5):
            self.pub_stop.publish(msg)
        print(f"\n!!! EMERGENCY STOP SENT (source: {source}) !!!\n")


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 9.5 Camera Detection Node (YOLOv8-nano)

This node subscribes to a camera feed, runs YOLOv8-nano detection, and publishes a text summary to `/camera_0/detections`. It falls back gracefully if no camera or model is available — the sensor aggregators still work; they just report "Camera not available."

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/gcs/camera_detection_node.py
"""
Camera detection node for Drone-0 (physical camera on PC-1).
Publishes YOLOv8-nano detection summaries to /camera_0/detections.
Falls back gracefully if camera hardware or ultralytics is unavailable.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

try:
    import cv2
    import numpy as np
    from ultralytics import YOLO
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False


class CameraDetectionNode(Node):
    def __init__(self):
        super().__init__('camera_detection_node')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.pub = self.create_publisher(String, '/camera_0/detections', 10)
        self._running = True

        if not VISION_AVAILABLE:
            self.get_logger().warning(
                "OpenCV or ultralytics not installed — camera in fallback mode. "
                "Sensor aggregator will still work; camera field shows static message.")
            self.create_timer(2.0, self._publish_fallback)
            return

        cam_idx = self.get_parameter('camera_index').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.get_logger().warning(
                f"Camera index {cam_idx} not available — running in fallback mode.")
            self.cap = None
            self.create_timer(2.0, self._publish_fallback)
            return

        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLOv8 loaded from {model_path}")
        except Exception as e:
            self.get_logger().warning(f"YOLO load failed ({e}) — fallback mode.")
            self.model = None
            self.cap.release()
            self.create_timer(2.0, self._publish_fallback)
            return

        self._detect_thread = threading.Thread(
            target=self._detect_loop, args=(1.0 / rate_hz,), daemon=True)
        self._detect_thread.start()
        self.get_logger().info(
            f"Camera detection running at {rate_hz}Hz (index={cam_idx})")

    def _publish_fallback(self):
        msg = String()
        msg.data = "Camera not available. Sensor data only."
        self.pub.publish(msg)

    def _detect_loop(self, period: float):
        while rclpy.ok() and self._running:
            ret, frame = self.cap.read()
            if not ret:
                msg = String()
                msg.data = "Camera read error. Sensor data only."
                self.pub.publish(msg)
                time.sleep(period)
                continue

            results = self.model(frame, conf=self.conf_threshold, verbose=False)
            detections = []
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names.get(cls_id, str(cls_id))
                    conf = float(box.conf[0])
                    detections.append(f"{cls_name}({conf:.0%})")

            summary = ", ".join(detections) if detections else "Clear — no obstacles"
            msg = String()
            msg.data = summary
            self.pub.publish(msg)
            time.sleep(period)

    def destroy_node(self):
        self._running = False
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

> **If you have no physical camera:** The node automatically detects this and publishes `"Camera not available. Sensor data only."` — this is fine. The sensor aggregator and SLM prompts still work; the camera field in the situational awareness block just shows that static message.

---

### 9.6 Build and Verify GCS Nodes

```bash
# [PC-1]
cd ~/major_ws
# --symlink-install: instead of copying Python source into install/, creates symlinks.
# This means edits to .py files take effect immediately without rebuilding.
# Use it during development. For final deployment, drop the flag.
colcon build --packages-select major_project --symlink-install
source install/setup.bash
```

```bash
# [PC-1] Verify entry points are registered
ros2 pkg executables major_project
# Should list all registered nodes, including camera_detection
```

---

### 9.7 Create requirements.txt

Pin all Python dependencies so the project is reproducible:

```bash
cat << 'EOF' > ~/major_ws/src/major_project/requirements.txt
# Python dependencies — install with: pip3 install -r requirements.txt
pydantic>=2.0,<3.0
faster-whisper>=1.0.0
requests>=2.28.0
numpy>=1.24.0
ultralytics>=8.0.0
rich>=13.0.0
pyaudio>=0.2.13
sounddevice>=0.4.6
scipy>=1.10.0
empy==3.3.4
EOF
```

```bash
# Install from requirements.txt:
pip3 install -r ~/major_ws/src/major_project/requirements.txt
```

### 9.8 Create .gitignore

```bash
cat << 'EOF' > ~/major_ws/src/major_project/.gitignore
# Python
__pycache__/
*.py[cod]
*.egg-info/
*.egg

# Build artifacts
build/
install/
log/

# Benchmark results (large CSV files)
benchmark/evaluation_results.csv
benchmark/results_*.txt

# IDE
.vscode/
.idea/

# Ollama model files
*.gguf
*.bin

# OS
.DS_Store
Thumbs.db
EOF
```

---

*End of Part 2. Continue in tutorial_part3_lead.md*
