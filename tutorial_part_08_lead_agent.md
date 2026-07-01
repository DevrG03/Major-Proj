# Part 8: Lead Pilot Agent

> **Target:** PC-1
> **Prerequisites:** Parts 1–7 complete and verified.

The Lead Agent is the autonomous brain for Drone-0. It runs a continuous think-act-observe loop.

**All 5 critical loophole fixes are applied:**
- **Fix #1 (passive agent):** Self-starts with STANDBY goal on node boot
- **Fix #2 (sleep blocks):** `_wait()` / `_search()` are interruptible; only 200ms pause for instant tools
- **Fix #3 (deadlock):** `ask_human` is non-blocking; agent monitors while waiting
- **Fix #4 (race condition):** `_abort_event` cleanly terminates the old loop before starting new
- **Fix #5 (silent stall):** SLM health monitor triggers RTL after 5 consecutive failures

---

## 8.1 Create Prompts Directory

```bash
mkdir -p ~/major_ws/src/major_project/major_project/lead_pilot/prompts
```

---

## 8.2 System Prompt (`lead_pilot/prompts/lead_agent_system.txt`)

```bash
cat << 'PROMPT_EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/prompts/lead_agent_system.txt
You are LEAD PILOT — the autonomous brain controlling Drone-0 (Lead).
Your Wingman controls Drone-1. Human Ground Commander gives you mission goals via voice.

You run in a think-act-observe loop. At each step output EXACTLY ONE tool call:
{"tool": "<name>", "params": {"key": value}}
No params? Use: {"tool": "<name>", "params": {}}
Always start your response with { and end with }. No prose, no markdown, no explanation.

══════════════════════════════════════════
AVAILABLE TOOLS:
══════════════════════════════════════════

FLIGHT:
  takeoff(altitude:float)             Arm and ascend to altitude metres (1–30). Returns ETA.
  move(direction:str, distance:float) Fly N/S/E/W/NE/NW/SE/SW/forward/backward/left/right. Returns ETA.
  move(direction, distance, altitude) Fly and change altitude simultaneously.
  hover()                             Hold current position.
  search(duration_sec:int)            Hover and scan camera for 5–60 seconds. Returns detections.
  land()                              Land at current position.
  rtl()                               Return to launch and land.

SENSING:
  get_situation()                     Full sensor readout: pos/alt/bat/GPS/mode/camera/wingman_pos.
  scan_camera()                       Camera detections with direction and distance.
  get_battery()                       Own battery percentage.
  get_wingman_situation()             Wingman's last known position from situation block.

MEMORY:
  remember(fact:str)                  Store a fact permanently.
  recall(query:str)                   Retrieve stored facts by keyword.

TIMING:
  wait(seconds:int)                   Pause 1–30s then continue. Abortable.

COMMUNICATION:
  message_wingman(message:str, msg_type:str)  Send to Wingman.
                  msg_type = task (new mission) | status (info only) | reply | abort
  ask_human(question:str)             Ask Ground Commander. NON-BLOCKING.
                  Agent keeps monitoring until [HUMAN ANSWERED] appears in context.
  notify_human(message:str)           Status to GCS. No wait.

COMPLETION:
  mission_complete(report:str)        End mission with full summary report.

══════════════════════════════════════════
AGENT RULES:
══════════════════════════════════════════
1. Start every mission with get_situation() to read current state.
2. After takeoff: call wait(N) then get_situation() to confirm altitude reached.
3. After move: call wait(ETA) then get_situation() to confirm arrival.
4. Battery ≤ 20%: call notify_human immediately. Plan RTL soon.
5. Battery ≤ 15%: call rtl() immediately. Safety monitor also does this independently.
6. Wingman NEVER contacts human — you are the sole human-drone interface.
7. Use message_wingman(msg_type='task') to assign missions to Wingman.
8. Use message_wingman(msg_type='status') for informational updates to Wingman.
9. Use message_wingman(msg_type='reply') to answer Wingman queries.
10. Use ask_human ONLY for: safety decisions, scope changes, genuine uncertainty.
11. Use notify_human for all routine status (no human reply needed).
12. Use remember() for: object positions, mission decisions, notable observations.
13. STANDBY goal: call get_situation(), then wait(30), repeat. Do NOT call mission_complete.
14. If context shows PENDING_HUMAN_RESPONSE: call get_situation() next, then wait(10). Repeat until [HUMAN ANSWERED] appears.

══════════════════════════════════════════
DIRECTION SHORTCUTS:
══════════════════════════════════════════
N=north S=south E=east W=west NE=northeast NW=northwest SE=southeast SW=southwest

══════════════════════════════════════════
EXAMPLES:
══════════════════════════════════════════
Mission start → {"tool":"get_situation","params":{}}
Take off →      {"tool":"takeoff","params":{"altitude":10}}
Wait for ETA →  {"tool":"wait","params":{"seconds":25}}
Confirm →       {"tool":"get_situation","params":{}}
Move north →    {"tool":"move","params":{"direction":"N","distance":50}}
Wait ETA →      {"tool":"wait","params":{"seconds":28}}
Scan area →     {"tool":"search","params":{"duration_sec":20}}
Found object →  {"tool":"remember","params":{"fact":"football at pos(50,0) N sector at 10m alt"}}
Tell wingman →  {"tool":"message_wingman","params":{"message":"Football found N50m. Cover E sector.","msg_type":"task"}}
Status GCS →    {"tool":"notify_human","params":{"message":"North sector surveyed. Found football at 50m N."}}
Done →          {"tool":"mission_complete","params":{"report":"Football found 50m north. Wingman covered east. Both RTL."}}
PROMPT_EOF
```

---

## 8.3 Lead Agent Node (`lead_pilot/lead_agent_node.py`)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_agent_node.py
"""
Lead Agent Node — the always-active autonomous brain for Drone-0.

All 5 critical loophole fixes from architectural audit:

Fix #1 — Self-Start (Loophole #1):
  Agent starts with STANDBY goal on node boot. No voice command needed.

Fix #2 — Non-Blocking Tools (Loophole #2):
  _wait() and _search() check _abort_event every 0.5s/2s.
  Only 200ms pause for instantaneous tools (not after every tool).

Fix #3 — Non-Blocking ask_human (Loophole #3):
  ask_human() returns PENDING sentinel immediately.
  Agent loop does passive get_situation() monitoring every 3s while waiting.
  Answer injected as [HUMAN ANSWERED] memory note when voice arrives.
  Timeout after 120s if no response.

Fix #4 — Safe Goal Replacement (Loophole #4):
  _abort_event is set before clearing context. Old loop detects abort
  at top of each iteration and exits cleanly. New loop starts after brief wait.

Fix #5 — SLM Health Monitor (Loophole #5):
  consecutive_failures tracked. After MAX_CONSECUTIVE_FAILURES:
    - RTL intent published directly
    - Human notified via /clarification_request
    - /agent/health reflects failure state
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import json
import os
import re
import threading
import time

# QoS matching the LeadIntentBridgeNode and GCS nodes
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from major_project.common.ollama_client import OllamaClient
from major_project.common.tool_registry import LeadToolRegistry
from major_project.common.context_manager import ContextManager
from major_project.common.agent_memory import AgentMemory


def _load_prompt(filename: str) -> str:
    """Load a system prompt from the prompts/ subdirectory."""
    path = os.path.join(os.path.dirname(__file__), 'prompts', filename)
    with open(path) as f:
        return f.read()


class LeadAgentNode(Node):

    # Tools that return in <50ms — apply brief pause to avoid SLM hammering
    FAST_TOOLS = frozenset({
        'notify_human', 'message_wingman', 'hover',
        'remember', 'get_battery',
    })

    # After this many consecutive SLM failures → trigger RTL fallback
    MAX_CONSECUTIVE_FAILURES = 5

    # After this many 3s monitoring cycles without human response → timeout
    HUMAN_WAIT_TIMEOUT_CYCLES = 40   # 40 × 3s = 120s

    def __init__(self):
        super().__init__('lead_agent_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('ollama_host', 'localhost')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('model', 'qwen2.5-coder:3b')
        self.declare_parameter('num_ctx', 3072)  # raised: 2048 too small for STANDBY loop

        host    = self.get_parameter('ollama_host').value
        port    = self.get_parameter('ollama_port').value
        model   = self.get_parameter('model').value
        self._num_ctx = self.get_parameter('num_ctx').value

        # ── SLM client + system prompt ─────────────────────────────
        self.ollama = OllamaClient(
            host=host, port=port, model=model, num_ctx=self._num_ctx)
        self.system_prompt = _load_prompt('lead_agent_system.txt')

        # ── Shared sensor state (protected by self.lock) ──────────
        self.lock            = threading.Lock()
        self.own_situation   = ""
        self.camera_summary  = ""
        self.obstacle_vector = ""
        self.battery_pct     = 100.0

        # ── Human interaction state (non-blocking) ────────────────
        self._waiting_for_human  = False
        self._human_response: str | None = None
        self._human_wait_cycles  = 0

        # ── Wingman query tracking ────────────────────────────────
        self._wingman_query_pending = False

        # ── Agent loop state ──────────────────────────────────────
        self._agent_running  = False
        self._mission_done   = False
        self._mission_report = ""
        self._abort_event    = threading.Event()

        # ── SLM health tracking ───────────────────────────────────
        self._consecutive_failures = 0
        self._slm_healthy          = True

        # ── Core components ───────────────────────────────────────
        self.ctx          = ContextManager()
        self.agent_memory = AgentMemory(db_name="lead_agent_memory.db")
        self.tools        = LeadToolRegistry(self)

        # ── Publishers ────────────────────────────────────────────
        # RELIABLE_QOS (TRANSIENT_LOCAL) must match intent bridge + GCS subscribers
        self.pub_intent         = self.create_publisher(
            String, '/lead/approved_intent', RELIABLE_QOS)
        self.pub_clarification  = self.create_publisher(
            String, '/clarification_request', RELIABLE_QOS)
        self.pub_mission_status = self.create_publisher(
            String, '/mission_status', RELIABLE_QOS)
        # Volatile OK — agent/health and wingman comms use depth=10 on both sides
        self.pub_wingman_msg    = self.create_publisher(
            String, '/agent/lead_to_wingman', 10)
        self.pub_health         = self.create_publisher(
            String, '/agent/health', 10)

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(
            String, '/drone_0/situation',   self._on_situation, 10)
        self.create_subscription(
            String, '/camera_0/detections', self._on_camera, 10)
        self.create_subscription(
            String, '/camera_0/obstacle_vector', self._on_obstacle, 10)
        self.create_subscription(
            String, '/voice_commands',      self._on_voice, 10)
        self.create_subscription(
            String, '/agent/wingman_to_lead', self._on_wingman_message, 10)
        self.create_subscription(
            String, '/safety/event',        self._on_safety_event, 10)

        # ── Periodic health publisher ──────────────────────────────
        self.create_timer(10.0, self._publish_health)

        self.get_logger().info(
            f"Lead Agent ready — Ollama {host}:{port} model:{model} ctx:{self._num_ctx}")

        # ── Fix #1: Self-start with STANDBY goal ──────────────────
        self._assign_goal(
            "STANDBY: Monitor drone situation and await mission goal from "
            "Ground Commander. Call get_situation() to check state, "
            "then wait(30) and repeat indefinitely.")

    # ════════════════════════════════════════════════════════════════
    # ROS2 Callbacks
    # ════════════════════════════════════════════════════════════════

    def _on_situation(self, msg: String):
        with self.lock:
            self.own_situation = msg.data
            # Extract battery % from situation string for quick access
            m = re.search(r'bat:(\d+(?:\.\d+)?)', msg.data)
            if m:
                self.battery_pct = float(m.group(1))
        # Update context with fresh situation (called from ROS spin thread)
        self.ctx.update_situation(msg.data)

    def _on_camera(self, msg: String):
        with self.lock:
            self.camera_summary = msg.data

    def _on_obstacle(self, msg: String):
        with self.lock:
            self.obstacle_vector = msg.data

    def _on_voice(self, msg: String):
        """Handle voice input: either a human answer or a new mission goal."""
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"Voice: '{text}'")

        # Fix #3: If waiting for human reply, this IS the answer
        if self._waiting_for_human:
            self._human_response = text
            return   # agent loop will detect and inject [HUMAN ANSWERED]

        # Otherwise: new mission goal — fix #4 handles safe replacement
        self._assign_goal(text)

    def _on_wingman_message(self, msg: String):
        """Parse typed AgentMessage envelope from Wingman."""
        try:
            data     = json.loads(msg.data)
            msg_type = data.get('type', 'status')
            content  = data.get('content', msg.data)
            sender   = data.get('sender', 'WINGMAN')
        except Exception:
            msg_type = 'status'
            content  = msg.data

        self.get_logger().info(f"Wingman [{msg_type}]: {content[:80]}")
        self.ctx.add_inter_agent_message("WINGMAN", f"[{msg_type.upper()}] {content}")

        if msg_type == 'query':
            self._wingman_query_pending = True

    def _on_safety_event(self, msg: String):
        """Inject safety events into agent context memory."""
        try:
            data = json.loads(msg.data)
            note = data.get('message', msg.data)
        except Exception:
            note = msg.data
        self.ctx.add_memory_note(f"[SAFETY] {note}")
        self.get_logger().warning(f"Safety event: {note[:100]}")

    # ════════════════════════════════════════════════════════════════
    # Goal Assignment (Fix #4 — safe replacement with abort event)
    # ════════════════════════════════════════════════════════════════

    def _assign_goal(self, goal: str):
        self.get_logger().info(f"New goal: '{goal[:100]}'")

        # Signal any running loop to exit cleanly
        if self._agent_running:
            self._abort_event.set()
            # Wait for the previous thread to actually finish executing and exit
            self.get_logger().info("Waiting for old agent thread to exit...")
            while self._agent_running:
                time.sleep(0.05)

        # Reset state for new goal
        self._abort_event.clear()
        self.ctx.clear_history()
        self.ctx.set_goal(goal)
        self._mission_done       = False
        self._mission_report     = ""
        self._waiting_for_human  = False
        self._human_response     = None
        self._human_wait_cycles  = 0
        self._consecutive_failures = 0
        self._slm_healthy        = True

        self._agent_running = True
        threading.Thread(target=self._agent_loop, daemon=True).start()

    # ════════════════════════════════════════════════════════════════
    # Agent Loop (the core think-act-observe cycle)
    # ════════════════════════════════════════════════════════════════

    def _agent_loop(self):
        self.get_logger().info("Lead agent loop started.")
        self._publish_status("Agent loop active.")

        while rclpy.ok() and not self._mission_done and not self._abort_event.is_set():

            # ── Step 1: Handle pending human response (Fix #3) ──────
            if self._waiting_for_human:
                if self._human_response is not None:
                    # Got the answer — inject into context
                    answer = self._human_response
                    self._human_response    = None
                    self._waiting_for_human = False
                    self._human_wait_cycles = 0
                    self.ctx.add_memory_note(f"[HUMAN ANSWERED] {answer}")
                    self.get_logger().info(f"Human answered: '{answer[:100]}'")
                    # Fall through to normal SLM inference with answer in context

                else:
                    # Still waiting: passive monitoring
                    self._human_wait_cycles += 1

                    if self._human_wait_cycles >= self.HUMAN_WAIT_TIMEOUT_CYCLES:
                        # Timeout — proceed autonomously
                        self._waiting_for_human = False
                        self._human_wait_cycles = 0
                        self.ctx.add_memory_note(
                            "[HUMAN TIMEOUT] No response after 120s. "
                            "Proceeding with best judgment.")
                        self.get_logger().warning(
                            "Human response timeout — continuing autonomously")
                    else:
                        # Get situation, then wait 3s before checking again
                        sit = self.tools.execute('get_situation', {})
                        self.ctx.add_tool_result('get_situation', {}, sit)
                        self._publish_status(
                            f"Waiting for human ({self._human_wait_cycles * 3}s elapsed)…")
                        time.sleep(3.0)
                        continue   # do not call SLM while waiting

            # ── Step 2: Handle Wingman query ─────────────────────────
            if self._wingman_query_pending:
                self._wingman_query_pending = False
                self.ctx.add_memory_note(
                    "[NOTE] Wingman has a pending query. "
                    "Use message_wingman(msg_type='reply') to answer it.")

            # ── Step 3: Abort check ──────────────────────────────────
            if self._abort_event.is_set():
                break

            # ── Step 4: Guard context size, then run SLM inference ────────
            # If prompt > 75% of num_ctx (chars estimated at ~3.5 chars/token),
            # auto-compress history to prevent context overflow that causes
            # the model to output garbage JSON.
            prompt    = self.ctx.build_prompt()
            ctx_chars = int(self._num_ctx * 3.5 * 0.75)
            if len(prompt) > ctx_chars:
                self.ctx.compress_history()
                prompt = self.ctx.build_prompt()
                self.get_logger().info(
                    f"Context compressed ({len(prompt)} chars > {ctx_chars} budget).")

            tool_name, params = self._infer_tool_call(prompt)

            # ── Step 5: Handle SLM failure ───────────────────────────
            if tool_name is None:
                self._consecutive_failures += 1
                self.get_logger().warning(
                    f"SLM parse failure "
                    f"#{self._consecutive_failures}/{self.MAX_CONSECUTIVE_FAILURES}")

                if self._consecutive_failures >= self.MAX_CONSECUTIVE_FAILURES:
                    self._trigger_slm_health_fallback()
                    break   # exit loop after fallback

                time.sleep(1.0)
                continue

            # ── Step 6: Successful inference — reset health counter ───
            self._consecutive_failures = 0
            if not self._slm_healthy:
                self._slm_healthy = True
                self.get_logger().info("SLM recovered — health restored.")

            # ── Step 7: Execute tool ─────────────────────────────────
            self.get_logger().info(
                f"Lead → {tool_name}({json.dumps(params)[:100]})")
            result = self.tools.execute(tool_name, params)
            self.get_logger().info(f"← {result[:120]}")

            # ── Step 8: Update context ───────────────────────────────
            self.ctx.add_tool_result(tool_name, params, result)
            self._publish_status(f"{tool_name}: {result[:80]}")

            # ── Step 9: Brief pause for instant-return tools only ─────
            # (Fix #2: removed unconditional 0.5s pause)
            if tool_name in self.FAST_TOOLS and not self._mission_done:
                time.sleep(0.2)

        # ── Loop exit ────────────────────────────────────────────────
        if self._mission_done:
            self.get_logger().info(
                f"Mission complete: {self._mission_report[:120]}")
            self._publish_status(f"MISSION COMPLETE: {self._mission_report}")
            # Announce to GCS speaker
            done_msg = String()
            done_msg.data = f"[MISSION COMPLETE] {self._mission_report}"
            self.pub_clarification.publish(done_msg)

        elif self._abort_event.is_set():
            self.get_logger().info("Agent loop aborted — new goal incoming.")

        else:
            self.get_logger().info("Agent loop ended (shutdown).")

        self._agent_running = False

    # ════════════════════════════════════════════════════════════════
    # SLM Health Fallback (Fix #5)
    # ════════════════════════════════════════════════════════════════

    def _trigger_slm_health_fallback(self):
        self._slm_healthy = False
        self.get_logger().error(
            f"SLM health failure ({self.MAX_CONSECUTIVE_FAILURES} consecutive failures) "
            "— initiating RTL fallback")

        # Notify human via GCS speaker
        alert = String()
        alert.data = (
            f"CRITICAL ALERT: Lead SLM inference failed "
            f"{self.MAX_CONSECUTIVE_FAILURES} consecutive times. "
            "Autonomous control suspended. RTL initiated for safety.")
        self.pub_clarification.publish(alert)

        # Issue RTL directly to commander (bypasses SLM)
        rtl_msg = String()
        rtl_msg.data = json.dumps({'action': 'rtl', 'confidence': 'high'})
        self.pub_intent.publish(rtl_msg)

        self._publish_status("SLM_HEALTH_FAILURE: RTL initiated")
        self._publish_health()   # immediately update health topic

    # ════════════════════════════════════════════════════════════════
    # SLM Inference with 3-attempt retry
    # ════════════════════════════════════════════════════════════════

    def _infer_tool_call(self, prompt: str) -> tuple[str | None, dict]:
        error_ctx = ""
        for attempt in range(3):
            # Build prompt with correction context on retry
            full_prompt = prompt
            if error_ctx:
                full_prompt += (
                    f"\n\n[CORRECTION NEEDED] {error_ctx}"
                    f"\nOutput a valid JSON tool call only. Begin {{ end }}")

            raw, latency = self.ollama.infer(full_prompt, self.system_prompt)
            self.get_logger().debug(f"Inference {latency * 1000:.0f}ms")

            # Abort check between retry attempts
            if self._abort_event.is_set():
                return None, {}

            if raw is None:
                error_ctx = "SLM returned no output."
                continue

            try:
                raw_clean = raw.strip()
                # Extract JSON object even if wrapped in prose
                start = raw_clean.find('{')
                end   = raw_clean.rfind('}') + 1
                if start >= 0 and end > start:
                    raw_clean = raw_clean[start:end]

                data      = json.loads(raw_clean)
                tool_name = data.get('tool', '')
                params    = data.get('params', {})

                if not isinstance(params, dict):
                    params = {}

                if self.tools.is_valid(tool_name):
                    return tool_name, params

                error_ctx = (
                    f"Unknown tool '{tool_name}'. "
                    f"Valid tools: {sorted(self.tools.tools.keys())}")

            except json.JSONDecodeError as exc:
                error_ctx = (
                    f"JSON parse error: {str(exc)[:80]}. "
                    "Output ONLY {{\"tool\":\"...\",\"params\":{{...}}}}")
            except Exception as exc:
                error_ctx = f"Parse error: {str(exc)[:80]}"

        return None, {}

    # ════════════════════════════════════════════════════════════════
    # Helpers
    # ════════════════════════════════════════════════════════════════

    def _publish_status(self, text: str):
        msg = String()
        msg.data = json.dumps({"lead": text, "wingman": "—"})
        self.pub_mission_status.publish(msg)

    def _publish_health(self):
        msg = String()
        msg.data = json.dumps({
            "node":                "lead",
            "slm_ok":              self._slm_healthy,
            "consecutive_failures": self._consecutive_failures,
            "agent_running":       self._agent_running,
            "waiting_for_human":   self._waiting_for_human,
        })
        self.pub_health.publish(msg)


# ════════════════════════════════════════════════════════════════════
# Entry point
# ════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = LeadAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

---

## 8.4 Verify Lead Agent

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install 2>&1 | tail -5
source install/setup.bash

# ── Import test ────────────────────────────────────────────────────
python3 - << 'PYEOF'
import sys, os
sys.path.insert(0, os.path.expanduser('~/major_ws/src/major_project'))

# Verify module imports without ROS2 runtime
import major_project.lead_pilot.lead_agent_node as m
assert hasattr(m, 'LeadAgentNode'), "LeadAgentNode class missing"
assert hasattr(m, 'main'), "main() function missing"

# Verify prompt file exists
prompt_path = os.path.expanduser(
    '~/major_ws/src/major_project/major_project/lead_pilot/prompts/lead_agent_system.txt')
assert os.path.exists(prompt_path), f"System prompt missing: {prompt_path}"
with open(prompt_path) as f:
    content = f.read()
assert 'STANDBY' in content, "STANDBY rule missing from system prompt"
assert 'PENDING_HUMAN_RESPONSE' in content, "Pending response rule missing"
assert 'message_wingman' in content, "message_wingman missing from tools list"
assert 'get_wingman_situation' in content, "get_wingman_situation missing"

print("Lead agent node imports OK")
print("System prompt OK")
print("All checks passed ✅")
PYEOF

# ── Verify entry point registered ─────────────────────────────────
ros2 pkg executables major_project | grep lead_agent
# Must output: major_project lead_agent
```

### Live Test (with Ollama running)

```bash
# Terminal 1: Start PX4 SITL + DDS agent (from Part 1)
# Terminal 2: Start lead sensor aggregator (from Part 6)
ros2 run major_project lead_sensor_aggregator

# Terminal 3: Start lead commander (from Part 7)
ros2 run major_project lead_px4_commander

# Terminal 4: Start lead agent
ros2 run major_project lead_agent

# Terminal 5: Watch agent output (self-starts, no voice needed)
ros2 topic echo /mission_status
# Must show output within 10 seconds of launch (Fix #1 verified)
# Expected: {"lead": "get_situation: bat:100% alt:0m mode:MANUAL ...", "wingman": "—"}
```

### Test Fix #3 (Non-blocking ask_human)

```bash
# ── Step 1: Inject obstacle data continuously (keep running in Terminal A) ──
# BOTH topics needed: detections feeds camera_summary, obstacle_vector feeds obstacle_vector
ros2 topic pub --rate 3 /camera_0/detections std_msgs/msg/String \
  '{"data": "Detected 1 obstacle(s): person ahead very_close"}'

ros2 topic pub --rate 3 /camera_0/obstacle_vector std_msgs/msg/String \
  '{"data": "person:ahead:very_close"}'

# ── Step 2: Give agent an ACTIVE mission (Terminal B) ────────────────────────
# The SLM only calls scan_camera/ask_human during an active mission, NOT in STANDBY.
# A voice command breaks it out of the STANDBY loop into mission execution.
ros2 topic pub --once \
  --qos-durability transient_local \
  --qos-reliability reliable \
  /voice_commands std_msgs/msg/String \
  '{"data": "take off and fly north 50 metres"}'

# ── Step 3: Watch for ask_human output (Terminal C) ─────────────────────────
ros2 topic echo /clarification_request
# Expected sequence (takes ~30-60s for SLM to reach scan_camera):
#   data: "Person detected directly ahead at very close range.
#          Do I stop and hover, or continue the mission?"

# ── Step 4: Verify agent is not frozen (Terminal D) ─────────────────────────
ros2 topic echo /mission_status
# Expected: /mission_status keeps updating every few seconds (Fix #3 verified)

# ── Step 5: Answer the question (Terminal B) ─────────────────────────────────
ros2 topic pub --once \
  --qos-durability transient_local \
  --qos-reliability reliable \
  /voice_commands std_msgs/msg/String \
  '{"data": "hover and observe, do not approach"}'
# Expected: [HUMAN ANSWERED] injected into context, agent continues with hover
```

### Test Fix #4 (Safe Goal Replacement)

```bash
ros2 topic pub --once /voice_commands std_msgs/msg/String \
  '{data: "fly north 100 metres"}'
sleep 3
ros2 topic pub --once /voice_commands std_msgs/msg/String \
  '{data: "abort and return home immediately"}'
ros2 topic echo /lead/approved_intent
# Expected: RTL or hover intent appears within 2 seconds of second command
# No conflicting move + RTL commands simultaneously
```

### Test Fix #5 (SLM Health Fallback)

```bash
# Stop Ollama
sudo systemctl stop ollama
# Wait ~30 seconds for 5 failures
ros2 topic echo /clarification_request
# Expected: "CRITICAL ALERT: Lead SLM inference failed 5 consecutive times..."
ros2 topic echo /lead/approved_intent
# Expected: {"action": "rtl", "confidence": "high"}
ros2 topic echo /agent/health
# Expected: {"node": "lead", "slm_ok": false, "consecutive_failures": 5, ...}

# Restore Ollama
sudo systemctl start ollama
```
