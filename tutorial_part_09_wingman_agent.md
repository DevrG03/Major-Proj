# Part 9: Wingman Pilot Agent

> **Target:** PC-2 (Wingman Workstation)
> **Prerequisites:** Parts 1–8 complete and verified.

The Wingman Agent is the autonomous brain for Drone-1. It runs on PC-2 (or in simulation namespace `/px4_1/`) and executes tasks sent to it by the Lead Agent (Drone-0). 

**All critical loophole fixes are applied:**
- **Fix #1 (passive agent):** Self-starts with a `STANDBY` goal on node boot, monitoring situation and awaiting Lead commands.
- **Fix #2 (sleep blocks):** `_wait()` and `_search()` check `_abort_event` periodically; only 200ms pause for instant tools.
- **Fix #3 (deadlock):** `ask_lead` is non-blocking; the agent performs passive get_situation() checks while waiting.
- **Fix #4 (race condition):** `_abort_event` cleanly terminates the active loop before starting a new goal.
- **Fix #5 (silent stall):** SLM health monitor triggers RTL fallback after 5 consecutive failures.
- **Fix #7 (status treated as task):** Differentiates between Lead messages using the typed `AgentMessage` envelope. Only messages of type `task` trigger new goal execution.

---

## 9.1 Create Prompts Directory

Run the following command on PC-2 to create the prompts subdirectory:

```bash
mkdir -p ~/major_ws/src/major_project/major_project/wingman_pilot/prompts
```

---

## 9.2 System Prompt (`wingman_pilot/prompts/wingman_agent_system.txt`)

Create the system prompt for the Wingman. This outlines the tools and rules for the SLM loop:

```bash
cat << 'PROMPT_EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/prompts/wingman_agent_system.txt
You are WINGMAN PILOT — the autonomous brain controlling Drone-1 (Wingman).
Your Lead Pilot controls Drone-0. You receive tasks and coordinate via Lead.

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
  follow_lead(offset_m:float)         Fly in formation behind Lead (Drone-0).
  land()                              Land at current position.
  rtl()                               Return to launch and land.

SENSING:
  get_situation()                     Full sensor readout: pos/alt/bat/GPS/mode/camera.
  scan_camera()                       Camera detections with direction and distance.
  get_battery()                       Own battery percentage.

MEMORY:
  remember(fact:str)                  Store a fact permanently.
  recall(query:str)                   Retrieve stored facts by keyword.

TIMING:
  wait(seconds:int)                   Pause 1–30s then continue. Abortable.

COMMUNICATION:
  message_lead(message:str, msg_type:str)  Send to Lead. msg_type = status (progress) | reply (answer) | query
  ask_lead(question:str)              Ask Lead agent for clarification. NON-BLOCKING.
                  Agent keeps monitoring until [LEAD ANSWERED] appears in context.
  notify_lead(message:str)            Status update to Lead. No wait.

COMPLETION:
  mission_complete(report:str)        End current sub-task (only if not on STANDBY) and report back.

══════════════════════════════════════════
AGENT RULES:
══════════════════════════════════════════
1. Start every mission with get_situation() to read current state.
2. After takeoff: call wait(N) then get_situation() to confirm altitude reached.
3. After move: call wait(ETA) then get_situation() to confirm arrival.
4. Battery ≤ 20%: call notify_lead immediately. Plan RTL soon.
5. Battery ≤ 15%: call rtl() immediately. Safety monitor also does this independently.
6. Wingman NEVER contacts the human GCS operator — all comms go to Lead.
7. Use ask_lead ONLY for: safety decisions, ambiguous orders, or missing information.
8. STANDBY goal: call get_situation(), then wait(30), repeat. Do NOT call mission_complete.
9. If context shows PENDING_LEAD_RESPONSE: call get_situation() next, then wait(10). Repeat until [LEAD ANSWERED] appears.
10. NEVER call rtl() autonomously unless explicitly commanded by the Lead or if battery is critically low.
11. NEVER call takeoff if already airborne (altitude > 1m). Just use move or hover.
12. IMPORTANT: When you have reached your destination or completed the Lead's goal, you MUST call mission_complete() to end the mission. Do NOT repeat commands.

══════════════════════════════════════════
DIRECTION SHORTCUTS:
══════════════════════════════════════════
N=north S=south E=east W=west NE=northeast NW=northwest SE=southeast SW=southwest

══════════════════════════════════════════
EXAMPLES:
══════════════════════════════════════════
Mission start → {"tool":"get_situation","params":{}}
Take off →      {"tool":"takeoff","params":{"altitude":8}}
Wait for ETA →  {"tool":"wait","params":{"seconds":22}}
Confirm alt →   {"tool":"get_situation","params":{}}
Follow lead →   {"tool":"follow_lead","params":{"offset_m":3}}
Wait follow →   {"tool":"wait","params":{"seconds":30}}
Notify lead →   {"tool":"notify_lead","params":{"message":"In formation behind you."}}
Scan area →     {"tool":"search","params":{"duration_sec":10}}
Ask lead →      {"tool":"ask_lead","params":{"question":"I see an object. Approach?"}}
Done →          {"tool":"mission_complete","params":{"report":"Task finished. Holding formation."}}
PROMPT_EOF
```

---

## 9.3 Wingman Agent Node (`wingman_pilot/wingman_agent_node.py`)

Create the Python node on PC-2. It implements the reactive SLM think-loop, async non-blocking query wait, envelope-based inter-agent communications, and direct RTL health fallback.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_agent_node.py
"""
Wingman Agent Node — the always-active autonomous brain for Drone-1.

Loophole Fixes Applied:
- Fix #1: Boot self-starts with STANDBY goal.
- Fix #2: Interruptible wait/search loop checking _abort_event.
- Fix #3: Non-blocking ask_lead returning PENDING and monitoring.
- Fix #4: Goal replacement aborting the running thread before re-init.
- Fix #5: SLM health monitor initiating RTL after 5 consecutive failures.
- Fix #7: Envelope-based typed inter-agent messages.
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

# QoS matching the WingmanIntentBridgeNode and monitor subscribers
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from major_project.common.ollama_client import OllamaClient
from major_project.common.tool_registry import WingmanToolRegistry
from major_project.common.context_manager import ContextManager
from major_project.common.agent_memory import AgentMemory
from major_project.common.schemas import AgentMessage


def _load_prompt(filename: str) -> str:
    """Load a system prompt from the prompts/ subdirectory."""
    path = os.path.join(os.path.dirname(__file__), 'prompts', filename)
    with open(path) as f:
        return f.read()


class WingmanAgentNode(Node):

    # Tools that return in <50ms — apply brief pause to avoid SLM hammering
    FAST_TOOLS = frozenset({
        'notify_lead', 'message_lead', 'hover',
        'remember', 'get_battery',
    })

    # After this many consecutive SLM failures → trigger RTL fallback
    MAX_CONSECUTIVE_FAILURES = 5

    # After this many 3s monitoring cycles without Lead response → timeout
    LEAD_WAIT_TIMEOUT_CYCLES = 20   # 20 × 3s = 60s

    def __init__(self):
        super().__init__('wingman_agent_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('ollama_host', 'localhost')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('model', 'qwen2.5-coder:3b')
        self.declare_parameter('num_ctx', 2048)  # raised: 1024 too small for system prompt

        host    = self.get_parameter('ollama_host').value
        port    = self.get_parameter('ollama_port').value
        model   = self.get_parameter('model').value
        self._num_ctx = self.get_parameter('num_ctx').value

        # ── SLM client + system prompt ─────────────────────────────
        self.ollama = OllamaClient(
            host=host, port=port, model=model, num_ctx=self._num_ctx)
        self.system_prompt = _load_prompt('wingman_agent_system.txt')

        # ── Shared sensor state (protected by self.lock) ──────────
        self.lock            = threading.Lock()
        self.own_situation   = ""
        self.camera_summary  = ""
        self.obstacle_vector = ""
        self.battery_pct     = 100.0

        # ── Lead interaction state (non-blocking) ─────────────────
        self._waiting_for_lead  = False
        self._lead_response: str | None = None
        self._lead_wait_cycles  = 0

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
        self.agent_memory = AgentMemory(db_name="wingman_agent_memory.db")
        self.tools        = WingmanToolRegistry(self)

        # ── Publishers ────────────────────────────────────────────
        # RELIABLE_QOS must match WingmanIntentBridgeNode + monitor subscribers
        self.pub_intent         = self.create_publisher(
            String, '/wingman/approved_intent', RELIABLE_QOS)
        self.pub_status_report  = self.create_publisher(
            String, '/wingman/status_report_text', RELIABLE_QOS)
        # Volatile OK — agent/health and lead comms use depth=10 on both sides
        self.pub_lead_msg       = self.create_publisher(
            String, '/agent/wingman_to_lead', 10)
        self.pub_health         = self.create_publisher(
            String, '/agent/health', 10)

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(
            String, '/drone_1/situation',   self._on_situation, 10)
        self.create_subscription(
            String, '/camera_1/detections', self._on_camera, 10)
        self.create_subscription(
            String, '/camera_1/obstacle_vector', self._on_obstacle, 10)
        self.create_subscription(
            String, '/agent/lead_to_wingman', self._on_lead_message, 10)
        self.create_subscription(
            String, '/safety/event',        self._on_safety_event, 10)

        # ── Periodic health publisher ──────────────────────────────
        self.create_timer(10.0, self._publish_health)

        self.get_logger().info(
            f"Wingman Agent ready — Ollama {host}:{port} model:{model} ctx:{self._num_ctx}")

        # ── Fix #1: Self-start with STANDBY goal ──────────────────
        self._assign_goal(
            "STANDBY: Monitor own situation and await mission tasks from "
            "Lead Pilot. Call get_situation() to check state, "
            "then wait(30) and repeat indefinitely.")

    # ════════════════════════════════════════════════════════════════
    # ROS2 Callbacks
    # ════════════════════════════════════════════════════════════════

    def _on_situation(self, msg: String):
        with self.lock:
            self.own_situation = msg.data
            m = re.search(r'bat:(\d+(?:\.\d+)?)', msg.data)
            if m:
                self.battery_pct = float(m.group(1))
        self.ctx.update_situation(msg.data)

    def _on_camera(self, msg: String):
        with self.lock:
            self.camera_summary = msg.data

    def _on_obstacle(self, msg: String):
        with self.lock:
            self.obstacle_vector = msg.data

    def _on_lead_message(self, msg: String):
        """Parse AgentMessage envelope from Lead."""
        try:
            envelope = AgentMessage.model_validate_json(msg.data)
            msg_type = envelope.type
            content  = envelope.content
        except Exception:
            # Legacy fallback: treat raw string as a task
            msg_type = 'task'
            content  = msg.data

        self.get_logger().info(f"Lead [{msg_type}]: {content[:80]}")

        # Fix #3: If waiting for lead reply, this is the answer
        if self._waiting_for_lead and msg_type in ('reply', 'status'):
            self._lead_response = content
            return

        # Handle typed actions (Fix #7)
        if msg_type == 'task':
            self._assign_goal(content)
        elif msg_type == 'abort':
            self._abort_event.set()
        elif msg_type in ('status', 'position'):
            self.ctx.add_inter_agent_message("LEAD", f"[{msg_type.upper()}] {content}")
        elif msg_type == 'query':
            self.ctx.add_inter_agent_message("LEAD", f"[QUERY] {content}")

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
        
        # Inject rigid stopping instruction to prevent SLM over-execution / hallucination
        if not goal.startswith("STANDBY"):
            goal += "\nCRITICAL: When you have finished the requested action, you MUST immediately call mission_complete to lock in your state and await the next command. Do not guess what to do next."
        self.ctx.set_goal(goal)
        self._mission_done       = False
        self._mission_report     = ""
        self._waiting_for_lead   = False
        self._lead_response      = None
        self._lead_wait_cycles   = 0
        self._consecutive_failures = 0
        self._last_substantive_tool = None
        self._slm_healthy        = True

        self._agent_running = True
        threading.Thread(target=self._agent_loop, daemon=True).start()

    # ════════════════════════════════════════════════════════════════
    # Agent Loop (the core think-act-observe cycle)
    # ════════════════════════════════════════════════════════════════

    def _agent_loop(self):
        self.get_logger().info("Wingman agent loop started.")
        self._publish_status("Agent loop active.")

        while rclpy.ok() and not self._mission_done and not self._abort_event.is_set():

            # ── Step 1: Handle pending Lead response (Fix #3) ────────
            if self._waiting_for_lead:
                if self._lead_response is not None:
                    answer = self._lead_response
                    self._lead_response   = None
                    self._waiting_for_lead = False
                    self._lead_wait_cycles = 0
                    self.ctx.add_memory_note(f"[LEAD ANSWERED] {answer}")
                    self.get_logger().info(f"Lead answered: '{answer[:100]}'")
                else:
                    self._lead_wait_cycles += 1

                    if self._lead_wait_cycles >= self.LEAD_WAIT_TIMEOUT_CYCLES:
                        self._waiting_for_lead = False
                        self._lead_wait_cycles = 0
                        self.ctx.add_memory_note(
                            "[LEAD TIMEOUT] No response after 60s. "
                            "Proceeding with best judgment.")
                        self.get_logger().warning(
                            "Lead response timeout — continuing autonomously")
                    else:
                        sit = self.tools.execute('get_situation', {})
                        self.ctx.add_tool_result('get_situation', {}, sit)
                        self._publish_status(
                            f"Waiting for Lead ({self._lead_wait_cycles * 3}s elapsed)…")
                        time.sleep(3.0)
                        continue

            # ── Step 2: Abort check ──────────────────────────────────
            if self._abort_event.is_set():
                break

            # ── Step 3: Build prompt and run SLM inference ───────────
            prompt = self.ctx.build_prompt()
            tool_name, params = self._infer_tool_call(prompt)

            # ── Step 4: Handle SLM failure ───────────────────────────
            if tool_name is None:
                self._consecutive_failures += 1
                self.get_logger().warning(
                    f"SLM parse failure "
                    f"#{self._consecutive_failures}/{self.MAX_CONSECUTIVE_FAILURES}")

                if self._consecutive_failures >= self.MAX_CONSECUTIVE_FAILURES:
                    self._trigger_slm_health_fallback()
                    break

                time.sleep(1.0)
                continue

            # ── Step 5: Successful inference — reset health counter ───
            self._consecutive_failures = 0
            if not self._slm_healthy:
                self._slm_healthy = True
                self.get_logger().info("SLM recovered — health restored.")

            # ── Step 6: Execute tool ─────────────────────────────────
            # SLM loop protection: small models often ignore instructions and repeat commands
            if tool_name not in ['get_situation', 'wait', 'scan_camera', 'get_battery', 'get_wingman_situation']:
                norm_p = {}
                for k, v in params.items():
                    if isinstance(v, str): norm_p[k] = v.lower().strip()
                    elif isinstance(v, (int, float)): norm_p[k] = float(v)
                    else: norm_p[k] = v
                tool_sig = f"{tool_name}:{json.dumps(norm_p, sort_keys=True)}"
                if tool_sig == getattr(self, '_last_substantive_tool', None):
                    self.get_logger().warning(f"SLM LOOP DETECTED on {tool_sig}. Halting drone and auto-completing mission.")
                    self.tools.execute('hover', {})
                    tool_name = 'mission_complete'
                    params = {'report': 'Auto-completed mission to prevent repetitive action loop. Drone halted.'}
                else:
                    self._last_substantive_tool = tool_sig

            self.get_logger().info(
                f"Wingman → {tool_name}({json.dumps(params)[:100]})")
            result = self.tools.execute(tool_name, params)
            self.get_logger().info(f"← {result[:120]}")

            # ── Step 7: Update context ───────────────────────────────
            self.ctx.add_tool_result(tool_name, params, result)
            self._publish_status(f"{tool_name}: {result[:80]}")

            # ── Step 8: Brief pause for instant-return tools only ─────
            # (Fix #2: removed unconditional 0.5s pause)
            if tool_name in self.FAST_TOOLS and not self._mission_done:
                time.sleep(0.2)

        # ── Loop exit ────────────────────────────────────────────────
        if self._mission_done:
            self.get_logger().info(
                f"Task complete: {self._mission_report[:120]}")
            self._publish_status(f"TASK COMPLETE: {self._mission_report}")
            # Notify Lead Agent of completion
            payload = json.dumps({
                "type": "reply", "sender": "WINGMAN",
                "content": f"[TASK COMPLETE] {self._mission_report}",
                "order_id": None
            })
            msg = String()
            msg.data = payload
            self.pub_lead_msg.publish(msg)

        elif self._abort_event.is_set():
            self.get_logger().info("Agent loop aborted — new task incoming.")

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

        # Notify Lead Agent via envelope message
        payload = json.dumps({
            "type": "status", "sender": "WINGMAN",
            "content": f"CRITICAL ALERT: Wingman SLM inference failed {self.MAX_CONSECUTIVE_FAILURES} consecutive times. Initiating RTL fallback.",
            "order_id": None
        })
        msg = String()
        msg.data = payload
        self.pub_lead_msg.publish(msg)

        # Issue RTL directly to commander (bypasses SLM)
        rtl_msg = String()
        rtl_msg.data = json.dumps({'action': 'rtl', 'confidence': 'high'})
        self.pub_intent.publish(rtl_msg)

        self._publish_status("SLM_HEALTH_FAILURE: RTL initiated")
        self._publish_health()

    # ════════════════════════════════════════════════════════════════
    # SLM Inference with 3-attempt retry
    # ════════════════════════════════════════════════════════════════

    def _infer_tool_call(self, prompt: str) -> tuple[str | None, dict]:
        error_ctx = ""
        for attempt in range(3):
            full_prompt = prompt
            if error_ctx:
                full_prompt += (
                    f"\n\n[CORRECTION NEEDED] {error_ctx}"
                    f"\nOutput a valid JSON tool call only. Begin {{ end }}")

            raw, latency = self.ollama.infer(full_prompt, self.system_prompt)
            self.get_logger().debug(f"Inference {latency * 1000:.0f}ms")

            if self._abort_event.is_set():
                return None, {}

            if raw is None:
                error_ctx = "SLM returned no output."
                continue

            try:
                raw_clean = raw.strip()
                # --- SLM Edge Device Optimization: Robust JSON Extraction ---
                # Small models (3B) often leak reasoning or output multiple JSON blocks.
                # A naive rfind('}') grabs everything, corrupting the JSON. We use brace 
                # counting to cleanly extract ONLY the very first complete JSON object.
                start = raw_clean.find('{')
                if start >= 0:
                    brace_count = 0
                    end = -1
                    in_str = False
                    escape = False
                    for i, char in enumerate(raw_clean[start:], start=start):
                        if escape:
                            escape = False
                            continue
                        if char == '\\':
                            escape = True
                        elif char == '"':
                            in_str = not in_str
                        elif not in_str:
                            if char == '{':
                                brace_count += 1
                            elif char == '}':
                                brace_count -= 1
                                if brace_count == 0:
                                    end = i + 1
                                    break
                    if end > start:
                        raw_clean = raw_clean[start:end]

                data      = json.loads(raw_clean)
                tool_name = data.get('tool', '')
                params    = data.get('params', {})

                # --- Edge Model Schema Fallback ---
                # If model hallucinates {"scan_camera": {}} instead of {"tool": "scan_camera", "params": {}}
                if not tool_name:
                    for k, v in data.items():
                        if self.tools.is_valid(k):
                            tool_name = k
                            params = v if isinstance(v, dict) else {}
                            break

                if not isinstance(params, dict):
                    params = {}

                if self.tools.is_valid(tool_name):
                    return tool_name, params

                error_ctx = (
                    f"Unknown tool '{tool_name}'. Raw output was: {raw} "
                    f"Valid tools: {sorted(self.tools.tools.keys())}")
                self.get_logger().warning(f"Inference attempt {attempt+1} failed: {error_ctx}")

            except json.JSONDecodeError as exc:
                error_ctx = (
                    f"JSON parse error: {str(exc)[:80]}. "
                    "Output ONLY {{\"tool\":\"...\",\"params\":{{...}}}}")
                self.get_logger().warning(f"Inference attempt {attempt+1} failed: {error_ctx}")
            except Exception as exc:
                error_ctx = f"Parse error: {str(exc)[:80]}"
                self.get_logger().warning(f"Inference attempt {attempt+1} failed: {error_ctx}")

        return None, {}

    # ════════════════════════════════════════════════════════════════
    # Helpers
    # ════════════════════════════════════════════════════════════════

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status_report.publish(msg)

    def _publish_health(self):
        msg = String()
        msg.data = json.dumps({
            "node":                "wingman",
            "slm_ok":              self._slm_healthy,
            "consecutive_failures": self._consecutive_failures,
            "agent_running":       self._agent_running,
            "waiting_for_lead":    self._waiting_for_lead,
        })
        self.pub_health.publish(msg)


# ════════════════════════════════════════════════════════════════════
# Entry point
# ════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = WingmanAgentNode()
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

## 9.4 Verify Wingman Agent

Run the following commands to compile and verify the Wingman Agent configuration.

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install 2>&1 | tail -5
source install/setup.bash

# ── Import test ────────────────────────────────────────────────────
python3 - << 'PYEOF'
import sys, os
sys.path.insert(0, os.path.expanduser('~/major_ws/src/major_project'))

# Verify module imports without ROS2 runtime
import major_project.wingman_pilot.wingman_agent_node as m
assert hasattr(m, 'WingmanAgentNode'), "WingmanAgentNode class missing"
assert hasattr(m, 'main'), "main() function missing"

# Verify prompt file exists
prompt_path = os.path.expanduser(
    '~/major_ws/src/major_project/major_project/wingman_pilot/prompts/wingman_agent_system.txt')
assert os.path.exists(prompt_path), f"System prompt missing: {prompt_path}"
with open(prompt_path) as f:
    content = f.read()
assert 'STANDBY' in content, "STANDBY rule missing from system prompt"
assert 'PENDING_LEAD_RESPONSE' in content, "Pending response rule missing"
assert 'message_lead' in content, "message_lead missing from tools list"
assert 'follow_lead' in content, "follow_lead missing from tools list"

print("Wingman agent node imports OK")
print("System prompt OK")
print("All checks passed ✅")
PYEOF

# ── Verify entry point registered ─────────────────────────────────
ros2 pkg executables major_project | grep wingman_agent
# Must output: major_project wingman_agent
```

### Live Test (with Ollama running)

```bash
# Terminal 1: Start Wingman sensor aggregator (from Part 6)
ros2 run major_project wingman_sensor_aggregator

# Terminal 2: Start Wingman commander (from Part 7)
ros2 run major_project wingman_px4_commander

# Terminal 3: Start Wingman agent
ros2 run major_project wingman_agent

# Terminal 4: Watch agent output (starts on boot in STANDBY)
ros2 topic echo /wingman/status_report_text
# Expected: get_situation: bat:100% alt:0m mode:MANUAL...
```

### Test Fix #3 (Non-blocking ask_lead)

```bash
# Force the wingman into a task
ros2 topic pub --once /agent/lead_to_wingman std_msgs/msg/String \
  '{data: "{\"type\": \"task\", \"sender\": \"LEAD\", \"content\": \"search for target at N10\"}"}'

# Trigger obstacle detection
ros2 topic pub --rate 3 /camera_1/obstacle_vector std_msgs/msg/String \
  '{data: "car:left:close"}'

# Wingman will call ask_lead — verify it does NOT freeze:
ros2 topic echo /agent/wingman_to_lead   # Query appears here
ros2 topic echo /wingman/status_report_text  # Wingman keeps publishing status
# Expected: status_report continues updating every 3 seconds

# Answer the query from Lead's topic:
ros2 topic pub --once /agent/lead_to_wingman std_msgs/msg/String \
  '{data: "{\"type\": \"reply\", \"sender\": \"LEAD\", \"content\": \"observe and keep distance\"}"}'
# Expected: [LEAD ANSWERED] injected into context, Wingman loop resumes
```

### Test Fix #7 (Goal Replacement and Envelope Validation)

```bash
# Send status message — Wingman should NOT replace task
ros2 topic pub --once /agent/lead_to_wingman std_msgs/msg/String \
  '{data: "{\"type\": \"status\", \"sender\": \"LEAD\", \"content\": \"Lead is at takeoff alt\"}"}'
# Expected: status_report continues with the active search task

# Send new task message — Wingman aborts search and takes off
ros2 topic pub --once /agent/lead_to_wingman std_msgs/msg/String \
  '{data: "{\"type\": \"task\", \"sender\": \"LEAD\", \"content\": \"abort current task and RTL\"}"}'
# Expected: Wingman transitions immediately, status reports RTL
```
