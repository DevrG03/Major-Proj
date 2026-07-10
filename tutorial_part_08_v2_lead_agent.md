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

You run in a think-act-observe loop. At each step output EXACTLY ONE JSON object:
{"thought": "<reasoning about your physical state, battery, and goal>", "tool": "<name>", "params": {"key": value}}
No params? Use: {"thought": "...", "tool": "<name>", "params": {}}
Always start your response with { and end with }. No prose outside the JSON.

══════════════════════════════════════════
AVAILABLE TOOLS:
══════════════════════════════════════════

FLIGHT:
  takeoff(altitude:float)             Arm and ascend to altitude metres (1–30). Returns ETA.
  move(direction:str, distance:float) Fly N/S/E/W/NE/NW/SE/SW/forward/backward/left/right. Returns ETA.
  move(direction, distance, altitude) Fly and change altitude simultaneously.
  follow_road(duration_sec:int)       Visually follow the asphalt road for N seconds. Returns completion.
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
2. After takeoff: call get_situation() to confirm altitude reached.
3. Before every move: call scan_camera() to check for obstacles. If an obstacle is detected in your path, DO NOT ask_human(). Instead, autonomously calculate an evasion path (e.g., hover, or move in a safe direction) and use notify_human() to report your action. If scan_camera() returns "Camera not available", assume the path is clear.
4. After move: call get_situation() to confirm arrival.
5. Battery ≤ 20%: call notify_human immediately. Plan RTL soon.
6. Battery ≤ 15%: call rtl() immediately. Safety monitor also does this independently.
7. Wingman NEVER contacts human — you are the sole human-drone interface.
8. Use message_wingman(msg_type='task') to assign missions to Wingman.
9. Use message_wingman(msg_type='status') for informational updates to Wingman.
10. Use message_wingman(msg_type='reply') to answer Wingman queries.
11. Use ask_human ONLY for: safety decisions, scope changes, genuine uncertainty.
12. Use notify_human for all routine status (no human reply needed).
13. Use remember() for: object positions, mission decisions, notable observations.
14. STANDBY goal: call get_situation(), then wait(30), repeat. Do NOT call mission_complete.
15. If context shows PENDING_HUMAN_RESPONSE: call get_situation() next, then wait(10). Repeat until [HUMAN ANSWERED] appears.
16. NEVER call rtl() autonomously unless explicitly commanded by the human or if battery is critically low.
17. NEVER call takeoff if already airborne (altitude > 1m). Just use move or hover.
18. IMPORTANT: When you have reached your destination or completed the user's goal, you MUST call mission_complete() to end the mission. Do NOT repeat the move command.
19. SWARM TACTIC: ALWAYS instruct the Wingman to takeoff and follow you BEFORE you takeoff yourself.
20. SWARM TACTIC: When conducting a search mission, ALWAYS divide the labor. Assign the Wingman a complementary sector (e.g., if you search North, message Wingman to search East).
21. RTL OVERRIDE: If the human commands you to "return" or "RTL", you MUST first use message_wingman() to order the Wingman to return. On your NEXT turn, you MUST use the rtl() tool. Do NOT manually calculate return paths.

══════════════════════════════════════════
DIRECTION SHORTCUTS:
══════════════════════════════════════════
N=north S=south E=east W=west NE=northeast NW=northwest SE=southeast SW=southwest

══════════════════════════════════════════
EXAMPLES:
══════════════════════════════════════════
Mission start → {"thought":"I just booted up. I need to check my sensors.","tool":"get_situation","params":{}}
Command Wingman → {"thought":"I need my wingman in the air before I takeoff.","tool":"message_wingman","params":{"message":"Takeoff to 10m and follow me at 3m offset.","msg_type":"task"}}
Take off →      {"thought":"Wingman is airborne. I will ascend to 10m.","tool":"takeoff","params":{"altitude":10}}
Confirm →       {"thought":"I called takeoff, now I must verify I reached 10m.","tool":"get_situation","params":{}}
Command Wingman → {"thought":"Splitting search area. I take North, Wingman takes East.","tool":"message_wingman","params":{"message":"I am moving North 50m. You move East 50m then search for 20s.","msg_type":"task"}}
Move north →    {"thought":"Wingman is heading East. I will proceed North to my search zone.","tool":"move","params":{"direction":"N","distance":50}}
Scan area →     {"thought":"I reached North 50m. Commencing camera sweep.","tool":"search","params":{"duration_sec":20}}
Found object →  {"thought":"Camera spotted a football. Saving to memory.","tool":"remember","params":{"fact":"football at pos(50,0) N sector at 10m alt"}}
Tell wingman →  {"thought":"I found the target. Recalling wingman.","tool":"message_wingman","params":{"message":"Football found N50m. Move back to origin.","msg_type":"task"}}
Status GCS →    {"thought":"Updating ground control about the discovery.","tool":"notify_human","params":{"message":"North sector surveyed. Found football at 50m N."}}
Done →          {"thought":"Mission objectives complete. Terminating.","tool":"mission_complete","params":{"report":"Football found 50m north. Wingman covered east. Goal achieved."}}
PROMPT_EOF
```

---

## 8.3 Lead Agent Node (`lead_pilot/lead_agent_node.py`)

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_agent_node.py
"""
Lead Agent Node — the always-active autonomous brain for Drone-0 (V2 with LangGraph).
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
from typing import TypedDict, Annotated, Sequence
import operator

from langchain_core.messages import BaseMessage, HumanMessage, AIMessage, ToolMessage, SystemMessage
from langgraph.graph import StateGraph, END
from langchain_community.chat_models import ChatOllama
from langchain_core.tools import StructuredTool

# QoS matching the LeadIntentBridgeNode and GCS nodes
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from major_project.common.tool_registry import get_lead_tools
from major_project.common.context_manager import ContextManager
from major_project.common.agent_memory import AgentMemory


def _load_prompt(filename: str) -> str:
    path = os.path.join(os.path.dirname(__file__), 'prompts', filename)
    with open(path) as f:
        return f.read()


class AgentState(TypedDict):
    messages: Annotated[Sequence[BaseMessage], operator.add]
    goal: str
    latest_situation: str
    mission_done: bool
    abort_flag: bool


class LeadAgentNode(Node):

    FAST_TOOLS = frozenset({
        'notify_human', 'message_wingman', 'hover',
        'remember', 'get_battery',
    })

    MAX_CONSECUTIVE_FAILURES = 5
    HUMAN_WAIT_TIMEOUT_CYCLES = 40

    def __init__(self):
        super().__init__('lead_agent_node')

        self.declare_parameter('ollama_host', 'localhost')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('model', 'qwen3.5:2b')
        self.declare_parameter('num_ctx', 8192)

        host    = self.get_parameter('ollama_host').value
        port    = self.get_parameter('ollama_port').value
        model   = self.get_parameter('model').value
        self._num_ctx = self.get_parameter('num_ctx').value

        self.llm = ChatOllama(
            base_url=f"http://{host}:{port}",
            model=model,
            num_ctx=self._num_ctx,
            temperature=0
        )
        self.system_prompt = _load_prompt('lead_agent_system.txt')

        self.lock            = threading.Lock()
        self.own_situation   = ""
        self.camera_summary  = ""
        self.obstacle_vector = ""
        self.battery_pct     = 100.0

        self._waiting_for_human  = False
        self._human_response = None
        self._human_wait_cycles  = 0
        self._wingman_query_pending = False

        self._agent_running  = False
        self._mission_done   = False
        self._mission_report = ""
        self._abort_event    = threading.Event()

        self._consecutive_failures = 0
        self._slm_healthy          = True

        self.ctx          = ContextManager()
        self.agent_memory = AgentMemory(db_name="lead_agent_memory.db")
        self.tools        = get_lead_tools(self)
        self.llm_with_tools = self.llm.bind_tools(self.tools)

        self.pub_intent         = self.create_publisher(String, '/lead/approved_intent', RELIABLE_QOS)
        self.pub_clarification  = self.create_publisher(String, '/clarification_request', RELIABLE_QOS)
        self.pub_mission_status = self.create_publisher(String, '/mission_status', RELIABLE_QOS)
        self.pub_wingman_msg    = self.create_publisher(String, '/agent/lead_to_wingman', 10)
        self.pub_health         = self.create_publisher(String, '/agent/health', RELIABLE_QOS)

        self.create_subscription(String, '/drone_0/situation',   self._on_situation, 10)
        self.create_subscription(String, '/camera_0/detections', self._on_camera, 10)
        self.create_subscription(String, '/camera_0/obstacle_vector', self._on_obstacle, 10)
        self.create_subscription(String, '/voice_commands',      self._on_voice, 10)
        self.create_subscription(String, '/agent/wingman_to_lead', self._on_wingman_message, 10)
        self.create_subscription(String, '/safety/event',        self._on_safety_event, 10)

        self.create_timer(10.0, self._publish_health)

        self._build_graph()

        self.get_logger().info(f"Lead Agent (LangGraph V2) ready — Ollama {host}:{port} model:{model}")
        self._assign_goal(
            "STANDBY: Monitor drone situation and await mission goal from "
            "Ground Commander. Call get_situation() to check state, "
            "then wait(30) and repeat indefinitely.")

    def _build_graph(self):
        workflow = StateGraph(AgentState)
        
        workflow.add_node("llm", self._llm_node)
        workflow.add_node("tools", self._tool_node)
        
        workflow.set_entry_point("llm")
        
        def should_continue(state: AgentState):
            if state.get("abort_flag", False) or state.get("mission_done", False):
                return END
            last_msg = state["messages"][-1]
            if hasattr(last_msg, 'tool_calls') and last_msg.tool_calls:
                return "tools"
            return END
            
        workflow.add_conditional_edges("llm", should_continue, {"tools": "tools", END: END})
        workflow.add_edge("tools", "llm")
        
        self.graph = workflow.compile()

    def _llm_node(self, state: AgentState):
        if self._abort_event.is_set():
            return {"abort_flag": True}
            
        sys_msg = SystemMessage(content=self.system_prompt + f"\n\n[GOAL]\n{state['goal']}\n\n[SITUATION]\n{state['latest_situation']}")
        msgs = [sys_msg] + list(state["messages"])
        
        try:
            response = self.llm_with_tools.invoke(msgs)
            self._consecutive_failures = 0
            self._slm_healthy = True
            return {"messages": [response]}
        except Exception as e:
            self.get_logger().warning(f"SLM inference failed: {e}")
            self._consecutive_failures += 1
            if self._consecutive_failures >= self.MAX_CONSECUTIVE_FAILURES:
                self._trigger_slm_health_fallback()
                return {"abort_flag": True}
            time.sleep(1.0)
            return {"messages": [AIMessage(content="{}", additional_kwargs={"error": str(e)})]}

    def _tool_node(self, state: AgentState):
        if self._abort_event.is_set():
            return {"abort_flag": True}
            
        last_msg = state["messages"][-1]
        results = []
        for call in getattr(last_msg, 'tool_calls', []):
            tool_name = call['name']
            params = call['args']
            tool_id = call['id']
            
            self.get_logger().info(f"Lead → {tool_name}({json.dumps(params)[:100]})")
            
            tool_fn = next((t for t in self.tools if t.name == tool_name), None)
            if tool_fn:
                try:
                    result = str(tool_fn.invoke(params))
                except Exception as e:
                    result = f"Error executing {tool_name}: {e}"
            else:
                result = f"Unknown tool '{tool_name}'."
                
            self.get_logger().info(f"← {result[:120]}")
            self._publish_status(f"{tool_name}: {result[:80]}")
            
            if tool_name in self.FAST_TOOLS and not self._mission_done:
                time.sleep(0.2)
                
            if tool_name == 'mission_complete':
                self._mission_done = True
                self._mission_report = result
                
            results.append(ToolMessage(content=result, tool_call_id=tool_id, name=tool_name))
            
        return {"messages": results, "mission_done": self._mission_done}

    # Callbacks
    def _on_situation(self, msg: String):
        with self.lock:
            sit = msg.data
            m = re.search(r'bat:(\d+(?:\.\d+)?)', sit)
            if m: self.battery_pct = float(m.group(1))
                
            has_obs = False
            if hasattr(self, 'camera_summary') and self.camera_summary:
                cam = self.camera_summary.lower()
                if 'no detection' not in cam and 'clear' not in cam and 'not available' not in cam:
                    has_obs = True
                    
            if has_obs:
                sit += f"\n[CRITICAL SAFETY ALERT] {self.camera_summary}"
                if hasattr(self, 'obstacle_vector') and self.obstacle_vector:
                    sit += f" ({self.obstacle_vector})"
                sit += "\nYou MUST call ask_human() immediately to handle this obstacle!"
                
            self.own_situation = sit
        self.ctx.update_situation(sit)

    def _on_camera(self, msg: String):
        with self.lock: self.camera_summary = msg.data

    def _on_obstacle(self, msg: String):
        with self.lock: self.obstacle_vector = msg.data

    def _on_voice(self, msg: String):
        text = msg.data.strip()
        if not text: return
        self.get_logger().info(f"Voice: '{text}'")
        if self._waiting_for_human:
            self._human_response = text
            return
        self._assign_goal(text)

    def _on_wingman_message(self, msg: String):
        try:
            data     = json.loads(msg.data)
            msg_type = data.get('type', 'status')
            content  = data.get('content', msg.data)
        except:
            msg_type = 'status'
            content  = msg.data
        self.get_logger().info(f"Wingman [{msg_type}]: {content[:80]}")
        self.ctx.add_inter_agent_message("WINGMAN", f"[{msg_type.upper()}] {content}")
        if msg_type == 'query': self._wingman_query_pending = True

    def _on_safety_event(self, msg: String):
        try:
            data = json.loads(msg.data)
            note = data.get('message', msg.data)
        except:
            note = msg.data
        self.ctx.add_memory_note(f"[SAFETY] {note}")
        self.get_logger().warning(f"Safety event: {note[:100]}")

    def _assign_goal(self, goal: str):
        self.get_logger().info(f"New goal: '{goal[:100]}'")
        if self._agent_running:
            self._abort_event.set()
            while self._agent_running: time.sleep(0.05)
            
        self._abort_event.clear()
        self.ctx.clear_history()
        
        if not goal.startswith("STANDBY"):
            goal += "\nCRITICAL: When you have finished the requested action, you MUST immediately call mission_complete to lock in your state and await the next command. Do not guess what to do next."
        
        self.ctx.set_goal(goal)
        self._mission_done       = False
        self._mission_report     = ""
        self._waiting_for_human  = False
        self._human_response     = None
        self._human_wait_cycles  = 0
        self._consecutive_failures = 0
        self._slm_healthy        = True

        self._agent_running = True
        threading.Thread(target=self._run_graph, args=(goal,), daemon=True).start()

    def _run_graph(self, goal: str):
        self.get_logger().info("LangGraph execution started.")
        self._publish_status("Agent loop active.")
        
        initial_state = {
            "messages": [HumanMessage(content="Start mission.")],
            "goal": goal,
            "latest_situation": self.own_situation,
            "mission_done": False,
            "abort_flag": False
        }
        
        while rclpy.ok() and not self._mission_done and not self._abort_event.is_set():
            
            if self._waiting_for_human:
                if self._human_response is not None:
                    answer = self._human_response
                    self._human_response    = None
                    self._waiting_for_human = False
                    self._human_wait_cycles = 0
                    initial_state["messages"].append(HumanMessage(content=f"[HUMAN ANSWERED] {answer}"))
                else:
                    self._human_wait_cycles += 1
                    if self._human_wait_cycles >= self.HUMAN_WAIT_TIMEOUT_CYCLES:
                        self._waiting_for_human = False
                        self._human_wait_cycles = 0
                        initial_state["messages"].append(HumanMessage(content="[HUMAN TIMEOUT] No response. Proceed autonomously."))
                    else:
                        time.sleep(3.0)
                        continue
                        
            if self._wingman_query_pending:
                self._wingman_query_pending = False
                initial_state["messages"].append(HumanMessage(content="[NOTE] Wingman has a pending query."))
                
            initial_state["latest_situation"] = self.own_situation
            
            result_state = self.graph.invoke(initial_state)
            
            if result_state.get("abort_flag") or self._abort_event.is_set():
                break
                
            if result_state.get("mission_done"):
                break
                
            initial_state["messages"] = result_state["messages"][-6:]
            initial_state["messages"].append(HumanMessage(content="Continue next step."))
            time.sleep(1.0)
            
        if self._mission_done:
            self.get_logger().info(f"Mission complete: {self._mission_report[:120]}")
            self._publish_status(f"MISSION COMPLETE: {self._mission_report}")
            done_msg = String()
            done_msg.data = f"[MISSION COMPLETE] {self._mission_report}"
            self.pub_clarification.publish(done_msg)
        elif self._abort_event.is_set():
            self.get_logger().info("Agent loop aborted — new goal incoming.")
            
        self._agent_running = False

    def _trigger_slm_health_fallback(self):
        self._slm_healthy = False
        self.get_logger().error(f"SLM health failure — initiating RTL fallback")
        alert = String()
        alert.data = "CRITICAL ALERT: Lead SLM inference failed. RTL initiated."
        self.pub_clarification.publish(alert)
        rtl_msg = String()
        rtl_msg.data = json.dumps({'action': 'rtl', 'confidence': 'high'})
        self.pub_intent.publish(rtl_msg)
        self._publish_status("SLM_HEALTH_FAILURE: RTL initiated")
        self._publish_health()

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
