# Part 9: Wingman Pilot Agent (V2 - LangGraph Upgrade)

> **Target:** PC-2 (Wingman Workstation)
> **Prerequisites:** Part 8 (V2) complete.

Just like the Lead Agent, the Wingman Agent receives a V2 upgrade to use a LangGraph `StateGraph`. This removes the fragile `while` loop, allowing deterministic task planning and execution. The Wingman receives goals via `AgentMessage` envelopes from the Lead and formulates a plan using its Planner node.

---

## 9.1 System Prompts (`wingman_pilot/prompts/`)

Create the system prompt for the Wingman Planner. It requires strict adherence to swarm rules (never contact GCS, avoid Lead drone).

```bash
mkdir -p ~/major_ws/src/major_project/major_project/wingman_pilot/prompts

cat << 'PROMPT_EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/prompts/wingman_planner_system.txt
You are WINGMAN PILOT (Drone-1). You receive goals from Lead Pilot (Drone-0).
Your job is to plan how to execute the Lead's instructions by generating a sequential checklist of tool calls.

Output EXACTLY ONE JSON object matching this schema:
{
  "thought": "Reasoning about how to achieve the goal.",
  "checklist": [
    {"tool": "tool_name", "params": {"key": value}},
    {"tool": "tool_name", "params": {}}
  ]
}

══════════════════════════════════════════
AVAILABLE TOOLS:
══════════════════════════════════════════
FLIGHT:
  takeoff(altitude:float)
  move(direction:str, distance:float, altitude:float|None)
  hover()
  follow_road(duration_sec:int)
  search(duration_sec:int)
  follow_lead(offset_m:float)
  land()
  rtl()

COMMUNICATION:
  message_lead(message:str, msg_type:str)
  ask_lead(question:str)
  notify_lead(message:str)

COMPLETION:
  mission_complete(message:str)

══════════════════════════════════════════
RULES:
══════════════════════════════════════════
1. NEVER contact the human GCS operator — all comms go to Lead via message_lead or notify_lead.
2. ALWAYS end your checklist with mission_complete to inform the Lead you finished the task.
3. If battery <= 20%, your checklist must consist of notify_lead and rtl.
4. TASK ISOLATION: Do NOT execute the Lead's task! If Lead says "I am moving North. You move East", your checklist MUST ONLY move East.
5. COLLISION AVOIDANCE: NEVER approach within 5.0m of the Lead drone.
6. If already flying (alt > 1m), DO NOT use takeoff.
PROMPT_EOF
```

---

## 9.2 Wingman Agent Node (`wingman_pilot/wingman_agent_node.py`)

We replace the while loop with the `StateGraph`.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/wingman_agent_node.py
"""
Wingman Agent Node V2 — LangGraph implementation.

Deterministic State Graph:
[ START ] ──> [ PLANNER ] ──> [ EXECUTOR ]
                                   ↑     │
                                   └─────┘
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import json
import os
import threading
import time
import re

from langgraph.graph import StateGraph, END
import sys
sys.path.insert(0, os.path.expanduser('~/major_ws/src/major_project'))

from major_project.common.ollama_client import OllamaClient
from major_project.common.tool_registry import WingmanToolRegistry
from major_project.common.schemas import AgentState, AgentMessage
from pydantic import BaseModel, Field

class TaskItem(BaseModel):
    tool: str = Field(description="The exact name of the tool to use")
    params: dict = Field(default_factory=dict, description="A dictionary of arguments for the tool")

class PlannerOutput(BaseModel):
    thought: str = Field(description="Step-by-step reasoning")
    checklist: list[TaskItem] = Field(description="The sequential list of tasks")

RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

class WingmanAgentNode(Node):

    def __init__(self):
        super().__init__('wingman_agent_node')
        self.ollama = OllamaClient(model='qwen3.5:2b', num_ctx=8192)
        
        prompt_path = os.path.join(os.path.dirname(__file__), 'prompts', 'wingman_planner_system.txt')
        with open(prompt_path) as f:
            self.system_prompt = f.read()

        self.lock = threading.Lock()
        self.own_situation = ""
        self.camera_summary = ""
        self.obstacle_vector = ""
        self.battery_pct = 100.0
        
        self._abort_event = threading.Event()
        self._mission_done = False
        self._lead_response = None
        self._waiting_for_lead = False

        from major_project.common.agent_memory import AgentMemory
        self.agent_memory = AgentMemory("wingman_agent_memory.db")
        self.tool_registry = WingmanToolRegistry(self)
        self.tools = {t.name: t for t in self.tool_registry.get_tools()}

        self.pub_intent = self.create_publisher(String, '/wingman/approved_intent', RELIABLE_QOS)
        self.pub_status_report = self.create_publisher(String, '/wingman/status_report_text', RELIABLE_QOS)
        self.pub_lead_msg = self.create_publisher(String, '/agent/wingman_to_lead', 10)

        self.create_subscription(String, '/drone_1/situation', self._on_situation, 10)
        self.create_subscription(String, '/camera_1/detections', self._on_camera, 10)
        self.create_subscription(String, '/camera_1/obstacle_vector', self._on_obstacle, 10)
        self.create_subscription(String, '/agent/lead_to_wingman', self._on_lead_message, 10)

        # Agent Health
        self.health_pub = self.create_publisher(String, '/agent/health', RELIABLE_QOS)
        self.health_timer = self.create_timer(10.0, self._publish_health)

        self.get_logger().info("Wingman Agent V2 (LangGraph) ready.")

        self.graph = self._build_graph()
        self._assign_goal("STANDBY: Hover and wait for Lead Pilot instructions.")

    # ── Diagnostics ───────────────────────────────────────────────────────────
    def _publish_health(self):
        msg = String()
        fails = getattr(self, '_consecutive_failures', 0)
        health_data = {
            "node": "wingman",
            "slm_ok": fails < 5,
            "consecutive_failures": fails
        }
        import json
        msg.data = json.dumps(health_data)
        self.health_pub.publish(msg)

    # ── ROS 2 Callbacks ───────────────────────────────────────────────────────
    
    def _on_situation(self, msg: String):
        with self.lock:
            sit = msg.data
            m = re.search(r'bat:(\d+(?:\.\d+)?)', sit)
            if m: self.battery_pct = float(m.group(1))
            
            has_obs = False
            if getattr(self, 'camera_summary', ''):
                cam = self.camera_summary.lower()
                if 'no detection' not in cam and 'clear' not in cam and 'not available' not in cam:
                    has_obs = True
                    
            if has_obs:
                sit += f"\n[CRITICAL SAFETY ALERT] {self.camera_summary}"
                if getattr(self, 'obstacle_vector', ''):
                    sit += f" ({self.obstacle_vector})"
                sit += "\nYou MUST call ask_lead() immediately to handle this obstacle!"
                
            self.own_situation = sit

    def _on_camera(self, msg: String):
        with self.lock:
            self.camera_summary = msg.data

    def _on_obstacle(self, msg: String):
        with self.lock:
            self.obstacle_vector = msg.data

    def _on_lead_message(self, msg: String):
        try:
            envelope = AgentMessage.model_validate_json(msg.data)
            msg_type = envelope.type
            content = envelope.content
        except Exception:
            msg_type = 'task'
            content = msg.data

        self.get_logger().info(f"Lead [{msg_type}]: {content[:80]}")

        if self._waiting_for_lead and msg_type in ('reply', 'status'):
            self._lead_response = content
            self._waiting_for_lead = False
            return

        if msg_type == 'task':
            self._assign_goal(content)
        elif msg_type == 'abort':
            self._abort_event.set()

    # ── Graph Compilation ────────────────────────────────────────────

    def _build_graph(self):
        builder = StateGraph(AgentState)
        builder.add_node("planner", self._node_planner)
        builder.add_node("executor", self._node_executor)
        
        builder.set_entry_point("planner")
        
        def executor_router(state: AgentState):
            if self._mission_done or self._abort_event.is_set():
                return END
            if state["current_task_index"] >= len(state["checklist"]):
                return "planner"
            return "executor"

        builder.add_conditional_edges("planner", lambda s: "executor" if s["checklist"] else END)
        builder.add_conditional_edges("executor", executor_router)
        return builder.compile()

    # ── Node Implementations ─────────────────────────────────────────

    def _node_planner(self, state: AgentState):
        self.get_logger().info("Running Planner Node...")
        
        context = f"[LEAD COMMAND]\n{state['mission_goal']}\n\n[SITUATION]\n{self.own_situation}"
        
        # Patch 4: Context Window Trimming (keep only last 6 messages)
        recent_messages = state.get("messages", [])[-6:]
        if recent_messages:
            context += "\n\n[RECENT ACTIONS]\n" + "\n".join([m.content for m in recent_messages])

        # Infer Checklist using Pydantic JSON Schema Native Integration
        schema = PlannerOutput.model_json_schema()
        raw_json, _ = self.ollama.infer(context, self.system_prompt, schema=schema)
        
        # Patch 3: SLM Health Tracking
        if not getattr(self, '_consecutive_failures', False):
            self._consecutive_failures = 0
            
        if not raw_json:
            self._consecutive_failures += 1
            self.get_logger().error(f"Planner failed to return JSON. Failures: {self._consecutive_failures}")
            if self._consecutive_failures >= 5:
                self._trigger_slm_health_fallback()
                self._abort_event.set()
            return {"checklist": [], "current_task_index": 0}

        try:
            # We can now confidently use Pydantic to validate the entire LLM output
            parsed_data = PlannerOutput.model_validate_json(raw_json)
            checklist = [item.model_dump() for item in parsed_data.checklist]
            
            self._consecutive_failures = 0 # Reset on success
            return {"checklist": checklist, "current_task_index": 0}
        except Exception as e:
            self._consecutive_failures += 1
            self.get_logger().error(f"Failed to parse planner output ({e}). Failures: {self._consecutive_failures}")
            if self._consecutive_failures >= 5:
                self._trigger_slm_health_fallback()
                self._abort_event.set()
            return {"checklist": [], "current_task_index": 0}

    def _node_executor(self, state: AgentState):
        task_idx = state["current_task_index"]
        task = state["checklist"][task_idx]
        tool_name = task.get("tool")
        params = task.get("params", {})
        
        # Patch 2: LangGraph Recursion Deadlock Fix
        wait_cycles = 0
        while getattr(self, '_waiting_for_lead', False) and not self._abort_event.is_set():
            self.get_logger().info(f"Waiting for lead response ({wait_cycles * 3}s elapsed)...")
            time.sleep(3.0)
            wait_cycles += 1
            if wait_cycles >= 20:
                self.get_logger().warning("Lead response timeout (60s). Proceeding autonomously.")
                self._waiting_for_lead = False
                self._lead_response = "TIMEOUT: No response received. Proceed with best judgment."
                break
            
        if self._abort_event.is_set():
            return {"current_task_index": task_idx}

        # Patch 6: Asynchronous Clarification Loophole
        ans = getattr(self, '_lead_response', None)
        if ans:
            self._lead_response = None
            from langchain_core.messages import AIMessage
            new_msg = AIMessage(content=f"[LEAD ANSWERED] {ans}")
            self.get_logger().info(f"Lead answered: {ans}. Forcing replan.")
            return {"current_task_index": 999, "messages": [new_msg]}

        if tool_name not in self.tools:
            return {"current_task_index": task_idx + 1}
            
        self.get_logger().info(f"Executing: {tool_name}({params})")
        
        try:
            result = self.tools[tool_name].invoke(params)
        except Exception as e:
            result = f"Error: {str(e)}"
            
        if tool_name == 'mission_complete':
            self._mission_done = True
            msg = String()
            msg.data = json.dumps({"type": "reply", "sender": "WINGMAN", "content": f"[TASK COMPLETE] {params.get('message', '')}", "order_id": None})
            self.pub_lead_msg.publish(msg)

        from langchain_core.messages import AIMessage
        new_msg = AIMessage(content=f"Completed {tool_name}: {result}")
        return {"current_task_index": task_idx + 1, "messages": [new_msg]}

    # ── Mission Control ──────────────────────────────────────────────

    def _assign_goal(self, goal: str):
        if getattr(self, '_agent_running', False):
            self._abort_event.set()
            while getattr(self, '_agent_running', False):
                time.sleep(0.05)
        
        self._abort_event.clear()
        self._mission_done = False
        self._waiting_for_lead = False
        self._lead_response = None
        
        self.get_logger().info(f"New Task: {goal}")
        self._agent_running = True
        threading.Thread(target=self._run_graph, args=(goal,), daemon=True).start()

    def _run_graph(self, goal: str):
        state = {
            "messages": [],
            "checklist": [],
            "current_task_index": 0,
            "sensor_telemetry": self.own_situation,
            "mission_goal": goal
        }
        
        try:
            for event in self.graph.stream(state, {"recursion_limit": 50}):
                if self._abort_event.is_set(): break
        except Exception as e:
            self.get_logger().error(f"Graph error: {e}")
        finally:
            self._agent_running = False

    def _trigger_slm_health_fallback(self):
        self.get_logger().error("SLM health failure (5 consecutive failures) — initiating RTL fallback")
        msg = String()
        msg.data = json.dumps({
            "type": "abort", "sender": "WINGMAN",
            "content": "CRITICAL ALERT: Wingman SLM inference failed. Initiating RTL fallback.",
            "order_id": None
        })
        self.pub_lead_msg.publish(msg)

        rtl_msg = String()
        rtl_msg.data = json.dumps({'action': 'rtl', 'confidence': 'high'})
        self.pub_intent.publish(rtl_msg)

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

## 9.3 Verify Wingman Agent V2

The V2 implementation relies on `langgraph`. Ensure dependencies are met.

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash

# Ensure dependencies
pip3 install langgraph langchain-core langchain-ollama pydantic

# Check entrypoint
ros2 pkg executables major_project | grep wingman_agent
```
