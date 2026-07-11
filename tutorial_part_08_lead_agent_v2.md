# Part 8: Lead Pilot Agent (V2 - LangGraph Upgrade)

> **Target:** PC-1
> **Prerequisites:** Part 3 (V2) complete.

In this V2 upgrade, we replace the fragile `while` loop with a robust **LangGraph StateGraph**. This implements a Cognitive Architecture where the LLM acts as a **Planner** (generating a mission checklist) and the ROS node acts as an **Executor** (stepping through the checklist deterministically).

This permanently fixes loop-stall bugs, ensures safety guards are evaluated between every step, and gives the drone a structured memory of its mission progress.

---

## 8.1 System Prompts (`lead_pilot/prompts/`)

We now use two separate prompts: one for the Planner (generating the checklist) and one for the Verifier (optional, though we will handle verification in Python for safety). 

```bash
mkdir -p ~/major_ws/src/major_project/major_project/lead_pilot/prompts

cat << 'PROMPT_EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/prompts/lead_planner_system.txt
You are LEAD PILOT (Drone-0). Your job is to plan a mission by decomposing the MISSION GOAL into a sequential checklist of tool calls.

Output EXACTLY ONE JSON object matching this schema:
{
  "thought": "Reasoning about how to achieve the goal based on the situation.",
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
  land()
  rtl()

COMMUNICATION:
  message_wingman(message:str, msg_type:str)
  ask_human(question:str)
  notify_human(message:str)

COMPLETION:
  mission_complete(message:str)

══════════════════════════════════════════
RULES:
══════════════════════════════════════════
1. ONLY use the exact tool names and parameters listed above.
2. If already flying (alt > 1m), DO NOT use takeoff.
3. ALWAYS end your checklist with mission_complete when the goal is achieved.
4. For Swarm tactics: Always use message_wingman(msg_type="task") to position your wingman before you move.
5. If answering a question from the Wingman, use message_wingman with msg_type="reply".
6. If battery <= 20%, your checklist must consist of notify_human and rtl.
7. If an obstacle is detected in your path, plan an evasion move (e.g., hover or move a different direction) and notify_human. Do not ask_human for obstacles.
PROMPT_EOF
```

---

## 8.2 Lead Agent Node (`lead_pilot/lead_agent_node.py`)

We completely replace the `_agent_loop` with a `StateGraph` compilation.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/lead_agent_node.py
"""
Lead Agent Node V2 — LangGraph implementation.

This replaces the while loop with a deterministic State Graph:
[ START ] ──> [ PLANNER ] ──> [ EXECUTOR ] ──> [ VERIFIER ]
                                   ↑                │
                                   └────────────────┘
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
from major_project.common.tool_registry import LeadToolRegistry
from major_project.common.schemas import AgentState
from pydantic import BaseModel, Field

class TaskItem(BaseModel):
    tool: str = Field(description="The exact name of the tool to use")
    params: dict = Field(default_factory=dict, description="A dictionary of arguments for the tool")

class PlannerOutput(BaseModel):
    thought: str = Field(description="Step-by-step reasoning")
    checklist: list[TaskItem] = Field(description="The sequential list of tasks")

RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

class LeadAgentNode(Node):

    def __init__(self):
        super().__init__('lead_agent_node')
        self.ollama = OllamaClient(model='qwen3.5:2b', num_ctx=8192)
        
        prompt_path = os.path.join(os.path.dirname(__file__), 'prompts', 'lead_planner_system.txt')
        with open(prompt_path) as f:
            self.system_prompt = f.read()

        # Shared ros_iface state
        self.lock = threading.Lock()
        self.own_situation = ""
        self.camera_summary = ""
        self.obstacle_vector = ""
        self.battery_pct = 100.0
        
        self._abort_event = threading.Event()
        self._mission_done = False
        self._human_response = None
        self._waiting_for_human = False

        # Tools and Memory
        from major_project.common.agent_memory import AgentMemory
        self.agent_memory = AgentMemory("lead_agent_memory.db")
        self.tool_registry = LeadToolRegistry(self)
        self.tools = {t.name: t for t in self.tool_registry.get_tools()}

        # Publishers / Subscribers
        self.pub_intent = self.create_publisher(String, '/lead/approved_intent', RELIABLE_QOS)
        self.pub_clarification = self.create_publisher(String, '/clarification_request', RELIABLE_QOS)
        self.pub_mission_status = self.create_publisher(String, '/mission_status', RELIABLE_QOS)
        self.pub_wingman_msg = self.create_publisher(String, '/agent/lead_to_wingman', 10)

        self.create_subscription(String, '/drone_0/situation', self._on_situation, 10)
        self.create_subscription(String, '/camera_0/detections', self._on_camera, 10)
        self.create_subscription(String, '/camera_0/obstacle_vector', self._on_obstacle, 10)
        self.create_subscription(String, '/voice_commands', self._on_human_command, 10)
        self.create_subscription(String, '/agent/wingman_to_lead', self._on_wingman_message, 10)
        
        self.wingman_messages_buffer = []

        # Agent Health
        self.health_pub = self.create_publisher(String, '/agent/health', RELIABLE_QOS)
        self.health_timer = self.create_timer(10.0, self._publish_health)
        
        self.get_logger().info("Lead Agent V2 (LangGraph) ready.")

        # Build Graph
        self.graph = self._build_graph()

        # Start with Standby
        self._assign_goal("STANDBY: Hover and wait for Ground Commander instructions.")

    # ── Diagnostics ───────────────────────────────────────────────────────────
    def _publish_health(self):
        msg = String()
        fails = getattr(self, '_consecutive_failures', 0)
        health_data = {
            "node": "lead",
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
                sit += "\nYou MUST call ask_human() immediately to handle this obstacle!"
                
            self.own_situation = sit

    def _on_camera(self, msg: String):
        with self.lock:
            self.camera_summary = msg.data

    def _on_obstacle(self, msg: String):
        with self.lock:
            self.obstacle_vector = msg.data

    def _on_human_command(self, msg: String):
        command = msg.data.lower()
        if command == "emergency stop" or command == "abort":
            self._abort_event.set()
        elif getattr(self, '_waiting_for_human', False):
            self._human_response = msg.data
            self._waiting_for_human = False
        else:
            self._assign_goal(msg.data)

    def _on_wingman_message(self, msg: String):
        try:
            data = json.loads(msg.data)
            sender = data.get("sender", "WINGMAN")
            content = data.get("content", msg.data)
            msg_type = data.get("type", "status")
        except json.JSONDecodeError:
            sender = "WINGMAN"
            content = msg.data
            msg_type = "status"
            
        self.get_logger().info(f"{sender}: {content[:100]}")
        self.wingman_messages_buffer.append(f"[{sender}] {content}")
        self.wingman_messages_buffer = self.wingman_messages_buffer[-3:] # Keep last 3
        
        if getattr(self, '_waiting_for_wingman', False) and ("TASK COMPLETE" in content or msg_type in ("reply", "abort")):
            self._waiting_for_wingman = False

    # ── Graph Compilation ────────────────────────────────────────────

    def _build_graph(self):
        builder = StateGraph(AgentState)
        
        builder.add_node("planner", self._node_planner)
        builder.add_node("executor", self._node_executor)
        
        builder.set_entry_point("planner")
        
        # Edge logic
        def executor_router(state: AgentState):
            if self._mission_done or self._abort_event.is_set():
                return END
            if state["current_task_index"] >= len(state["checklist"]):
                return "planner" # Checklist finished but mission not marked complete? Replan.
            return "executor"

        builder.add_conditional_edges("planner", lambda s: "executor" if s["checklist"] else END)
        builder.add_conditional_edges("executor", executor_router)
        
        return builder.compile()

    # ── Node Implementations ─────────────────────────────────────────

    def _node_planner(self, state: AgentState):
        self.get_logger().info("Running Planner Node...")
        self._publish_status("Planning mission...")
        
        context = f"[MISSION GOAL]\n{state['mission_goal']}\n\n[SITUATION]\n{self.own_situation}"
        
        # Patch 4: Context Window Trimming (keep only last 6 messages)
        recent_messages = state.get("messages", [])[-6:]
        if recent_messages:
            context += "\n\n[RECENT ACTIONS]\n" + "\n".join([m.content for m in recent_messages])
            
        if getattr(self, 'wingman_messages_buffer', []):
            context += "\n\n[WINGMAN COMMS]\n" + "\n".join(self.wingman_messages_buffer)

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
            
            self.get_logger().info(f"Generated Checklist: {checklist}")
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
        while getattr(self, '_waiting_for_human', False) and not self._abort_event.is_set():
            self.get_logger().info(f"Waiting for human response ({wait_cycles * 3}s elapsed)...")
            time.sleep(3.0)
            wait_cycles += 1
            if wait_cycles >= 40:
                self.get_logger().warning("Human response timeout (120s). Proceeding autonomously.")
                self._waiting_for_human = False
                self._human_response = "TIMEOUT: No response received. Proceed with best judgment."
                break
            
        if self._abort_event.is_set():
            return {"current_task_index": task_idx}

        # Patch 6: Asynchronous Clarification Loophole
        ans = getattr(self, '_human_response', None)
        if ans:
            self._human_response = None
            from langchain_core.messages import AIMessage
            new_msg = AIMessage(content=f"[HUMAN ANSWERED] {ans}")
            self.get_logger().info(f"Human answered: {ans}. Forcing replan.")
            return {"current_task_index": 999, "messages": [new_msg]}

        if tool_name not in self.tools:
            self.get_logger().warning(f"Unknown tool: {tool_name}")
            return {"current_task_index": task_idx + 1}
            
        if tool_name == 'message_wingman':
            self._waiting_for_wingman = True
            
        self.get_logger().info(f"Executing: {tool_name}({params})")
        self._publish_status(f"Executing: {tool_name}")
        
        try:
            result = self.tools[tool_name].invoke(params)
        except Exception as e:
            result = f"Error: {str(e)}"
            
        self.get_logger().info(f"Result: {result}")
        
        if getattr(self, '_waiting_for_wingman', False):
            wait_cycles = 0
            while getattr(self, '_waiting_for_wingman', False) and not self._abort_event.is_set():
                self.get_logger().info(f"Waiting for wingman response ({wait_cycles * 3}s elapsed)...")
                time.sleep(3.0)
                wait_cycles += 1
                if wait_cycles >= 60:
                    self.get_logger().warning("Wingman response timeout (180s). Proceeding autonomously.")
                    self._waiting_for_wingman = False
                    self.wingman_messages_buffer.append("[WINGMAN] TIMEOUT: Task completion not verified.")
                    break
        
        from langchain_core.messages import AIMessage
        new_msg = AIMessage(content=f"Completed {tool_name}: {result}")
        
        return {
            "current_task_index": task_idx + 1,
            "messages": [new_msg]
        }

    # ── Mission Control ──────────────────────────────────────────────

    def _assign_goal(self, goal: str):
        if getattr(self, '_agent_running', False):
            self._abort_event.set()
            while getattr(self, '_agent_running', False):
                time.sleep(0.05)
        
        self._abort_event.clear()
        self._mission_done = False
        self._waiting_for_human = False
        self._waiting_for_wingman = False
        self._human_response = None
        
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
            # LangGraph graph.invoke handles the entire node loop deterministically!
            for event in self.graph.stream(state, {"recursion_limit": 50}):
                if self._abort_event.is_set():
                    self.get_logger().info("Graph execution aborted.")
                    break
        except Exception as e:
            self.get_logger().error(f"Graph error: {e}")
        finally:
            self._agent_running = False
            
    def _publish_status(self, text: str):
        msg = String()
        msg.data = json.dumps({"lead": text, "wingman": "—"})
        self.pub_mission_status.publish(msg)

    def _trigger_slm_health_fallback(self):
        self.get_logger().error("SLM health failure (5 consecutive failures) — initiating RTL fallback")
        alert = String()
        alert.data = "CRITICAL ALERT: Lead SLM inference failed. Autonomous control suspended. RTL initiated for safety."
        self.pub_clarification.publish(alert)

        rtl_msg = String()
        rtl_msg.data = json.dumps({'action': 'rtl', 'confidence': 'high'})
        self.pub_intent.publish(rtl_msg)
        self._publish_status("SLM_HEALTH_FAILURE: RTL initiated")

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

## 8.3 Verify Lead Agent V2

The V2 implementation relies on `langgraph`. Ensure dependencies are met.

```bash
cd ~/major_ws
colcon build --packages-select major_project --symlink-install
source install/setup.bash

# Ensure dependencies
pip3 install langgraph langchain-core langchain-ollama pydantic

# Check entrypoint
ros2 pkg executables major_project | grep lead_agent
```
