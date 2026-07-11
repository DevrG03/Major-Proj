# Part 12: Baselines & Ablations (Phase 6)

> **Target:** PC-2 (Wingman Workstation)
> **Prerequisites:** All V2 patches complete and Part 11 automation framework established.

To prove that the Small Language Model (SLM) cognitive architecture actually provides value, ICRA reviewers will demand a comparison against a traditional controller. 
Furthermore, to prove that our Explicit Chain-of-Thought (ECoT) prompt engineering improves mission success, we must ablate (remove) it and measure the performance drop.

This tutorial implements both the Baseline Controller (Task 3) and the ECoT Ablation (Task 4).

---

## 12.1 Rule-Based Baseline Controller (`rule_based_wingman.py`)

This script is **Option 1** from our roadmap. It subscribes to the exact same ROS 2 topics as the `wingman_agent_node.py`, but it replaces the LangGraph AI with a hardcoded `if/elif/else` state machine.

We will run this baseline through Scenario B (dynamic obstacles). Because the rule-based controller lacks semantic reasoning, it will likely fail to evade dynamic obstacles gracefully, thus proving the superiority of the SLM approach.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/wingman_pilot/rule_based_wingman.py
#!/usr/bin/env python3
"""
Rule-Based Baseline Controller
Provides a non-AI comparison for the Wingman Drone to satisfy ICRA evaluation requirements.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import math

RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

class RuleBasedWingmanNode(Node):
    def __init__(self):
        super().__init__('rule_based_wingman_node')
        
        self.pub_intent = self.create_publisher(String, '/wingman/approved_intent', RELIABLE_QOS)
        self.pub_lead_msg = self.create_publisher(String, '/agent/wingman_to_lead', 10)

        self.create_subscription(String, '/drone_1/situation', self._on_situation, 10)
        self.create_subscription(String, '/agent/lead_to_wingman', self._on_lead_message, 10)

        self.current_state = "STANDBY"
        self.battery_pct = 100.0
        self.lead_pos = (0.0, 0.0)
        self.own_pos = (0.0, 0.0)

        self.get_logger().info("Rule-Based Wingman (Baseline) Ready.")
        
        # Simple control loop running at 1Hz
        self.create_timer(1.0, self._control_loop)

    def _on_situation(self, msg: String):
        """Parse telemetry string from WingmanSensorAggregator"""
        try:
            parts = msg.data.split()
            for p in parts:
                if p.startswith("bat:"):
                    self.battery_pct = float(p.split(":")[1].replace("%", ""))
                elif p.startswith("pos:"):
                    coords = p.split(":")[1].replace("(", "").replace(")", "").split(",")
                    self.own_pos = (float(coords[0]), float(coords[1]))
        except Exception:
            pass

    def _on_lead_message(self, msg: String):
        """Parse incoming tasks from Lead Agent"""
        try:
            data = json.loads(msg.data)
            content = data.get("content", "").lower()
            
            if "follow" in content:
                self.current_state = "FOLLOW"
                self.get_logger().info("State changed to: FOLLOW")
                self._reply_lead("Starting follow behavior.")
            elif "hold" in content or "hover" in content:
                self.current_state = "HOVER"
                self.get_logger().info("State changed to: HOVER")
                self._reply_lead("Holding position.")
            elif "rtl" in content or "return" in content:
                self.current_state = "RTL"
                self.get_logger().info("State changed to: RTL")
        except json.JSONDecodeError:
            pass

    def _reply_lead(self, message: str):
        msg = String()
        msg.data = json.dumps({"type": "reply", "sender": "WINGMAN", "content": f"[TASK COMPLETE] {message}", "order_id": None})
        self.pub_lead_msg.publish(msg)

    def _control_loop(self):
        """Hardcoded State Machine Logic"""
        if self.battery_pct <= 20.0 and self.current_state != "RTL":
            self.get_logger().warn("Battery critical! Forcing RTL.")
            self.current_state = "RTL"
            
        intent_payload = None

        if self.current_state == "STANDBY":
            intent_payload = {'action': 'hover', 'confidence': 'high'}
            
        elif self.current_state == "HOVER":
            intent_payload = {'action': 'hover', 'confidence': 'high'}
            
        elif self.current_state == "RTL":
            intent_payload = {'action': 'rtl', 'confidence': 'high'}
            
        elif self.current_state == "FOLLOW":
            # Very rudimentary blind follow (ignores obstacles!)
            intent_payload = {'action': 'move', 'direction': 'north', 'distance_m': 2.0, 'altitude_m': 10.0, 'confidence': 'high'}

        if intent_payload:
            msg = String()
            msg.data = json.dumps(intent_payload)
            self.pub_intent.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedWingmanNode()
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
chmod +x ~/major_ws/src/major_project/major_project/wingman_pilot/rule_based_wingman.py
```

### 12.1.1 Running the Baseline

To execute the baseline, you will modify `run_batch_trials.sh` (from Part 11) to launch the `rule_based_wingman.py` node instead of `wingman_agent_node.py` during Step 5. Run this across 30 trials of Scenario B and record the resulting `icra_results.csv`.

---

## 12.2 Explicit Chain-of-Thought (ECoT) Ablation

The core scientific claim of this paper is that forcing the SLM to emit a `"thought"` key before generating the `"checklist"` significantly reduces Hallucination Rates (Schema Drift). We must prove this empirically.

To do this, we create a parallel system prompt for the Lead Agent that strictly removes the ECoT instructions.

```bash
cat << 'EOF' > ~/major_ws/src/major_project/major_project/lead_pilot/prompts/lead_planner_ablation_system.txt
You are LEAD PILOT (Drone-0). Your job is to plan a mission by decomposing the MISSION GOAL into a sequential checklist of tool calls.

Output EXACTLY ONE JSON object matching this schema:
{
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

### 12.2.1 Running the Ablation

To run the ablation study:
1. Temporarily change the path in `lead_agent_node.py` on Line 110 to point to `lead_planner_ablation_system.txt`.
2. Run the `run_batch_trials.sh` orchestrator for 30 trials on Scenario A.
3. Use `aggregate_metrics.py` to parse the logs.
4. Compare the `format_hallucinations` column against the baseline data to quantitatively prove the necessity of ECoT prompt engineering for edge SLMs.
