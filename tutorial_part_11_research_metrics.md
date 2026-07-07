# Part 11 — Research Metrics & Evaluation

> **Series position:** This is a supplementary research guide designed for evaluating the swarm architecture built in Parts 1–10. It outlines the precise, atomic metrics required to publish a research paper on LLM/SLM-driven multi-agent robotics.

---

## Table of Contents

1. [11.1 Quantitative Task Performance](#111-quantitative-task-performance)
2. [11.2 Safety and Coordination Metrics](#112-safety-and-coordination-metrics)
3. [11.3 SLM & Edge Computing Analysis](#113-slm--edge-computing-analysis)
4. [11.4 Data Collection (ROS 2 Bags)](#114-data-collection-ros-2-bags)

---

## 11.1 Quantitative Task Performance

When evaluating an SLM-driven swarm, you must prove that the language model can actually complete robotic missions reliably. 

### 1. Mission Success Rate (MSR)
The percentage of missions completed successfully out of total attempts ($N \ge 30$ recommended).
- **Strict Pass Condition:** The SLM successfully delegates the task, both drones reach their designated spatial coordinates within a tolerance of $\pm0.5m$, and both report "mission complete" via their Intent Bridges.
- **Fail Condition:** A drone stalls, flies to an incorrect coordinate, or fails to interpret the Speech-to-Text (STT) goal.

### 2. End-to-End Execution Time ($T_{e2e}$)
The wall-clock time from the moment the STT command is published to the moment the drones achieve the goal state. 
- **Equation:** $T_{e2e} = T_{flight} + T_{inference} + T_{network}$
- **Significance:** Demonstrates the viability of edge SLMs compared to cloud-based LLMs (like GPT-4), which suffer from high, unpredictable network latency ($T_{network}$).

---

## 11.2 Safety and Coordination Metrics

Swarms are inherently dangerous if not properly constrained. Your paper must quantify the effectiveness of your hybrid deterministic-probabilistic architecture.

### 1. Minimum Separation Distance ($D_{min}$)
The absolute closest physical distance between Drone-0 and Drone-1 during a mission.
- **Metric:** Log the Euclidean distance $\sqrt{(x_1 - x_0)^2 + (y_1 - y_0)^2 + (z_1 - z_0)^2}$ continuously.
- **Pass Condition:** $D_{min}$ must never drop below the `min_separation_m` threshold (e.g., $5.0m$).

### 2. Hardware Fallback Trigger Rate
How often the deterministic `safety_monitor` overrides the SLM's decisions. 
- **Metric:** Count of `RTL` (Return-to-Launch) commands injected directly by the Safety Monitor due to battery limits or geofence breaches. 
- **Significance:** Proves that the system remains physically safe even if the SLM outputs malicious or impossible JSON commands.

### 3. Communication Efficiency
The total number of envelope messages exchanged on `/agent/lead_to_wingman` and `/agent/wingman_to_lead` to achieve consensus.
- **Significance:** Evaluates the token economy of the swarm. Fewer messages mean less edge processing power consumed.

---

## 11.3 SLM & Edge Computing Analysis

Because you are running Qwen3.5 2B locally, you must provide benchmarks on the edge computing load.

### 1. Token Inference Latency
The average time required for the `qwen3.5:2b` model to generate an action.
- **Metric:** Tokens generated per second (TPS). Tracked via Ollama's API response metrics (`eval_duration` / `eval_count`).

### 2. Hallucination Rate
The frequency at which the SLM generates outputs that violate the schema or physical reality.
- **Format Hallucination:** SLM outputs invalid JSON or forgets the required `thought` and `action` keys.
- **Physical Hallucination:** SLM commands `fly(0, -500)` (outside geofence) or attempts to delegate a command to a non-existent drone.
- **Significance:** Shows the necessity of the Explicit Chain of Thought (ECoT) prompting and the JSON-enforced guardrails you implemented.

### 3. SLM Health Strike Rate
How often the Wingman's 5-strike consecutive failure loop is triggered.
- **Metric:** Time spent in a `DEGRADED` health state versus `OK` state in the `/agent/health` diagnostic topic.

---

## 11.4 Data Collection (ROS 2 Bags)

To write your paper, you cannot rely on terminal printouts. You must mathematically log the system using `ros2 bag`.

### The Atomic Record Command
Run this command in a background terminal before initiating any test flights. It records all vital telemetry, SLM outputs, health statuses, and visual detections required to compute the metrics above.

```bash
ros2 bag record \
  /drone_0/situation \
  /drone_1/situation \
  /camera_0/detections \
  /camera_1/detections \
  /agent/health \
  /agent/lead_to_wingman \
  /agent/wingman_to_lead \
  /fmu/out/vehicle_odometry \
  /px4_1/fmu/out/vehicle_odometry
```

### Analyzing the Bag
After the mission, use ROS 2 bag tools or Python's `rosbags` library to extract the SQLite database.
1. Extract `/fmu/out/vehicle_odometry` to compute **Minimum Separation Distance ($D_{min}$)** and prove zero collisions.
2. Extract `/agent/lead_to_wingman` to count **Communication Efficiency**.
3. Extract `/drone_0/situation` to measure **End-to-End Execution Time** by comparing the timestamp of the goal received versus the final pose.
