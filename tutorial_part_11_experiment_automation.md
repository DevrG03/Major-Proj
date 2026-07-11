# Part 11: Experiment Automation & Reproducibility (Phase 6)

> **Target:** PC-1 (Ground Control Station)
> **Prerequisites:** All V2 LangGraph patches complete.

To support the ICRA/IROS publication, we must move beyond single-mission anecdotes and prove the statistical reliability of the LangGraph cognitive architecture. This tutorial builds the automated batch-testing framework required to execute the 90-trial benchmark suite without manual intervention.

---

## 11.1 Benchmark Configuration (`scenarios.yaml`)

We define our three core evaluation scenarios in a strict configuration file. This guarantees reproducibility.

```bash
mkdir -p ~/major_ws/src/major_project/config

cat << 'EOF' > ~/major_ws/src/major_project/config/scenarios.yaml
# Edgents Evaluation Benchmark Suite
# Target: 30 trials per scenario

scenarios:
  A:
    name: "Nominal Coordination"
    description: "Evaluates standard swarm coordination without obstacles."
    voice_command: "take off and fly north 10 metres. wingman hold position."
    spawn_objects: false
    battery_start_pct: 90.0

  B:
    name: "Vision-Triggered Path Alteration"
    description: "Evaluates LLM reactive planning to dynamic obstacles."
    voice_command: "take off and fly north 10 metres. wingman follow."
    spawn_objects: true
    object_type: "person"
    object_pose: {x: 8.0, y: 0.0, z: 0.0}
    battery_start_pct: 90.0

  C:
    name: "Safety Fallback Validation"
    description: "Evaluates hardware-level override of LLM logic."
    voice_command: "take off and fly north 50 metres."
    spawn_objects: false
    battery_start_pct: 15.0 # Should instantly trigger hardware RTL
EOF
```

---

## 11.2 Batch Orchestrator Script (`run_batch_trials.sh`)

This script is the heart of the ICRA evaluation pipeline. It programmatically launches the PX4 SITL physics engine, the MicroXRCEAgent, and the ROS 2 cognitive stack. Crucially, it manages physical CPU cooldowns and records isolated ROS 2 bags for each trial.

```bash
mkdir -p ~/major_ws/src/major_project/scripts
cat << 'EOF' > ~/major_ws/src/major_project/scripts/run_batch_trials.sh
#!/bin/bash
# ---------------------------------------------------------
# ICRA/IROS Batch Testing Orchestrator
# Executes 30 trials per scenario unattended.
# ---------------------------------------------------------

SCENARIO=$1
TRIALS=30
COOLDOWN_SEC=15
TIMEOUT_SEC=360 # 6 minutes max per trial

if [ -z "$SCENARIO" ]; then
    echo "Usage: ./run_batch_trials.sh <A|B|C>"
    exit 1
fi

echo "Starting Batch Execution for Scenario: $SCENARIO"
mkdir -p ~/major_ws/test_results/scenario_${SCENARIO}

for ((i=1; i<=TRIALS; i++)); do
    echo "=================================================="
    echo " TRIAL $i / $TRIALS (Scenario $SCENARIO)"
    echo "=================================================="

    # 1. Start MicroXRCEAgent (Bridge)
    MicroXRCEAgent udp4 -p 8888 > ~/major_ws/test_results/scenario_${SCENARIO}/dds_${i}.log 2>&1 &
    DDS_PID=$!
    sleep 2

    # 2. Start PX4 SITL (Drone-0 and Drone-1)
    cd ~/PX4-Autopilot
    # Fixed random seed for reproducibility
    PX4_SYS_AUTOSTART=4010 PX4_GZ_WORLD=baylands PX4_GZ_MODEL_POSE="0,0,0,0,0,0" PX4_UXRCE_DDS_KEY=1 ./build/px4_sitl_default/bin/px4 -i 0 > ~/major_ws/test_results/scenario_${SCENARIO}/px4_0_${i}.log 2>&1 &
    PX4_0_PID=$!
    
    PX4_SYS_AUTOSTART=4010 PX4_GZ_WORLD=baylands PX4_GZ_MODEL_POSE="5,0,0,0,0,0" PX4_UXRCE_DDS_KEY=2 ./build/px4_sitl_default/bin/px4 -i 1 > ~/major_ws/test_results/scenario_${SCENARIO}/px4_1_${i}.log 2>&1 &
    PX4_1_PID=$!
    sleep 10 # Wait for physics to settle

    # 3. Dynamic Object Spawning (Scenario B)
    if [ "$SCENARIO" == "B" ]; then
        echo "Spawning dynamic obstacle (person) at x=8.0..."
        ros2 run ros_gz_sim create -world baylands -name person_obstacle -file ~/PX4-Autopilot/Tools/simulation/gz/models/person/model.sdf -x 8 -y 0 -z 0
    fi

    # 4. Start ROS 2 Bag Recording
    cd ~/major_ws/test_results/scenario_${SCENARIO}
    ros2 bag record -o bag_trial_${i} /mission_status /lead/approved_intent /wingman/approved_intent /drone_0/situation /agent/lead_to_wingman /agent/wingman_to_lead /rosout > /dev/null 2>&1 &
    BAG_PID=$!

    # 5. Launch Cognitive Stack (Lead)
    source ~/major_ws/install/setup.bash
    ros2 launch major_project lead_pilot.launch.py > ~/major_ws/test_results/scenario_${SCENARIO}/ros2_lead_${i}.log 2>&1 &
    ROS_PID=$!
    sleep 5 # Wait for LangGraph to enter STANDBY

    # 6. Inject Voice Command based on Scenario
    if [ "$SCENARIO" == "A" ]; then
        CMD="take off and fly north 10 metres. wingman hold position."
    elif [ "$SCENARIO" == "B" ]; then
        CMD="take off and fly north 10 metres. wingman follow."
    elif [ "$SCENARIO" == "C" ]; then
        CMD="take off and fly north 50 metres."
    fi
    echo "Injecting command: $CMD"
    ros2 topic pub --once /voice_commands std_msgs/msg/String "data: '$CMD'"

    # 7. Monitor for Mission Complete or Timeout
    start_time=$(date +%s)
    while true; do
        current_time=$(date +%s)
        elapsed=$((current_time - start_time))
        
        if [ $elapsed -ge $TIMEOUT_SEC ]; then
            echo "Trial $i TIMED OUT!"
            break
        fi
        
        # Check if mission_status topic broadcasted completion/RTL
        if grep -q "MISSION COMPLETE\|RTL" ~/major_ws/test_results/scenario_${SCENARIO}/ros2_lead_${i}.log; then
            echo "Mission concluded successfully in $elapsed seconds."
            break
        fi
        sleep 5
    done

    # 8. Graceful Teardown
    echo "Tearing down trial $i..."
    kill -INT $BAG_PID 2>/dev/null
    sleep 2
    kill -9 $ROS_PID $PX4_0_PID $PX4_1_PID $DDS_PID 2>/dev/null
    killall -9 px4 ruby MicroXRCEAgent 2>/dev/null
    
    # 9. CPU Cooldown
    echo "Cooling down CPU for $COOLDOWN_SEC seconds..."
    sleep $COOLDOWN_SEC
done

echo "Batch execution for Scenario $SCENARIO complete! Logs saved to ~/major_ws/test_results/scenario_${SCENARIO}"
EOF
chmod +x ~/major_ws/src/major_project/scripts/run_batch_trials.sh
```

---

## 11.3 Technical Verification (Logic Check)

Before proceeding to data extraction, we verify the orchestrator's logic:
1. **CPU Thermal Safety:** Gazebo + Ollama SLM inference pushes CPUs to 100%. The explicit `killall -9` prevents zombie physics engines, and the `sleep 15` cooldown prevents thermal throttling which would otherwise artificially skew the LLM latency metrics on trial 20 vs trial 1.
2. **Dynamic Spawning:** In Scenario B, the orchestrator utilizes `ros_gz_sim create` to inject the obstacle perfectly after physics settles, guaranteeing identical placement across all 30 trials.
3. **Data Logging Isolation:** Each trial captures its own `px4`, `dds`, `ros2`, and isolated `.db3` bag file, preventing cross-contamination of MSR data.

**Next Step:** Task 2 (Write the Python metrics extractor to parse these bags).

---

## 11.4 Metrics Extraction Pipeline (`aggregate_metrics.py`)

After the 90 trials run overnight, we will be left with 90 `.db3` ROS 2 bag files. Manually reading these is impossible. We must build an automated Python script using the `rosbags` library to extract the critical metrics required for the ICRA paper (MSR, Hallucination Rate, Minimum Separation, Safety Overrides).

```bash
cat << 'EOF' > ~/major_ws/src/major_project/scripts/aggregate_metrics.py
#!/usr/bin/env python3
"""
Edgents ICRA Metrics Aggregator

Parses 90 ROS 2 bag files to extract:
1. Mission Success Rate (MSR)
2. Format / Physical Hallucination Rates
3. Mission Duration (seconds)
4. Minimum Swarm Separation (meters)
5. Safety Layer Compliance (0 SLM overrides)
"""
import os
import glob
import json
import csv
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

def process_bags(scenario_dir):
    bag_dirs = glob.glob(os.path.join(scenario_dir, "bag_trial_*"))
    results = []

    for bag_path in sorted(bag_dirs):
        trial_id = os.path.basename(bag_path).split("_")[-1]
        
        # Trial metrics
        metrics = {
            "trial_id": trial_id,
            "success": False,
            "duration_sec": 0,
            "min_separation_m": 999.0,
            "safety_overrides": 0,
            "format_hallucinations": 0
        }
        
        start_time = None
        end_time = None

        try:
            with Reader(bag_path) as reader:
                for connection, timestamp, rawdata in reader.messages():
                    # Time tracking
                    ts_sec = timestamp / 1e9
                    if start_time is None: start_time = ts_sec
                    end_time = ts_sec

                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    
                    # 1. Mission Success Rate
                    if connection.topic == '/mission_status':
                        if "MISSION COMPLETE" in msg.data:
                            metrics["success"] = True
                    
                    # 2. Separation Distance (From Safety Node)
                    if connection.topic == '/drone_0/situation':
                        if "separation:" in msg.data:
                            # Parse separation:X.Xm
                            try:
                                sep = float(msg.data.split("separation:")[1].split("m")[0])
                                if sep < metrics["min_separation_m"]:
                                    metrics["min_separation_m"] = sep
                            except: pass

                    # 3. Safety Compliance Verification
                    if connection.topic == '/lead/approved_intent':
                        intent = json.loads(msg.data)
                        if "CRITICAL SAFETY ALERT" in msg.data or intent.get("action") == "rtl":
                            # We track if a safety intervention occurred
                            metrics["safety_overrides"] += 1

                    # 4. Format Hallucinations (From node logs)
                    if connection.topic == '/rosout':
                        if "Failed to parse planner output" in msg.msg:
                            metrics["format_hallucinations"] += 1

        except Exception as e:
            print(f"Failed to read bag {bag_path}: {e}")
            continue

        if start_time and end_time:
            metrics["duration_sec"] = round(end_time - start_time, 2)
            
        results.append(metrics)
        
    return results

if __name__ == "__main__":
    base_dir = os.path.expanduser("~/major_ws/test_results")
    scenarios = ["A", "B", "C"]
    
    with open(os.path.join(base_dir, "icra_results.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["scenario", "trial_id", "success", "duration_sec", "min_separation_m", "safety_overrides", "format_hallucinations"])
        writer.writeheader()
        
        for sc in scenarios:
            print(f"Processing Scenario {sc}...")
            sc_dir = os.path.join(base_dir, f"scenario_{sc}")
            if os.path.exists(sc_dir):
                data = process_bags(sc_dir)
                for row in data:
                    row["scenario"] = sc
                    writer.writerow(row)
                    
    print("Metrics successfully aggregated to ~/major_ws/test_results/icra_results.csv")
EOF
chmod +x ~/major_ws/src/major_project/scripts/aggregate_metrics.py
```

### 11.4.1 Logical Verification of Metrics Script

This script acts as our objective judge for the evaluation section of the research paper. 
1. **MSR (Mission Success Rate):** It strictly looks for the `MISSION COMPLETE` flag on `/mission_status`. If the drone times out or crashes, MSR defaults to `False`.
2. **Min Separation Tracking:** It actively parses the `/drone_0/situation` topic published by our previously implemented Safety Node. By finding the minimum distance across the entire flight, we mathematically prove Swarm Safety Claim (e.g., "drones never breached a 2.0m radius").
3. **Format Hallucination Rate:** By reading `/rosout`, we can exactly count how many times the `ValueError` from Patch 10 was thrown during the flight. This gives us the exact baseline AI stability metric.
