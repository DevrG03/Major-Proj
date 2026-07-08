# Diagram 1 — Pilot Agent Architecture

> Focuses on the internal intelligence pipeline of one agent (Lead shown; Wingman is symmetric).

```mermaid
flowchart TD
    subgraph INPUT["📥 Inputs"]
        VC["/voice_commands\nSTT Goal String"]
        SIT["/drone_0/situation\n1 Hz situation string\nbat · alt · pos · gps · camera · temporal"]
        CAM["/camera_0/detections\n2 Hz YOLOv8 detections"]
        WTL["/agent/wingman_to_lead\nJSON status / query / ack"]
        SEV["/safety/event\nJSON SafetyEvent"]
    end

    subgraph AGENT["🧠 lead_agent_node  —  Dual-Thread Model"]
        direction TB
        ROS["🔄 ROS 2 Thread\nrclpy.spin()\nHandles all topic callbacks"]
        LOCK["🔐 threading.Lock\nShared state protection"]
        LOOP["🧵 Daemon Thread\n_agent_loop()\nInfinite think-act loop"]

        subgraph CTX["context_manager.py — Prompt Builder"]
            GOAL["[MISSION GOAL]\n~20 tokens"]
            CURR["[CURRENT SITUATION]\n~80 tokens"]
            MEMO["[MEMORY]\nrecall() results ~100 tokens"]
            MSGS["[MESSAGES FROM OTHER AGENT]\n~50 tokens"]
            ACTS["[RECENT ACTIONS]\nSliding window ≤8 entries ~400 tokens"]
            ANCHOR["[NEXT ACTION]\nOutput one tool call JSON:"]
        end

        subgraph INFER["Inference Pipeline"]
            BUILD["build_prompt()\n~960 tokens total"]
            OLLAMA["Ollama HTTP\nPOST /api/generate\nqwen3.5:2b  num_ctx=8192"]
            ECOT["ECoT Output\n{thought: '...', tool: '...', params: {...}}"]
            RETRY["3-Attempt Retry Loop\nAppend error_ctx on fail\nSkip cycle if all fail"]
        end

        subgraph TOOLS["LeadToolRegistry — 19 Tools"]
            direction LR
            FLIGHT["✈️ Flight\ntakeoff · move · hover\nsearch · land · rtl"]
            SENSE["👁️ Sensing\nget_situation\nscan_camera\nget_battery"]
            MEM2["🗄️ Memory\nremember · recall\nwait · mission_complete"]
            COMM["📡 Communication\nmessage_wingman\nask_human · notify_human"]
        end

        subgraph MEMORY["agent_memory.py"]
            SQLDB["SQLite\nlead_agent_memory.db\n~/.ros/"]
        end
    end

    subgraph OUTPUT["📤 Outputs"]
        IAP["/lead/approved_intent\nFlightIntent JSON"]
        LTW["/agent/lead_to_wingman\nNatural language order"]
        CR["/clarification_request\nTTS question to operator"]
        MS["/mission_status\nJSON {lead, wingman}"]
    end

    %% Input → Agent
    VC -->|"_on_voice()\n_assign_goal() or _human_event.set()"| ROS
    SIT -->|"_on_situation()\nupdate own_situation"| ROS
    CAM -->|"_on_camera()\nupdate camera_summary"| ROS
    WTL -->|"_on_wingman_message()\ninject into context"| ROS
    SEV -->|"_on_safety_event()\ninject into context"| ROS

    ROS <-->|shared state| LOCK
    LOCK <-->|shared state| LOOP

    %% Agent loop internals
    LOOP --> BUILD
    GOAL & CURR & MEMO & MSGS & ACTS & ANCHOR --> BUILD
    BUILD --> OLLAMA
    OLLAMA --> ECOT
    ECOT --> RETRY
    RETRY -->|valid tool call| TOOLS
    RETRY -->|ctx.add_tool_result| ACTS

    TOOLS --> FLIGHT
    TOOLS --> SENSE
    TOOLS --> MEM2
    TOOLS --> COMM

    MEM2 <--> SQLDB

    %% Outputs
    FLIGHT -->|FlightIntent JSON| IAP
    COMM -->|String| LTW
    COMM -->|String| CR
    LOOP -->|after each tool| MS

    classDef clean fill:#ffffff,color:#000000,stroke:#000000,stroke-width:1.5px

    class VC,SIT,CAM,WTL,SEV clean
    class IAP,LTW,CR,MS clean
    class ROS,LOCK,LOOP,BUILD,ECOT,RETRY,GOAL,CURR,MEMO,MSGS,ACTS,ANCHOR clean
    class OLLAMA clean
    class FLIGHT,SENSE,MEM2,COMM clean
    class SQLDB clean
```

---

# Diagram 2 — Complete System Architecture

> PC-to-PC deployment topology: PX4, Gazebo, ROS 2, DDS bridges, and both agent stacks.

```mermaid
flowchart TB
    subgraph OPERATOR["👤 Human Operator"]
        MIC["🎙️ Microphone\nVoice Goal"]
        SPK["🔊 Speaker\nTTS Clarifications"]
        TERM["🖥️ Terminal\nMission Status"]
    end

    subgraph PC1["💻 PC-1 — Lead  ·  10.34.211.86  ·  Ubuntu 26.04 LTS"]
        direction TB

        subgraph SIM1["Gazebo Harmonic — x500_mono_cam (Drone-0)"]
            PX4_0["PX4 SITL\nDrone-0\nns /fmu/  key:1"]
            CAM0["Camera Plugin\nGazebo gz.msgs.Image"]
        end

        GZB["ros_gz_bridge\n/world/.../imager/image\n→ /camera/image_raw"]
        XRCE0["MicroXRCE-DDS Agent\nudp4 -p 8888"]

        subgraph LEAD_STACK["Lead ROS 2 Stack  (lead_pilot.launch.py — 10 nodes)"]
            STT["stt_node"]
            LSA["lead_sensor_aggregator\n→ /drone_0/situation  1Hz"]
            YOLO["camera_detection_node\nYOLOv8-nano  2Hz\n→ /camera_0/detections"]
            SM["safety_monitor_node\n⚡ bat≤15% / gps<3 → RTL\nboth drones"]
            LAGENT["lead_agent_node\nqwen3.5:2b via Ollama"]
            BRIDGE["lead_intent_bridge_node"]
            LCMD["lead_px4_commander\n10Hz OFFBOARD\n/fmu/in/*"]
            SPEAK["clarification_speaker_node"]
            MMON["mission_monitor_node"]
            ESTOP["emergency_stop_node"]
        end
    end

    WIFI["🛜 CycloneDDS over WiFi\nROS_DOMAIN_ID=42\n/agent/lead_to_wingman\n/agent/wingman_to_lead\n/safety/event"]

    subgraph PC2["💻 PC-2 — Wingman  ·  10.34.211.15  ·  Ubuntu 26.04 LTS"]
        direction TB

        subgraph SIM2["Gazebo Harmonic — x500_mono_cam (Drone-1)"]
            PX4_1["PX4 SITL\nDrone-1\nns /px4_1/fmu/  key:2"]
            CAM1["Camera Plugin\nGazebo gz.msgs.Image"]
        end

        GZB1["ros_gz_bridge\n/world/.../imager/image\n→ /px4_1/camera/image_raw"]
        XRCE1["MicroXRCE-DDS Agent\nudp4 -p 8888"]

        subgraph WING_STACK["Wingman ROS 2 Stack  (wingman_pilot.launch.py — 3 nodes)"]
            WSA["wingman_sensor_aggregator\n→ /drone_1/situation  1Hz"]
            WAGENT["wingman_agent_node\nqwen3.5:2b via Ollama"]
            WCMD["wingman_px4_commander\n10Hz OFFBOARD\n/px4_1/fmu/in/*"]
        end
    end

    %% Operator ↔ GCS
    MIC -->|speech| STT
    SPEAK -->|pyttsx3| SPK
    MMON --> TERM

    %% PC-1 simulation → DDS bridge → ROS 2
    PX4_0 <-->|uORB| XRCE0
    CAM0 -->|gz.msgs.Image| GZB
    GZB -->|/camera/image_raw| YOLO
    XRCE0 -->|/fmu/out/...| LSA
    XRCE0 -->|/fmu/out/battery_status\n/fmu/out/vehicle_gps_position| SM

    %% PC-1 internal data flow
    STT -->|/voice_commands| LAGENT
    LSA -->|/drone_0/situation| LAGENT
    YOLO -->|/camera_0/detections| LAGENT
    YOLO -->|/camera_0/detections| LSA
    SM -->|/safety/event| LAGENT
    SM -->|/fmu/in/vehicle_command RTL| XRCE0
    LAGENT -->|/lead/approved_intent| BRIDGE
    BRIDGE -->|/lead/approved_intent| LCMD
    LCMD -->|10Hz OFFBOARD setpoints| XRCE0
    XRCE0 -->|TrajectorySetpoint\nVehicleCommand| PX4_0
    LAGENT -->|/clarification_request| SPEAK
    LAGENT -->|/mission_status| MMON
    ESTOP -->|/fmu/in/vehicle_command kill| XRCE0

    %% Cross-PC DDS WiFi
    LAGENT <-->|CycloneDDS WiFi| WIFI
    WIFI <-->|CycloneDDS WiFi| WAGENT
    SM -->|/safety/event| WIFI

    %% PC-2 simulation → DDS bridge → ROS 2
    PX4_1 <-->|uORB| XRCE1
    CAM1 -->|gz.msgs.Image| GZB1
    GZB1 -->|/px4_1/camera/image_raw| WSA
    XRCE1 -->|/px4_1/fmu/out/...| WSA
    XRCE1 -->|/px4_1/fmu/out/battery_status\n/px4_1/fmu/out/vehicle_gps_position| SM
    SM -->|/px4_1/fmu/in/vehicle_command RTL| XRCE1

    %% PC-2 internal data flow
    WSA -->|/drone_1/situation| WAGENT
    WAGENT -->|/wingman/approved_intent| WCMD
    WCMD -->|10Hz OFFBOARD setpoints| XRCE1
    XRCE1 -->|TrajectorySetpoint\nVehicleCommand| PX4_1
    ESTOP -->|/px4_1/fmu/in/vehicle_command kill| XRCE1

    classDef clean fill:#ffffff,color:#000000,stroke:#000000,stroke-width:1.5px

    class PX4_0,PX4_1,CAM0,CAM1 clean
    class GZB,GZB1,XRCE0,XRCE1 clean
    class LAGENT,WAGENT clean
    class SM clean
    class LCMD,WCMD,BRIDGE clean
    class STT,SPEAK,MMON,ESTOP clean
    class WIFI clean
    class LSA,WSA,YOLO clean
```
