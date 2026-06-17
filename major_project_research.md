# Deep Research Report: Multi-Drone SLM Pilot System
**Date:** 2026-06-03
**Scope:** Architecture decisions for extending minor project (single-drone confidence-gated SLM) to a 2-drone system with rank-based SLM pilots communicating in natural language over WiFi.

**Research stats:** 110 agents · 27 sources · 128 claims extracted · 25 adversarially verified · 8 confirmed · 17 killed

---

## 1. Simulation Environment — Verdict: Gazebo Garden

**Confirmed (3-0 unanimous):**
- Multi-vehicle PX4 SITL is **Linux-only** — rules out Windows entirely
- **Isaac Sim** requires NVIDIA RTX 3080Ti (16 GB GPU) — impractical for this setup
- **AirSim** is effectively abandoned post-Microsoft acquisition
- **Gazebo Garden** is the correct and only viable choice given hardware constraints

**Source:** PX4 official docs verbatim: *"Multi-Vehicle Simulation with Gazebo is only supported on Linux."*
Reference: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation

### XRCE-DDS Multi-Vehicle (confirmed 3-0)
- A **single XRCE-DDS agent** serves both PX4 SITL instances simultaneously over UDP
- Topic namespacing is automatic:
  - Drone 0 → default namespace (`/fmu/...`)
  - Drone 1 → `/px4_1/fmu/...`
- Each instance needs a unique non-zero `UXRCE_DDS_KEY`
- Run the DDS agent on PC-1; both PX4 instances connect via UDP over WiFi

**Source:** https://docs.px4.io/main/en/ros2/multi_vehicle

---

## 2. Critical Finding: Free-Form NL Inter-Agent Communication Does NOT Work

**This is the most important finding from the research.**

Multiple claims about decentralized natural language coordination between SLM agents were **killed 0-3 by adversarial verification:**
- Dynamic role assignment/negotiation via NL → refuted (0-3)
- Spontaneous NL coordination between LLM agents → refuted (0-3)
- Free-form NL as viable inter-agent protocol → refuted (0-3)

**What the literature actually shows** (IEEE WoWMoM 2026, SwarmBench May 2025):
- Six frontier LLMs including Qwen 3 8B and Claude Haiku v4.5 *"still struggle to achieve reliable execution — even for simple swarm tasks — when operating without explicit grounding and execution support"*
- Most models score zero on complex swarm tasks in zero-shot decentralized coordination
- **All reported success cases rely on scaffolding, tool-calling, or structured execution support**

**Implication for this project:** The "radio chatter" vision (pilots talking in NL) is achievable as a **display/log layer** — readable NL summaries generated *after* a structured command is validated. The actual control wire between agents must always be schema-validated JSON (Pydantic v2 already in stack).

**Sources:**
- arXiv:2605.03788 — "Say the Mission, Execute the Swarm" (IEEE WoWMoM 2026)
- arXiv:2505.04364 — SwarmBench (May 2025)

---

## 3. Recommended Architecture: Hierarchical (confirmed 3-0)

**Confirmed:** Hierarchical multi-agent LLM frameworks achieve state-of-the-art on multi-robot benchmarks:
- 0.95 success rate on compound tasks
- 0.84 on complex tasks
- 0.60 on vague/underspecified tasks
- +2/+7/+15 pp over prior SOTA (LaMMA-P, ICRA 2025)

**Architecture:** Upper-layer SLM decomposes tasks → lower-layer SLMs execute structured plans.
This maps directly to: **Lead Pilot (mission planner) → Wingman (executor)**.

**Caveat:** These results used GPT-4o. Performance with Qwen2.5-Coder:3b at 3B parameters will be lower — the 500M threshold from the minor project may need revisiting at multi-agent scale.

**Source:** arXiv:2602.21670 — "Hierarchical LLM-Based Multi-Agent Framework with Prompt Optimization for Multi-Robot Task Planning" (Feb 2026)

---

## 4. Ollama Over WiFi — Validated (confirmed 3-0)

A ROS2 architecture offloading LLM inference (via Ollama) to a remote networked workstation is validated in simulation and on a physical PX4 quadcopter (arXiv:2506.07509, June 2025):
- PX4 + ROS2 + Ollama-hosted LLMs
- *"A separate Ubuntu 22.04 workstation used to host the Ollama models whereby it acts as a remote server through a standard local network connection"*
- Physical validation: custom quadcopter, 40% mission success rate

**Critical gap:** The paper omits end-to-end latency measurements. PX4 offboard mode requires >2 Hz setpoint updates. **This must be benchmarked experimentally — it is the #1 risk factor.**

**Source:** arXiv:2506.07509 — "Taking Flight with Dialogue" (June 2025)

---

## 5. Sensor Fusion Without LiDAR

Camera raw frames **cannot** go into a 3B SLM — the model has no vision capability. The validated pattern is **pre-processed situational awareness summaries** fed as text context:

| Sensor | Pre-processing | SLM Input Format |
|---|---|---|
| Camera | Object detection (YOLO/MobileNet) | `"obstacle at 4m bearing 045"` |
| GPS | Direct passthrough | `"pos: 28.6N 77.2E alt: 50m"` |
| IMU | EKF2 state estimation (PX4 built-in) | `"speed: 3.2m/s heading: 090"` |
| Barometer | Direct passthrough | `"alt_baro: 48.3m"` |

A `sensor_aggregator_node` per drone collects these, formats them into a structured text block, and injects into the SLM prompt each inference cycle.

---

## 6. ROS2 Multi-Machine WiFi Setup

Confirmed working configuration:
- Identical `ROS_DOMAIN_ID` (e.g., `42`) on both PCs
- **CycloneDDS** preferred over FastDDS for WiFi (better multicast reliability)
- XRCE-DDS agent on PC-1, both PX4 instances connect via UDP
- Inter-agent SLM messages: dedicated ROS2 topics

**Sources:**
- https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html
- arXiv:2508.11366v1

---

## 7. Refuted Claims (Do Not Design Around These)

| Claim | Verdict |
|---|---|
| Free-form NL is viable for agent-to-agent coordination | 0-3 killed |
| Centralized single-LLM can orchestrate multi-drone via MCP/WoT | 0-3 killed |
| Decentralized swarm LLMs achieve spontaneous NL coordination | 0-3 killed |
| Dynamic role negotiation (leader/follower) at runtime via NL | 0-3 killed |
| Qwen2.5-3B achieves 100% valid command generation | 0-3 killed |
| SmolVLM on RPi4 for onboard vision inference | 0-3 killed |
| GGUF VLA model on RPi4 @ ~11s inference | 0-3 killed |
| Isaac Sim / AirSim viable for this hardware | Not viable |

---

## 8. Open Questions (Resolve Experimentally)

1. **Latency:** What is the round-trip latency of Qwen2.5-Coder:3b via Ollama over WiFi in a ROS2 publish/subscribe loop? Is it compatible with PX4 offboard >2 Hz? — **#1 risk**
2. **3B parameter floor:** Is Qwen2.5-Coder:3b sufficient for multi-agent structured command generation, or is 7B the practical floor?
3. **CycloneDDS vs FastDDS:** Which performs better for 2-PC WiFi distributed Gazebo+PX4 SITL?
4. **Minimum agent-to-agent message schema:** What is the smallest Pydantic schema to convey situational awareness for coordinated maneuvers?

---

## 9. Novel Contribution Opportunities

These gaps in literature are original contribution opportunities for the major project:

1. **Minimum inter-agent message schema for sub-1B SLM drone coordination** — not defined anywhere in literature
2. **Confidence-gated hierarchical command propagation** — extending the minor project's gate to the lead→wingman channel (does wingman ask lead for clarification the way lead asks human?)
3. **Empirical latency benchmark** of Qwen2.5-Coder:3b in a ROS2 WiFi control loop — explicitly noted as a gap in arXiv:2506.07509
4. **Rank-based SLM pilot system** — no paper implements Air Force-style rank hierarchy with SLMs; closest is hierarchical planners using GPT-4o

---

## 10. Decision Summary

| Decision | Choice | Reason |
|---|---|---|
| Simulator | Gazebo Garden | Only viable; already in stack |
| Architecture | Hierarchical (Lead + Wingman) | Only pattern with confirmed multi-robot success |
| Inter-agent protocol | Schema-validated JSON (Pydantic v2) | Free-form NL coordination killed 0-3 |
| NL "radio chatter" | Display/log layer only | Not the control wire |
| Inference hosting | Ollama on each PC | Validated in arXiv:2506.07509 |
| DDS | CycloneDDS | Better WiFi multicast |
| Camera perception | Pre-processed text summaries | 3B SLM has no vision |
| Sensor pipeline | EKF2 → text context → SLM prompt | Validated pattern |
| Confidence gating | Both lead and wingman have gates | Extension of minor project |

---

*Report generated from deep research workflow. 110 agents, 27 primary sources, adversarial verification at 2-of-3 threshold.*
