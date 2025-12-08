# Runtime Enforcement of Safe Lane-Change Behavior in Autonomous Vehicles

This repository contains our Reading Elective project focused on runtime enforcement of safe lane-change maneuvers in Autonomous Vehicles (AVs). The project uses Runtime Verification and DFA-based Automata Learning to ensure safety in high-risk driving situations.

## Project Overview

Autonomous vehicles may produce unsafe decisions due to perception errors or unpredictable traffic behavior. This project introduces a mechanism that:
- Learns a safety model (DFA) from correct lane-change traces
- Monitors event sequences at runtime
- Blocks or corrects unsafe behavior

This semester demonstrates a forced two-vehicle lane-change/overtake scenario in CARLA.

## Tools and Frameworks

CARLA Simulator: driving environment
Python CARLA API: route planning and control
Custom scripts: lane-change control experiments
AALpy: automata learning for DFA inference


## Running the Simulation

Prerequisites:
- Python 3.8+
- CARLA 0.9.x installed

Installation:
pip install -r requirements.txt

Start CARLA in a separate terminal:
./CarlaUE4.sh --quality-level=Epic --world-port=2000

Run scripts:
python scripts/generate_traffic.py
python scripts/waypoint_navigation.py
python scripts/forced_overtake.py


## Future Work

- Real-time enforcement in the control loop
- Expanded DFA models for more behaviors
- Performance evaluation and safety metrics
- Multi-vehicle traffic scenarios


## Authors

Nirbhay Sharma  
Harsh Gupta  

Supervisor: Saumya Shankar

## Contributions

Academic project under development.
Feedback and pull requests welcome.
