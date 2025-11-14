# Roboplow — SYSC 4805 Autonomous Snowplow Project

# Group L2 — SYSC 4805 Computer Systems Design Lab, Carleton University

# Team Members 
1. Ryan Page (101268082) RyanPage@cmail.carleton.ca
2. Neeraj Suchindrum
3. Neel Patel
4. Adeyehun Folahanmi

# Project overview
Roboplow is an autonomous snowplow robot designed to clear artificial snow blocks from a defined arena while avoiding stationary and moving obstacles. The system uses multiple sensors, a modular control architecture, and real-time decision making to navigate safely and efficiently.

# Repository structure
- /Deliverables — Proposal, progress reports, and supporting documents  
- /Roboplow — Source code for all robot modules  
    - /testing - Unit testing of submodules witj validation scripts

# Key features
- Autonomous navigation using IMU, line sensors, and distance sensors  
- Obstacle detection and avoidance (static and dynamic)  
- Plow mechanism for clearing 20 mm wooden cubes  
- Modular software architecture with subsystem handlers  
- Real-time Due-Nano communication over UART  
- Integrated watchdog timer for safety and recovery

# Getting started
Basic setup, build, and test instructions will be added as development progresses. Typical tasks include:
1. Install required toolchains for the target microcontroller and host environment.  
2. Build firmware from /src.  
3. Run unit tests in /tests.  
4. Deploy firmware to development hardware for integration testing.

# Contributions
Contributions and issues are welcome. Please open an issue or pull request with proposed changes and testing details.

# License
Specify your project license here (e.g., MIT, Apache 2.0). Update as needed.
