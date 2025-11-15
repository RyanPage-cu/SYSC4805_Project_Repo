# SYSC 4805 Computer Systems Design Lab, Carleton University

# Group L2 - Roboplow - SYSC 4805 Autonomous Snowplow Project

# Team Members 
1. Ryan Page (101268082) RyanPage@cmail.carleton.ca
2. Neeraj Suchindrum (101199014)
3. Neel Patel (101226947)
4. Adeyehun Folahanmi (101237546)

# Project overview
Roboplow is an autonomous snowplow robot designed to clear artificial snow blocks from a defined arena while avoiding stationary and moving obstacles. The system uses multiple sensors, a modular control architecture, and real-time decision making to navigate safely and efficiently.

# Repository structure
- /deliverables - Proposal, progress reports, and supporting documents  
- /roboplow - Source code for all robot modules  
    - /testing - Unit testing of submodules witj validation scripts

# Key features
- Autonomous navigation using IMU, line sensors, and distance sensors  
- Obstacle detection and avoidance (static and dynamic)  
- Plow mechanism for clearing 20 mm wooden cubes  
- Modular software architecture with subsystem handlers  
- Real-time Due-Nano communication over UART  
- Integrated watchdog timer for safety and recovery

# Getting started
Arduino IDE does most of the heavy lifting for setting up the project. 

1. Clone this repo
```
git clone https://github.com/RyanPage-cu/SYSC4805_Project_Repo
```

2. Open Arduino IDE and navigate to File > Open

3. Navigate to the path to the repo/roboplow and select any sketch (`.ino`) found in any of the directory.

The main `.ino` file is `roboplow.ino` which contains all the logic for the system. There are also unit tests in the testing directory for each of the main sensors and the motors.

