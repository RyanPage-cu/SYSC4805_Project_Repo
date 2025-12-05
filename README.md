# 🧊 Roboplow – SYSC 4805 Autonomous Snowplow Project  
**Carleton University – Computer Systems Design Lab (SYSC 4805)**

## 👥 Group L2 – Team Members
| Name | Student Number | Email |
|------|----------------|-------|
| Ryan Page | 101268082 | RyanPage@cmail.carleton.ca |
| Neeraj Suchindrum | 101199014 |  |
| Neel Patel | 101226947 |  |
| Adeyehun Folahanmi | 101237546 |  |

---

# ❄️ Project Overview

**Roboplow** is an autonomous snowplow robot designed to navigate an indoor testing arena, clear artificial snow blocks, and avoid static or dynamic obstacles.  
Built on the **Arduino Due**, the system integrates multiple sensors, modular subsystem handlers, and a state-driven control algorithm.

This repository contains:

- All design deliverables  
- Complete embedded source code  
- Unit tests for all main hardware components  
- Architecture documentation and calibration data  

Roboplow was developed for **Carleton University’s SYSC 4805: Computer Systems Design Lab**.

---

# 📁 Repository Structure

```
SYSC4805_Project_Repo/
│
├── Deliverables/               
│   ├── Proposal.docx
│   ├── Progress_Report.docx
│   ├── System_Architecture/
│   └── ...
│
├── roboplow/                   
│   ├── include/                
│   │   └── sensor_manager.hpp
│   │
│   ├── testing/                
│   │   ├── test_motors.ino
│   │   ├── test_ultrasonic.ino
│   │   ├── test_lineSensors.ino
│   │   ├── test_tof.ino
│   │   └── ...
│   │
│   ├── roboplow.ino            
│   ├── motors.cpp
│   ├── ultrasonic_distance_sensor.cpp
│   ├── tof_distance_sensor.cpp
│   ├── line_detection_sensors.cpp
│   ├── ir_obstacle_detection_sensors.cpp
│   ├── magnetometer.cpp
│   └── ...
│
└── README.md
```

---

# ⭐ Key Features

- Autonomous navigation using IMU, line sensors, ultrasonic sensors, IR, and Time-of-Flight sensing  
- Obstacle avoidance using ODDM (Obstacle Detection & Decision Making) logic  
- Boundary detection and correction using LDDM (Line Detection & Decision Making)  
- Plowing mechanism capable of pushing 20 mm wooden blocks  
- Modular software architecture (SensorManager, MotorController, FSM-like loop)  
- UART-based subsystem communication (Due–Nano)  
- Safety watchdog, emergency stop, and motion recovery routines  

---

# ⚙️ Hardware Summary

### **Microcontroller**
- Arduino Due

### **Sensors**
- VL53L1X Time-of-Flight  
- HC-SR04 Ultrasonic sensors  
- VMA330 IR proximity sensors  
- DFRobot Line Follower array  
- LIS3MDL Magnetometer  

### **Actuation**
- 4× DC motors  
- Cytron motor driver  

### **Power**
- 7.4 V Li-Po battery with 5 V regulator  

---

# 🧠 Software Architecture Overview

### **Top-Level Controller (`roboplow.ino`)**
Coordinates:
- SensorManager  
- MotorController  
- Movement routines  
- ODDM obstacle avoidance  
- LDDM line correction  

Operates as a **finite-state-like system** with major states:
- **MOVEMENT**
- **ODDM**
- **LDDM**

---

# 🧪 Testing Subsystems

Located in:

```
/roboplow/testing/
```

Includes:
- Motor test  
- Ultrasonic test  
- ToF test  
- Line sensor test  
- IR sensor test  
- Magnetometer calibration test  

Each `.ino` file can be uploaded individually for hardware verification.

---

# 🚀 Getting Started

### 1. Clone the repository
```
git clone https://github.com/RyanPage-cu/SYSC4805_Project_Repo
```

### 2. Open in Arduino IDE
```
File → Open → roboplow/roboplow.ino
```

### 3. Install required libraries
- VL53L1X  
- LIS3MDL  
- Arduino SAM (Due) support  

### 4. Upload to Arduino Due
Use the programming port.

---

# 📫 Contact

**Ryan Page**  
101268082  
RyanPage@cmail.carleton.ca
