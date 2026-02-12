# Voronoi Pursuit-Evasion Integration Plan Analysis
## Plan Review for PX4 Multi-Drone Simulation

**Date:** 12 February 2026  
**Objective:** Integrate 3D Voronoi pursuit-evasion strategy with PX4 Gazebo simulation

---

## ✅ Your Original Plan Summary

1. **Universal Python code** with Voronoi calculations and control
2. **Launch 5 agents** (4 pursuers + 1 evader) using hybrid MAVSDK/ROS2
3. **Reach initial positions** (tetrahedral formation) before starting
4. **Record all data** (positions, velocities, controls, orientations)
5. **Save with timestamps** in CSV/TXT format
6. **Plotting scripts** for static and animated 3D trajectories

---

## 🔍 Critical Analysis

### ✅ Strengths of Your Plan

1. **Hybrid MAVSDK+ROS2** - Already proven working, excellent choice
2. **Data logging** - Essential for research, well thought out
3. **Initial position coordination** - Critical for consistent experiments
4. **Comprehensive visualization** - Static + animated plots are valuable
5. **Tetrahedral formation** - Geometrically sound for 3D pursuit-evasion

### ⚠️ Issues and Corrections Needed

---

## 🚨 **MAJOR ISSUE #1: Architecture Choice**

### Your Plan Says: "Universal Python code for all 5 agents"

**Problem:** This is ambiguous - do you mean:
- **Option A:** Each drone runs the same script independently (distributed)?
- **Option B:** One coordinator script controls all 5 drones (centralized)?

### ✅ **RECOMMENDED: Centralized Coordinator**

**Why Centralized is Better:**

1. **Voronoi requires all positions** simultaneously to compute controls
2. **Synchronization is trivial** - one event loop coordinates everything
3. **Data logging is consistent** - single node records all states
4. **Easier debugging** - one process, one log file
5. **Matches MultiAgentPE design** - PE_node.py is centralized

**Centralized Architecture:**
```
┌─────────────────────────────────────────┐
│  Coordinator Node (Single Python Script)│
│                                          │
│  1. Subscribes to all 5 drone positions │
│  2. Computes Voronoi controls for all   │
│  3. Publishes setpoints to all 5 drones │
│  4. Logs all data to single CSV         │
│  5. Detects capture condition           │
└─────────────────────────────────────────┘
         ↓ MAVSDK + ROS2 ↓
┌────┬────┬────┬────┬────┐
│ P1 │ P2 │ P3 │ P4 │ E1 │  (PX4 Gazebo Drones)
└────┴────┴────┴────┴────┘
```

**Distributed would require:**
- State sharing via ROS2 topics (extra complexity)
- Synchronized clocks (timing issues)
- Duplicate Voronoi calculations (inefficient)
- Coordination barriers (complex)

### 📝 **Correction #1:** Use centralized coordinator, not distributed "universal code"

---

## 🚨 **MAJOR ISSUE #2: Synchronization Mechanism**

### Your Plan Says: "Start game at same time after reaching positions"

**Problem:** How do you know when all 5 drones are ready?

### ✅ **SOLUTION: State Machine with Barriers**

```python
States:
1. INITIALIZING  - Connecting to drones
2. ARMING        - Arming all via MAVSDK
3. TAKING_OFF    - Ascending to safe altitude
4. MOVING_TO_INIT- Flying to tetrahedral positions
5. WAITING_READY - Checking all within threshold
6. GAME_ACTIVE   - Running Voronoi control loop
7. CAPTURED      - Evader caught, landing
8. LANDED        - Mission complete
```

**Readiness Check:**
```python
def all_drones_ready(self, target_positions, threshold=0.5):
    """Check if all drones within threshold of targets"""
    for i, drone in enumerate(self.drones):
        current_pos = drone.position  # From VehicleLocalPosition
        target_pos = target_positions[i]
        distance = np.linalg.norm(current_pos - target_pos)
        if distance > threshold:
            return False
    return True
```

### 📝 **Correction #2:** Add explicit state machine with position threshold checking

---

## 🚨 **MAJOR ISSUE #3: Control Loop Integration**

### Your Plan Says: "Universal code with all calculations"

**Problem:** How does Voronoi velocity integrate with hybrid control?

### ✅ **SOLUTION: Velocity → Trajectory Setpoint Conversion**

**Current Hybrid Script:**
- Publishes `TrajectorySetpoint` with position (x, y, z)
- 50Hz heartbeat for `OffboardControlMode`

**MultiAgentPE:**
- Computes velocity commands (vx, vy, vz)
- 10Hz update rate

**Integration Required:**
```python
# In control loop (50Hz):
def control_loop(self):
    self.publish_offboard_mode()  # Heartbeat
    
    if self.state == "GAME_ACTIVE":
        # Get current positions
        pursuer_poses = self.get_pursuer_positions()  # 4x3 array
        evader_pos = self.get_evader_position()        # 1x3 array
        
        # Compute Voronoi velocities
        relative_poses = pursuer_poses - evader_pos
        pursuer_vels, voronoi_corners, _, _, _, K = pursuer_voronoi_controller(
            relative_poses, strategy=2, dimension=3, gamma=1, norm_bound=0.5
        )
        evader_vel = evader_voronoi_controller(
            voronoi_corners, K, strategy=1, norm_bound=0.5
        )
        
        # Convert velocity to position setpoint (integrate over dt)
        dt = 0.02  # 50Hz
        for i, drone in enumerate(self.pursuers):
            new_pos = drone.position + pursuer_vels[i] * dt
            self.publish_setpoint(drone, new_pos)
        
        evader_new_pos = self.evader.position + evader_vel * dt
        self.publish_setpoint(self.evader, evader_new_pos)
```

**OR Better: Use TrajectorySetpoint velocity field**
```python
msg = TrajectorySetpoint()
msg.position = [float('nan'), float('nan'), float('nan')]  # Ignore position
msg.velocity = [vx, vy, vz]  # Use velocity control
msg.yaw = 0.0
```

### 📝 **Correction #3:** Integrate Voronoi at 10-20Hz, publish setpoints at 50Hz

---

## 🚨 **ISSUE #4: Data Logging Structure**

### Your Plan Says: "All data recorded"

**Problem:** What exact data fields? How to handle 5 drones in one file?

### ✅ **SOLUTION: Structured CSV Format**

**Recommended CSV Structure:**
```csv
timestamp,drone_id,role,x,y,z,vx,vy,vz,roll,pitch,yaw,cmd_vx,cmd_vy,cmd_vz,distance_to_evader
0.000,1,pursuer,-1.0,0.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.1,0.05,0.0,5.23
0.000,2,pursuer,1.0,0.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.1,0.05,0.0,5.11
...
0.000,5,evader,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.15,-0.2,0.0,0.0
0.05,1,pursuer,-0.995,0.005,2.0,0.1,0.05,0.0,0.01,0.02,0.0,0.12,0.06,0.0,5.18
...
```

**Data Sources:**
- `VehicleLocalPosition` → x, y, z, vx, vy, vz
- `VehicleAttitude` → roll, pitch, yaw (need to add subscription)
- Voronoi controller → cmd_vx, cmd_vy, cmd_vz
- Distance calculation → distance_to_evader

### 📝 **Correction #4:** Define explicit CSV schema with all required fields

---

## 🚨 **ISSUE #5: Capture Detection**

### Your Plan Doesn't Mention: How to know when game ends?

### ✅ **SOLUTION: Capture Radius Check**

```python
def check_capture(self, capture_radius=0.5):
    """Check if any pursuer has captured the evader"""
    evader_pos = self.evader.position
    for pursuer in self.pursuers:
        distance = np.linalg.norm(pursuer.position - evader_pos)
        if distance < capture_radius:
            self.get_logger().info(f'🎯 CAPTURED by Pursuer {pursuer.instance}!')
            self.state = "CAPTURED"
            self.capture_time = self.get_time()
            return True
    return False
```

**Capture Radius Tuning:**
- Crazyflie (MultiAgentPE): 0.05-0.1m (very agile)
- PX4 Simulation: 0.3-0.5m (less agile, larger drones)

### 📝 **Correction #5:** Add capture detection with tunable radius

---

## 🚨 **ISSUE #6: Voronoi Library Import**

### Your Plan Says: "All calculations in universal code"

**Problem:** Rewriting Voronoi math is error-prone and unnecessary

### ✅ **SOLUTION: Import from MultiAgentPE**

**Build the library first:**
```bash
cd /home/gaurav/ros2_ws/MultiAgentPE/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pursuit_evasion_lib --symlink-install
source install/setup.bash
```

**Then import in your code:**
```python
import sys
sys.path.append('/home/gaurav/ros2_ws/MultiAgentPE/ros2_ws/install/pursuit_evasion_lib/lib/python3.10/site-packages')

from pursuit_evasion_lib.controllers.voronoi import (
    pursuer_voronoi_controller, 
    evader_voronoi_controller
)
from pursuit_evasion_lib.controllers.utils import voronoi_corner_new
```

**OR better: Add to PYTHONPATH in launch script**

### 📝 **Correction #6:** Import Voronoi from MultiAgentPE, don't rewrite

---

## 🚨 **ISSUE #7: Initial Positions Definition**

### Your Plan Says: "Tetrahedral positions"

**Question:** What are the exact coordinates?

### ✅ **SOLUTION: Configuration File**

**Create `voronoi_config.yaml`:**
```yaml
# 3D Pursuit-Evasion Configuration

# Tetrahedral formation for 4 pursuers
# Evader starts at origin (0, 0, 0) at altitude base_altitude
# Pursuers form tetrahedron around evader

base_altitude: 10.0  # meters above ground
tetrahedron_radius: 3.0  # meters from center to vertices

# Initial positions will be computed as:
# Evader: [0, 0, base_altitude]
# Pursuers: vertices of regular tetrahedron centered at evader

pursuer_instances: [1, 2, 3, 4]  # PX4 instance numbers
evader_instance: 5

# Voronoi parameters
gamma: 1.0                # Control law parameter (1 for guaranteed capture)
pursuer_strategy: 2       # Strategy from paper
evader_strategy: 1        # Move toward furthest corner
norm_bound: 0.5           # Max velocity (m/s)
capture_radius: 0.5       # Capture threshold (m)

# Control parameters
control_frequency: 20.0   # Hz (Voronoi computation)
publish_frequency: 50.0   # Hz (ROS2 setpoint publishing)

# Simulation parameters
max_simulation_time: 300.0  # seconds
dt: 0.02                    # time step for integration

# Safety parameters
max_altitude: 50.0
min_altitude: 5.0
boundary_radius: 20.0     # Maximum distance from origin
```

**Python code to generate tetrahedron:**
```python
def generate_tetrahedral_positions(center, radius):
    """Generate 4 vertices of regular tetrahedron centered at 'center'"""
    # Regular tetrahedron vertices
    vertices = np.array([
        [1, 1, 1],
        [1, -1, -1],
        [-1, 1, -1],
        [-1, -1, 1]
    ]) * radius / np.sqrt(3)
    
    return vertices + center
```

### 📝 **Correction #7:** Create config file with tetrahedral geometry

---

## 📋 **REVISED IMPLEMENTATION PLAN**

### **File Structure:**
```
PX4-Autopilot-Official/
├── voronoi_pursuit_evasion/
│   ├── px4_voronoi_coordinator.py      # Main coordinator node
│   ├── voronoi_config.yaml             # Configuration
│   ├── data_logger.py                  # CSV logging class
│   ├── visualize_static.py             # Static plots
│   ├── animate_trajectory.py           # 3D animation
│   └── data/                           # Logged data directory
│       └── pe_YYYYMMDD_HHMMSS.csv
```

### **Implementation Steps:**

#### **Phase 1: Core Coordinator (1-2 hours)**
1. Create `PX4VoronoiCoordinator` class extending ROS2 Node
2. Initialize 5 drones (4 pursuers + 1 evader) with MAVSDK
3. Subscribe to VehicleLocalPosition and VehicleAttitude for all
4. Implement state machine (INITIALIZING → LANDED)
5. Test basic connectivity and state transitions

#### **Phase 2: Position Control (1 hour)**
1. Load config file (tetrahedral positions)
2. Implement arm + takeoff sequence via MAVSDK
3. Implement move-to-position using TrajectorySetpoint
4. Test all 5 drones reach initial positions
5. Add readiness barrier check

#### **Phase 3: Voronoi Integration (2 hours)**
1. Build and import pursuit_evasion_lib
2. Integrate Voronoi controllers in control loop
3. Convert velocities → trajectory setpoints
4. Test pursuers chase evader
5. Verify control frequencies (20Hz compute, 50Hz publish)

#### **Phase 4: Data Logging (1 hour)**
1. Create DataLogger class
2. Subscribe to all necessary topics
3. Log at 20Hz to CSV with proper schema
4. Add metadata header (config parameters)
5. Test data saved correctly with timestamps

#### **Phase 5: Capture & Shutdown (30 min)**
1. Implement capture detection
2. Add graceful landing sequence
3. Ensure data saved before exit
4. Test full mission: init → capture → land → save

#### **Phase 6: Visualization (1-2 hours)**
1. Create static plotter (positions vs time, distances, etc.)
2. Create 3D animator with matplotlib
3. Show initial positions, trajectories, Voronoi regions
4. Test with logged data

**Total Estimated Time: 6-8 hours**

---

## 🎯 **CORRECTED PLAN SUMMARY**

### **What Changes from Original Plan:**

| Original                          | Corrected                                    |
|-----------------------------------|----------------------------------------------|
| "Universal code for all agents"   | **Centralized coordinator controls all 5**   |
| Implicit synchronization          | **Explicit state machine with barriers**     |
| Generic "calculations"            | **Import Voronoi from MultiAgentPE**         |
| Unspecified data format           | **Structured CSV with defined schema**       |
| No capture detection              | **Distance threshold with tunable radius**   |
| No tetrahedral geometry           | **Config file with computed positions**      |
| No control loop details           | **20Hz Voronoi + 50Hz setpoint publish**     |

---

## ⚡ **CRITICAL DECISIONS BEFORE CODING**

### **1. Velocity vs Position Setpoints?**
- **Option A:** Integrate velocities → new positions (your hybrid script style)
- **Option B:** Use TrajectorySetpoint velocity field directly
- **Recommendation:** Option B (cleaner, less integration error)

### **2. Data Logging Frequency?**
- **10 Hz:** Lightweight, sufficient for analysis
- **20 Hz:** Good balance (recommended)
- **50 Hz:** Very detailed but large files

### **3. PX4 Instance Allocation?**
- **Pursuers:** Instances 1, 2, 3, 4
- **Evader:** Instance 5
- **Confirm this matches your `launch_multi_drones.sh`**

### **4. Safety Features?**
- Altitude limits (5m - 50m)?
- Boundary sphere (20m radius)?
- Emergency stop on keyboard interrupt?
- **Recommendation:** Yes to all

---

## 🚀 **NEXT STEPS**

### **Before I Write Code:**

1. **Confirm Architecture:** Centralized coordinator is OK?
2. **Confirm Control Method:** Use velocity setpoints (Option B)?
3. **Confirm Instances:** Pursuers=1-4, Evader=5?
4. **Confirm Safety:** Add boundaries and altitude limits?
5. **Review Config:** Is `voronoi_config.yaml` structure acceptable?

### **Once Confirmed:**
I will create:
1. `px4_voronoi_coordinator.py` - Complete coordinator (500-600 lines)
2. `voronoi_config.yaml` - Configuration file
3. `data_logger.py` - Logging utility
4. `visualize_static.py` - Static plotting
5. `animate_trajectory.py` - 3D animation
6. `README_VORONOI_PE.md` - Usage documentation

---

## 📌 **CONCLUSION**

**Your plan is SOLID in concept** but needs these critical corrections:

✅ Use **centralized coordinator** (not distributed universal script)  
✅ Add **explicit synchronization** with state machine  
✅ **Import Voronoi library** (don't rewrite)  
✅ Define **data schema** clearly  
✅ Add **capture detection**  
✅ Use **config file** for parameters  
✅ Specify **control frequencies** (20Hz + 50Hz)

**Once you confirm the corrections above, I'm ready to implement the complete system!** 🚁🎯

