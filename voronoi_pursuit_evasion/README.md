# Voronoi Pursuit-Evasion for PX4 Multi-Drone System

Complete implementation of 3D Voronoi-based pursuit-evasion game for PX4 drones in Gazebo simulation.

## 🎯 Overview

This system implements a **distributed** pursuit-evasion game where:
- 4 **Pursuers** (PX4 instances 1-4) attempt to capture the evader using Voronoi control strategy
- 1 **Evader** (PX4 instance 5) tries to escape by moving toward furthest Voronoi cell vertex
- **Centralized Voronoi computation** node calculates velocities at 50Hz
- **Distributed drone agents** execute velocity commands independently
- All data logged to individual CSV files for post-analysis

## 📁 File Structure

```
voronoi_pursuit_evasion/
├── config.yaml                    # Configuration (positions, parameters)
├── voronoi_controller.py          # Voronoi control algorithms
├── initial_positions.py           # Setup tetrahedral formation
├── voronoi_computation_node.py    # Centralized velocity calculator
├── drone_agent.py                 # Individual drone controller
├── launch_game.sh                 # Launch all game nodes
├── merge_csv.py                   # Merge individual CSV files
├── plot_static.py                 # Generate analysis plots
├── animate_3d.py                  # Create 3D animation GIF
└── data/                          # Logged data directory
```

## 🚀 Quick Start

### Prerequisites

1. **PX4 multi-drone simulation running:**
   ```bash
   cd /home/gaurav/ros2_ws/PX4-Autopilot-Official
   ./launch_multi_drones.sh  # Launches instances 1-5
   ```

2. **MicroXRCEAgent running:**
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

3. **px4_msgs built:**
   ```bash
   cd /home/gaurav/ros2_ws/PX4-Autopilot-Official/ros2_ws
   colcon build --packages-select px4_msgs
   source install/setup.bash
   ```

### Step 1: Initial Position Setup

Move all 5 drones to tetrahedral starting formation:

```bash
cd voronoi_pursuit_evasion
python3 initial_positions.py
```

**What it does:**
- Arms all 5 drones via MAVSDK
- Flies to offboard mode
- Moves drones to initial tetrahedral formation
- Waits until all reach target positions
- Exits automatically when ready

**Initial Formation:**
- Center: `(0, 0, 10m)` altitude
- Pursuers: 4 vertices of regular tetrahedron (radius 3m)
- Evader: At center of tetrahedron

### Step 2: Launch Game

Start the pursuit-evasion engine:

```bash
chmod +x launch_game.sh
./launch_game.sh
```

**What it launches:**
1. **Voronoi Computation Node** - Calculates velocity commands
2. **5 Drone Agents** - One for each drone (4 pursuers + 1 evader)

### Step 3: Start the Game

Publish game start signal:

```bash
ros2 topic pub --once /voronoi/start_game std_msgs/msg/Bool "{data: true}"
```

**Game will run until:**
- Any pursuer gets within 0.4m of evader (capture!)
- You press Ctrl+C

### Step 4: Merge and Visualize Data

```bash
# Merge individual CSVs
python3 merge_csv.py --auto

# Generate static plots
python3 plot_static.py --auto

# Create 3D animation
python3 animate_3d.py --auto
```

## 📊 Data Files

### Individual CSV Files
Each drone agent logs its own data:
```
data/drone_1_pursuer_20260212_143022.csv
data/drone_2_pursuer_20260212_143022.csv
data/drone_3_pursuer_20260212_143022.csv
data/drone_4_pursuer_20260212_143022.csv
data/drone_5_evader_20260212_143022.csv
```

**Columns:**
- `timestamp` - Time since game start (seconds)
- `drone_id` - Instance number (1-5)
- `role` - "pursuer" or "evader"
- `x, y, z` - Position (NED coordinates, meters)
- `vx, vy, vz` - Velocity (m/s)
- `roll, pitch, yaw` - Attitude (radians)
- `cmd_vx, cmd_vy, cmd_vz` - Commanded velocity from Voronoi (m/s)

### Merged CSV File
```
data/merged_all_drones_20260212_143022.csv
```
All 5 drones' data in single file, sorted by timestamp.

## 🎮 Architecture

### Distributed Design

```
┌─────────────────────────────────────┐
│  Voronoi Computation Node           │
│  - Subscribes: All 5 positions      │
│  - Computes: Voronoi velocities     │
│  - Publishes: /voronoi/cmd_vel/*    │
└─────────────────────────────────────┘
         │ ROS2 Topics (Twist msgs)
         ↓
┌────┬────┬────┬────┬────────────────┐
│ D1 │ D2 │ D3 │ D4 │ D5   Agents    │
│ P  │ P  │ P  │ P  │ E    Each:     │
│    │    │    │    │   - Subscribes │
│    │    │    │    │   - MAVSDKTrajectory │
│    │    │    │    │   - Logs CSV   │
└────┴────┴────┴────┴────────────────┘
```

**Why Distributed?**
- Realistic for real-world deployment
- Computation node can run on ground station
- Agents can run on individual drone companion computers
- Communication via standard ROS2 topics

### Control Flow

1. **Voronoi Node** (50Hz):
   - Reads all 5 drone positions
   - Computes pursuer velocities: `pursuer_voronoi_controller()`
   - Computes evader velocity: `evader_voronoi_controller()`
   - Publishes to `/voronoi/cmd_vel/instance_X`
   - Checks capture condition

2. **Drone Agents** (50Hz):
   - Subscribe to own velocity command
   - Publish `TrajectorySetpoint` with velocity field
   - Publish `OffboardControlMode` heartbeat
   - Log state to CSV

## ⚙️ Configuration

Edit `config.yaml` to customize:

### Formation Geometry
```yaml
formation:
  base_altitude: 10.0     # Center altitude (m)
  radius: 3.0             # Tetrahedron radius (m)
```

### Voronoi Parameters
```yaml
voronoi:
  gamma: 1.0              # Control law parameter
  pursuer_strategy: 2     # Strategy 2 from paper
  evader_strategy: 1      # Escape toward furthest corner
  max_velocity: 0.5       # Velocity limit (m/s)
```

### Game Rules
```yaml
game:
  capture_radius: 0.4     # Capture threshold (m)
  max_duration: 300.0     # Max game time (s)
  control_frequency: 50.0 # Computation rate (Hz)
```

## 📈 Visualization

### Static Plots (`plot_static.py`)
Generates:
- `trajectories_2d.png` - XY, XZ, YZ projections
- `distances.png` - Distance from each pursuer to evader vs time
- `velocities.png` - Velocity magnitudes vs time
- `positions_vs_time.png` - X, Y, Z positions vs time

### 3D Animation (`animate_3d.py`)
Creates animated GIF showing:
- Drone positions (colored markers)
- Trajectories (dotted lines)
- Initial positions (stars)
- Time evolution

## 🔧 Troubleshooting

### "No module named 'px4_msgs'"
```bash
cd ../ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

### Drones don't reach initial positions
- Check PX4 instances are running: `ps aux | grep px4`
- Verify MicroXRCEAgent connected
- Increase `position_threshold` in config.yaml

### Game doesn't start
- Check Voronoi node is running: `ros2 node list | grep voronoi`
- Verify agents subscribed: `ros2 topic info /voronoi/cmd_vel/instance_1`
- Publish start signal again

### Capture not detected
- Check `capture_radius` in config (may need larger value)
- Monitor distances: `ros2 topic echo /voronoi/capture`

## 🧪 Testing Individual Components

### Test initial positions only:
```bash
python3 initial_positions.py
# Drones will reach formation and hold
# Ctrl+C when satisfied
```

### Test Voronoi computation node:
```bash
python3 voronoi_computation_node.py
# Monitor velocity commands:
ros2 topic echo /voronoi/cmd_vel/instance_1
```

### Test single drone agent:
```bash
python3 drone_agent.py --instance 1 --role pursuer
```

## 📚 Algorithm Details

### Voronoi Pursuit Controller (Strategy 2)
```python
# For each pursuer i:
K[i] = Σ c[j] * ||corner[j]||^(γ-2) * (p[j] - corner[j])
v[i] = -(K[i] / ||K[i]||) * v_max
```

- Guarantees capture if γ=1
- Based on Voronoi decomposition of pursuit space
- Paper: [Insert paper reference]

### Evader Controller (Strategy 1)
```python
# Move toward furthest Voronoi corner
idx = argmax(||corner[i]||)
v_evader = corner[idx] / ||corner[idx]|| * v_max
```

- Optimal escape strategy
- Maximizes distance from all pursuers
- Delays capture time

## 🎓 Research Applications

- Multi-agent coordination
- Pursuit-evasion game theory
- Cooperative control
- Swarm robotics
- Formation control

## 📝 Citation

If you use this code in research, please cite:
```
[Your paper/thesis information]
```

## 🤝 Contributing

Based on [MultiAgentPE](https://github.com/deepboliya/MultiAgentPE) repository.

Adapted for PX4 simulation by [Your Name], February 2026.

## 📄 License

[Your license]

---

**Ready to run a pursuit-evasion game? Follow the Quick Start guide above!** 🚁🎯
