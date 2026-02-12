# Multi-Drone Offboard Control - Usage Guide

## Overview
The hybrid MAVSDK+ROS2 offboard control system now supports controlling multiple drones simultaneously with command-line arguments.

## Files
- `hybrid_offboard_control.py` - Main control script (single or multiple drones)
- `multi_drone_mission.sh` - Bash script for concurrent multi-drone launch
- `multi_drone_control.py` - Python script for coordinated multi-drone control

## Single Drone Usage

### Basic (default: instance 1, 50m altitude, 5s hover)
```bash
python3 hybrid_offboard_control.py
```

### Custom parameters
```bash
# Drone instance 2, 30m altitude, 3s hover
python3 hybrid_offboard_control.py --instance 2 --altitude 30.0 --hover 3.0

# Short flags
python3 hybrid_offboard_control.py -i 3 -a 40.0 -t 4.0
```

### Help
```bash
python3 hybrid_offboard_control.py --help
```

## Multi-Drone Usage

### Option 1: Bash Script (Simple)
```bash
# Launch 3 drones with different missions
./multi_drone_mission.sh
```

Edit the script to customize individual drone parameters (altitude, hover time).

### Option 2: Python Concurrent Control (Advanced)
```bash
# Launch 3 drones with same parameters
python3 multi_drone_control.py --num-drones 3 --altitude 50.0 --hover 5.0

# Launch 5 drones with 2-second stagger between launches
python3 multi_drone_control.py -n 5 -a 40.0 -t 3.0 -s 2.0

# Help
python3 multi_drone_control.py --help
```

### Option 3: Manual Terminal Launch
Open multiple terminals and launch each drone separately:

**Terminal 1:**
```bash
python3 hybrid_offboard_control.py -i 1 -a 50.0 -t 5.0
```

**Terminal 2:**
```bash
python3 hybrid_offboard_control.py -i 2 -a 30.0 -t 3.0
```

**Terminal 3:**
```bash
python3 hybrid_offboard_control.py -i 3 -a 40.0 -t 4.0
```

## Port Mapping

The script automatically calculates ports based on drone instance:

| Instance | MAVLink Port | MAVSDK gRPC Port | Namespace |
|----------|--------------|------------------|-----------|
| 1        | 14541        | 50051            | /px4_1/fmu |
| 2        | 14542        | 50052            | /px4_2/fmu |
| 3        | 14543        | 50053            | /px4_3/fmu |
| N        | 14540+N      | 50050+N          | /px4_N/fmu |

## Prerequisites

1. **Launch PX4 multi-drone simulation:**
   ```bash
   cd /home/gaurav/ros2_ws/PX4-Autopilot-Official
   ./launch_multi_drones.sh  # Launches 3 drones by default
   ```

2. **Start MicroXRCEAgent:**
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

3. **Verify drones are running:**
   ```bash
   ros2 topic list | grep px4
   ```

## Example Missions

### Formation Flight Test
```bash
# All drones to 50m, different hover times
python3 hybrid_offboard_control.py -i 1 -a 50.0 -t 5.0 &
python3 hybrid_offboard_control.py -i 2 -a 50.0 -t 7.0 &
python3 hybrid_offboard_control.py -i 3 -a 50.0 -t 3.0 &
wait
```

### Staggered Altitude Test
```bash
# Each drone at different altitude
./multi_drone_mission.sh  # Pre-configured for 50m, 30m, 40m
```

### Large Fleet Test
```bash
# 5 drones, 2-second stagger, 40m altitude
python3 multi_drone_control.py -n 5 -a 40.0 -t 3.0 -s 2.0
```

## Troubleshooting

### "Connection refused" on MAVSDK port
- Ensure PX4 instance is running for that drone number
- Check `ps aux | grep px4` to see active instances
- Verify port not in use: `netstat -tuln | grep 5005`

### ROS2 topics not found
- Check namespace: `ros2 topic list | grep px4_<instance>`
- Verify MicroXRCEAgent running: `ps aux | grep MicroXRCEAgent`

### Drone doesn't arm
- Wait 2-3 seconds after launch for GPS/position lock
- Check health: monitor MAVSDK connection logs
- Ensure `COM_RCL_EXCEPT=4` set in PX4 parameters

## Changes from Original

### Removed
- ❌ Hardcoded instance/ports
- ❌ Manual disarm call (causes errors, auto-disarm after landing)

### Added
- ✅ Command-line argument parsing
- ✅ Automatic port calculation
- ✅ Multi-drone support
- ✅ Instance-based configuration
- ✅ Usage documentation in script header

## Next Steps

1. **Test single drone:** `python3 hybrid_offboard_control.py -i 1`
2. **Test multi-drone:** `./multi_drone_mission.sh`
3. **Implement formation control:** Modify setpoints for coordinated flight
4. **Add trajectory planning:** Replace linear climb with bezier curves

---
Created: 2026-02-11
Author: GitHub Copilot
