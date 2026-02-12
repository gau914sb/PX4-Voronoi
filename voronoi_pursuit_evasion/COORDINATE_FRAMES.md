# Coordinate Frame Reference for Multi-Drone System

## Overview

This document explains the coordinate frame transformations required when working with multiple PX4 drones in Gazebo simulation. **Critical for correct formation control and position feedback.**

---

## The Problem

Each drone has its **own local coordinate frame** centered at its spawn position. To form coordinated formations (like a tetrahedron), all drones must work in a **common global frame**.

### Spawn Configuration

From `launch_multi_drones.sh`:
```bash
Y_POS=$(echo "($i - 1) * 2.0" | bc)
PX4_GZ_MODEL_POSE="0,$Y_POS"  # Gazebo spawn positions
```

**Spawn positions in Gazebo:**
- Drone 1 (instance 1): `(0, 0)` 
- Drone 2 (instance 2): `(0, 2)` → 2m offset
- Drone 3 (instance 3): `(0, 4)` → 4m offset
- Drone 4 (instance 4): `(0, 6)` → 6m offset
- Drone 5 (instance 5): `(0, 8)` → 8m offset

---

## Coordinate Frame Conversions

### Gazebo → PX4 Frame Convention

**Gazebo uses ENU (East, North, Up):**
- X = East
- Y = North  
- Z = Up

**PX4 uses NED (North, East, Down):**
- X = North
- Y = East
- Z = Down

### Conversion Rules

| Gazebo ENU | → | PX4 NED |
|------------|---|---------|
| X (East)   | → | Y (East) |
| Y (North)  | → | X (North) |
| Z (Up)     | → | -Z (Down) |

### Spawn Offset in NED Coordinates

```python
def get_spawn_offset(instance):
    """
    Calculate spawn offset for drone instance in NED frame
    
    Gazebo spawns at (0, Y_POS) in ENU where Y_POS = (instance - 1) * 2.0
    This converts to (X_POS, 0) in NED where X_POS = (instance - 1) * 2.0
    
    Returns: [X, Y, Z] in NED coordinates (meters)
    """
    return np.array([(instance - 1) * 2.0, 0.0, 0.0])
```

**Example spawn offsets:**
- Instance 1: `[0.0, 0.0, 0.0]` - reference drone
- Instance 2: `[2.0, 0.0, 0.0]` - 2m north of reference
- Instance 3: `[4.0, 0.0, 0.0]` - 4m north of reference
- Instance 4: `[6.0, 0.0, 0.0]` - 6m north of reference
- Instance 5: `[8.0, 0.0, 0.0]` - 8m north of reference

---

## Implementation in Code

### Position Feedback (Local → Global)

When receiving position from PX4 (via `VehicleLocalPosition`):

```python
# PX4 reports position relative to drone's HOME (spawn position)
position_local = np.array([msg.x, msg.y, msg.z])  # NED relative to home

# Convert to global frame
spawn_offset = get_spawn_offset(instance)
position_global = position_local + spawn_offset
```

**Example for Drone 2 (instance 2):**
- Local position from PX4: `[1.5, -0.5, -10.0]`
- Spawn offset: `[2.0, 0.0, 0.0]`
- Global position: `[3.5, -0.5, -10.0]`

### Position Setpoints (Global → Local)

When sending position commands to PX4 (via `TrajectorySetpoint`):

```python
# You have a target in global coordinates
target_global = np.array([2.83, 0.0, -9.0])  # NED global

# Convert to local frame for this drone
spawn_offset = get_spawn_offset(instance)
target_local = target_global - spawn_offset

# Publish to PX4
msg = TrajectorySetpoint()
msg.position = [target_local[0], target_local[1], target_local[2]]
```

**Example for Drone 2 (instance 2):**
- Global target: `[2.83, 0.0, -9.0]`
- Spawn offset: `[2.0, 0.0, 0.0]`
- Local target: `[0.83, 0.0, -9.0]` ← what to send to PX4

---

## DroneController Class Pattern

**Recommended pattern used in `initial_positions.py`:**

```python
class DroneController(Node):
    def __init__(self, instance, mavsdk_port, mavlink_port):
        super().__init__(f'drone_{instance}')
        self.instance = instance
        
        # Auto-calculate spawn offset
        self.spawn_offset = np.array([(instance - 1) * 2.0, 0.0, 0.0])
        
        # Store positions in local frame (as PX4 reports)
        self.position_local = np.array([0.0, 0.0, 0.0])
        self.target_local = np.array([0.0, 0.0, -5.0])
    
    @property
    def position_global(self):
        """Convert local position to global frame"""
        return self.position_local + self.spawn_offset
    
    @property
    def target_global(self):
        """Convert local target to global frame"""
        return self.target_local + self.spawn_offset
    
    def publish_setpoint(self, x, y, z, frame='global'):
        """
        Publish position setpoint
        
        Args:
            x, y, z: Position coordinates
            frame: 'global' or 'local'
        """
        if frame == 'global':
            target_global = np.array([x, y, z])
            self.target_local = target_global - self.spawn_offset
        else:
            self.target_local = np.array([x, y, z])
        
        msg = TrajectorySetpoint()
        msg.position = [float(self.target_local[0]), 
                       float(self.target_local[1]), 
                       float(self.target_local[2])]
        msg.timestamp = self.timestamp()
        self.setpoint_pub.publish(msg)
```

---

## Usage in Voronoi Controller

### Key Principles

1. **All Voronoi calculations** should be in **global frame**
2. **Position feedback** must be converted to global: `position_global = position_local + spawn_offset`
3. **Position commands** must be converted to local: `target_local = target_global - spawn_offset`
4. **Distance calculations** must use global positions

### Example: Voronoi Pursuit-Evasion

```python
class VoronoiController:
    def __init__(self, instance):
        self.instance = instance
        self.spawn_offset = np.array([(instance - 1) * 2.0, 0.0, 0.0])
        
        # Subscribe to all drones' positions
        self.drone_positions = {}  # Store in GLOBAL frame
    
    def position_callback(self, msg, drone_id):
        """Receive position from any drone"""
        local_pos = np.array([msg.x, msg.y, msg.z])
        offset = np.array([(drone_id - 1) * 2.0, 0.0, 0.0])
        
        # Store in global frame
        self.drone_positions[drone_id] = local_pos + offset
    
    def compute_voronoi_control(self):
        """Compute control based on Voronoi partition"""
        # All positions are already in global frame
        my_pos = self.drone_positions[self.instance]
        other_positions = [pos for id, pos in self.drone_positions.items() 
                          if id != self.instance]
        
        # Compute Voronoi control in global frame
        control_velocity = self.voronoi_law(my_pos, other_positions)
        
        # Integrate to get target position in global frame
        target_global = my_pos + control_velocity * dt
        
        # Convert to local and publish
        target_local = target_global - self.spawn_offset
        self.publish_setpoint(target_local)
```

---

## Distance Calculations

**Always use global positions for distance/capture checks:**

```python
# ❌ WRONG - using local positions
distance = np.linalg.norm(pursuer.position_local - evader.position_local)

# ✅ CORRECT - using global positions  
distance = np.linalg.norm(pursuer.position_global - evader.position_global)

# Check capture
if distance < CAPTURE_RADIUS:
    print("Evader captured!")
```

---

## Altitude Convention (NED Z-axis)

**Remember: PX4 uses NED where Z is DOWN**

- Ground level: `z = 0`
- 10 meters altitude: `z = -10.0` (negative!)
- 5 meters altitude: `z = -5.0` (negative!)

**When specifying altitudes:**

```python
# Want drone at 10m altitude
altitude_meters = 10.0
z_ned = -altitude_meters  # z = -10.0

# Convert altitude to NED position
position = np.array([x, y, -altitude_meters])
```

---

## Quick Reference Formulas

### For Each Drone Instance

```python
# Spawn offset (NED)
spawn_offset = [(instance - 1) * 2.0, 0.0, 0.0]

# Local → Global
position_global = position_local + spawn_offset

# Global → Local  
position_local = position_global - spawn_offset
```

### For Multi-Drone Formation

```python
# Define formation in global frame
global_targets = {
    1: [0.0, 0.0, -13.0],     # Apex
    2: [2.83, 0.0, -9.0],      # Base vertex 1
    3: [-1.41, 2.45, -9.0],    # Base vertex 2
    4: [-1.41, -2.45, -9.0],   # Base vertex 3
    5: [0.0, 0.0, -10.0]       # Center (evader)
}

# Convert for each drone
for instance, global_target in global_targets.items():
    offset = [(instance - 1) * 2.0, 0.0, 0.0]
    local_target = global_target - offset
    publish_to_drone(instance, local_target)
```

---

## Validation Checks

### After Implementation

1. **Verify spawn offsets are correct:**
   ```python
   # All drones should report (0, 0, 0) immediately after spawn
   assert np.allclose(position_local, [0, 0, 0])
   ```

2. **Verify global positions match spawn:**
   ```python
   # Drone 2 at ground should be at global [2.0, 0, 0]
   assert np.allclose(drone2.position_global, [2.0, 0, 0])
   ```

3. **Verify formation shape:**
   ```python
   # Check distances in global frame
   distances = [np.linalg.norm(p1 - p2) for p1, p2 in pairs]
   # For tetrahedron with radius=3m, all edges should be consistent
   ```

---

## Common Pitfalls

### ❌ Don't Do This

```python
# Using local positions for multi-drone operations
distance = np.linalg.norm(drone1.position_local - drone2.position_local)

# Mixing frames
target_local = target_global  # Forgot to subtract offset!

# Hardcoding spawn positions
spawn_offset = [0, 2, 0]  # Wrong axis! Should be [2, 0, 0]
```

### ✅ Do This

```python
# Always specify frame explicitly
distance = np.linalg.norm(drone1.position_global - drone2.position_global)

# Use helper function
spawn_offset = get_spawn_offset(instance)

# Add assertions
assert spawn_offset[1] == 0.0, "Y offset should be zero in NED"
assert spawn_offset[0] == (instance - 1) * 2.0, "X offset calculation wrong"
```

---

## Testing Procedure

To verify coordinate transformations are working:

1. **Launch drones and check initial positions**
   ```bash
   ros2 topic echo /px4_1/fmu/out/vehicle_local_position_v1
   # Should show (0, 0, 0) on ground
   ```

2. **Command simple global target**
   ```python
   # All drones go to same global point
   target_global = [10.0, 5.0, -10.0]
   # They should converge to same location in Gazebo
   ```

3. **Verify formation**
   ```python
   # Check actual Gazebo positions match expected global positions
   # Use gz topic -e -t /model/x500_{instance}/pose
   ```

---

## Summary

| Operation | Formula | Frame |
|-----------|---------|-------|
| **Spawn offset** | `[(instance-1)*2.0, 0, 0]` | NED |
| **Position feedback** | `global = local + offset` | Local → Global |
| **Position setpoint** | `local = global - offset` | Global → Local |
| **Voronoi control** | Use global positions | Global |
| **Distance checks** | Use global positions | Global |
| **Altitude** | `z = -altitude_meters` | NED (negative up) |

**Golden Rule:** Think in global, publish in local!
