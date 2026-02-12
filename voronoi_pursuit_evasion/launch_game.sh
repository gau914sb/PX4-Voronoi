#!/bin/bash
# Launch Voronoi Pursuit-Evasion Game
# Starts Voronoi computation node + all 5 drone agents

echo "🎮 Launching Voronoi Pursuit-Evasion Game"
echo "=========================================="
echo ""

# Check if config exists
if [ ! -f "config.yaml" ]; then
    echo "❌ Error: config.yaml not found!"
    exit 1
fi

# Source ROS2 workspaces for PX4
PX4_BASE="/home/gaurav/ros2_ws/PX4-Autopilot-Official"

if [ -f "$PX4_BASE/install/setup.bash" ] && [ -f "$PX4_BASE/ros2_ws/install/setup.bash" ]; then
    source $PX4_BASE/install/setup.bash
    source $PX4_BASE/ros2_ws/install/setup.bash
    echo "   ✅ Sourced PX4 ROS2 workspaces"
else
    echo "   ❌ Error: PX4 ROS2 workspaces not found!"
    echo "   Expected: $PX4_BASE/install/setup.bash"
    echo "            $PX4_BASE/ros2_ws/install/setup.bash"
    exit 1
fi

echo "1️⃣  Starting Voronoi Computation Node..."
python3 voronoi_computation_node.py &
VORONOI_PID=$!
echo "   PID: $VORONOI_PID"
sleep 2

echo ""
echo "2️⃣  Starting Drone Agents..."

# Launch pursuers (instances 1-4)
for i in 1 2 3 4; do
    python3 drone_agent.py --instance $i --role pursuer &
    AGENT_PID=$!
    echo "   Pursuer $i (PID: $AGENT_PID)"
    sleep 1
done

# Launch evader (instance 5)
python3 drone_agent.py --instance 5 --role evader &
EVADER_PID=$!
echo "   Evader 5 (PID: $EVADER_PID)"
sleep 2

echo ""
echo "✅ All nodes launched!"
echo ""
echo "📡 To start the game, publish to /voronoi/start_game:"
echo "   ros2 topic pub --once /voronoi/start_game std_msgs/msg/Bool \"{data: true}\""
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Wait for user interrupt
wait

echo ""
echo "🛑 Shutting down..."
