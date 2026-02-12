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

# Source ROS2 workspace (adjust path as needed)
if [ -f "/home/gaurav/ros2_ws/install/setup.bash" ]; then
    source /home/gaurav/ros2_ws/install/setup.bash
    echo "   Sourced ROS2 workspace"
elif [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source $HOME/ros2_ws/install/setup.bash
    echo "   Sourced ROS2 workspace"
else
    echo "   ⚠️  Warning: ROS2 workspace not found, some dependencies may be missing"
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
