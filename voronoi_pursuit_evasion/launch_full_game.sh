#!/bin/bash
# Complete Voronoi Game Launch Script
# Handles initial positions → agent handover → game start automatically

set -e  # Exit on error

echo "🎮 Voronoi Pursuit-Evasion: Full Game Launch"
echo "=============================================="
echo ""

# Source ROS2 workspaces
PX4_BASE="/home/gaurav/ros2_ws/PX4-Autopilot-Official"

if [ -f "$PX4_BASE/install/setup.bash" ] && [ -f "$PX4_BASE/ros2_ws/install/setup.bash" ]; then
    source $PX4_BASE/install/setup.bash
    source $PX4_BASE/ros2_ws/install/setup.bash
    echo "✅ Sourced PX4 ROS2 workspaces"
else
    echo "❌ Error: PX4 ROS2 workspaces not found!"
    exit 1
fi

# Check if config exists
if [ ! -f "config.yaml" ]; then
    echo "❌ Error: config.yaml not found!"
    echo "   Run this script from voronoi_pursuit_evasion directory"
    exit 1
fi

# Configuration
POSITION_SETUP_TIME=40  # Total time for initial_positions.py to hold (auto-handover timeout)
AGENT_START_TIME=32     # Start agents before timeout to ensure smooth handover
AUTO_START_GAME=true    # Automatically start game after handover

# Trap Ctrl+C to clean up all processes
cleanup() {
    echo ""
    echo "🛑 Shutting down all processes..."
    kill $(jobs -p) 2>/dev/null || true
    sleep 1
    echo "✅ Cleanup complete"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo ""
echo "📍 STEP 1: Moving drones to initial positions"
echo "   (tetrahedral formation with evader at center)"
echo ""

# Launch initial_positions.py in background with auto-handover timeout
python3 initial_positions.py --auto-handover $POSITION_SETUP_TIME &
POSITION_PID=$!
echo "   Started initial_positions.py (PID: $POSITION_PID)"
echo "   Auto-handover timeout: ${POSITION_SETUP_TIME}s"

echo ""
echo "⏳ Waiting ${AGENT_START_TIME}s for drones to reach positions..."
echo "   (You can monitor progress in the initial_positions.py output above)"
echo ""

# Wait for drones to approach positions (not full timeout)
sleep $AGENT_START_TIME

echo ""
echo "🤖 STEP 2: Launching Voronoi controller and agents"
echo "   (Starting agents BEFORE initial_positions exits for smooth handover)"
echo ""

# Start Voronoi computation node
echo "   Starting Voronoi computation node..."
python3 voronoi_computation_node.py &
VORONOI_PID=$!
echo "   ✓ Voronoi PID: $VORONOI_PID"
sleep 2

# Start drone agents
echo ""
echo "   Starting drone agents..."
for i in 1 2 3 4; do
    python3 drone_agent.py --instance $i --role pursuer &
    AGENT_PID=$!
    echo "   ✓ Pursuer $i (PID: $AGENT_PID)"
    sleep 0.5
done

python3 drone_agent.py --instance 5 --role evader &
EVADER_PID=$!
echo "   ✓ Evader 5 (PID: $EVADER_PID)"
sleep 2

echo ""
echo "🔄 STEP 3: Handover in progress"
echo "   initial_positions.py will auto-exit when agents take over..."
echo ""

# Wait a bit for handover to complete
sleep 5

# Check if initial_positions is still running
if ps -p $POSITION_PID > /dev/null 2>&1; then
    echo "   ⏳ Waiting for handover to complete..."
    # Wait up to 10 more seconds for graceful exit
    for i in {1..10}; do
        if ! ps -p $POSITION_PID > /dev/null 2>&1; then
            break
        fi
        sleep 1
    done
fi

if ! ps -p $POSITION_PID > /dev/null 2>&1; then
    echo "   ✅ Handover complete - agents in control!"
else
    echo "   ⚠️  initial_positions still running (agents may not have connected)"
    echo "      Continuing anyway..."
fi

echo ""
echo "=" "============================================================"
echo "✅ ALL SYSTEMS READY"
echo "================================================================"
echo ""
echo "🎯 Drones in formation:"
echo "   - Evader at center (0, 0, -10m)"
echo "   - 4 Pursuers in tetrahedral formation (radius 6m)"
echo ""
echo "🤖 Active processes:"
echo "   - Voronoi computation node (centralized controller)"
echo "   - 5 Drone agents (publishing setpoints to PX4)"
echo ""

if [ "$AUTO_START_GAME" = true ]; then
    echo "🚀 Auto-starting game in 5 seconds..."
    echo "   (Waiting for all nodes to be ready...)"
    sleep 5
    
    echo ""
    echo "🎮 GAME STARTED!"
    echo ""
    
    # Publish start message multiple times to ensure delivery
    for i in {1..3}; do
        ros2 topic pub --once /voronoi/start_game std_msgs/msg/Bool "{data: true}" 2>/dev/null
        sleep 0.5
    done
    
    echo "✅ Start signal published"
    echo ""
    echo "📊 Monitor game status:"
    echo "   ros2 topic echo /voronoi/game_status"
    echo ""
    echo "📡 Watch capture events:"
    echo "   ros2 topic echo /voronoi/capture"
    echo ""
else
    echo "To start the game manually:"
    echo "   ros2 topic pub --once /voronoi/start_game std_msgs/msg/Bool \"{data: true}\""
    echo ""
fi

echo "Press Ctrl+C to stop all processes"
echo ""
echo "================================================================"
echo ""

# Keep script running and wait for all background jobs
wait
