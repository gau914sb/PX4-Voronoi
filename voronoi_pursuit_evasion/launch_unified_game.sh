#!/bin/bash
# Launch Unified Voronoi Game
# Single continuous agents handle everything: arming, positioning, and game

echo "🎮 Launching Unified Voronoi Pursuit-Evasion Game"
echo "================================================="
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
GAME_START_DELAY=45  # Seconds before auto-starting game (agents move to formation first)

# Generate single timestamp for this game run (all agents will use this)
export VORONOI_SIM_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
echo "📅 Simulation timestamp: $VORONOI_SIM_TIMESTAMP"
echo ""

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
echo "🤖 STEP 1: Launching Voronoi Computation Node"
echo ""

python3 voronoi_computation_node.py &
VORONOI_PID=$!
echo "   ✓ Voronoi computation node (PID: $VORONOI_PID)"
sleep 2

echo ""
echo "🚀 STEP 2: Launching Unified Agents"
echo "   (Each agent handles: arming → positioning → Voronoi control)"
echo ""

# Launch pursuers (instances 1-4)
for i in 1 2 3 4; do
    python3 unified_voronoi_agent.py --instance $i --role pursuer &
    AGENT_PID=$!
    echo "   ✓ Pursuer $i (PID: $AGENT_PID)"
    sleep 1
done

# Launch evader (instance 5)
python3 unified_voronoi_agent.py --instance 5 --role evader &
EVADER_PID=$!
echo "   ✓ Evader 5 (PID: $EVADER_PID)"

echo ""
echo "================================================================"
echo "✅ ALL AGENTS LAUNCHED"
echo "================================================================"
echo ""
echo "⏳ Phase 1 (0-40s): Drones arming and moving to formation"
echo "   - Evader moving to center (0, 0, -10m)"
echo "   - Pursuers moving to tetrahedral formation (radius 6m)"
echo ""
echo "🎮 Phase 2 (${GAME_START_DELAY}s+): Voronoi pursuit-evasion game"
echo "   - Auto-starting in ${GAME_START_DELAY} seconds..."
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""
echo "================================================================"

# Wait for drones to reach formation
sleep $GAME_START_DELAY

echo ""
echo "🚀 AUTO-STARTING GAME NOW!"
echo ""

# Publish start signal multiple times
for i in {1..3}; do
    ros2 topic pub --once /voronoi/start_game std_msgs/msg/Bool "{data: true}" 2>/dev/null
    sleep 0.5
done

echo "✅ Game started!"
echo ""
echo "📊 Monitor game status:"
echo "   ros2 topic echo /voronoi/game_status"
echo ""
echo "📡 Watch capture events:"
echo "   ros2 topic echo /voronoi/capture"
echo ""
echo "================================================================"
echo ""

# Keep script running
wait
