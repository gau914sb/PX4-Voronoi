#!/bin/bash
# Quick reference guide for Voronoi Pursuit-Evasion
# Shows status and provides quick commands

echo "╔════════════════════════════════════════════════════════╗"
echo "║   Voronoi Pursuit-Evasion - Quick Reference Guide     ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

# Check if PX4 instances are running
echo "🔍 Checking PX4 instances..."
px4_count=$(ps aux | grep -c "px4.*instance")
if [ $px4_count -gt 5 ]; then
    echo "   ✅ PX4 instances running: ~$px4_count processes detected"
else
    echo "   ❌ PX4 not detected. Start with: ./launch_multi_drones.sh"
fi

# Check MicroXRCEAgent
echo ""
echo "🔍 Checking MicroXRCEAgent..."
if pgrep -x "MicroXRCEAgent" > /dev/null; then
    echo "   ✅ MicroXRCEAgent is running"
else
    echo "   ❌ MicroXRCEAgent not running. Start with: MicroXRCEAgent udp4 -p 8888"
fi

# Check ROS2 nodes
echo ""
echo "🔍 Checking ROS2 nodes..."
if command -v ros2 &> /dev/null; then
    voronoi_node=$(ros2 node list 2>/dev/null | grep -c "voronoi_computation_node")
    agent_nodes=$(ros2 node list 2>/dev/null | grep -c "drone_agent")
    
    if [ $voronoi_node -gt 0 ]; then
        echo "   ✅ Voronoi computation node running"
    else
        echo "   ⚪ Voronoi computation node not running"
    fi
    
    if [ $agent_nodes -gt 0 ]; then
        echo "   ✅ Drone agents running: $agent_nodes found"
    else
        echo "   ⚪ Drone agents not running"
    fi
else
    echo "   ⚠️  ROS2 not in PATH"
fi

# Check for data files
echo ""
echo "📂 Data files:"
if [ -d "data" ]; then
    csv_count=$(ls data/*.csv 2>/dev/null | wc -l)
    if [ $csv_count -gt 0 ]; then
        echo "   Found $csv_count CSV file(s)"
        latest=$(ls -t data/*.csv 2>/dev/null | head -1)
        if [ -n "$latest" ]; then
            echo "   Latest: $(basename "$latest")"
        fi
    else
        echo "   No CSV files yet"
    fi
else
    echo "   data/ directory not found"
fi

echo ""
echo "════════════════════════════════════════════════════════"
echo "📋 WORKFLOW"
echo "════════════════════════════════════════════════════════"
echo ""
echo "Step 1: Setup Initial Positions"
echo "   python3 initial_positions.py"
echo ""
echo "Step 2: Launch Game"
echo "   ./launch_game.sh"
echo ""
echo "Step 3: Start Game (in another terminal)"
echo "   ros2 topic pub --once /voronoi/start_game std_msgs/msg/Bool \"{data: true}\""
echo ""
echo "Step 4: Analyze Data"
echo "   python3 merge_csv.py --auto"
echo "   python3 plot_static.py --auto"
echo "   python3 animate_3d.py --auto"
echo ""
echo "════════════════════════════════════════════════════════"
echo "🛠️  USEFUL COMMANDS"
echo "════════════════════════════════════════════════════════"
echo ""
echo "Monitor Voronoi velocity commands:"
echo "   ros2 topic echo /voronoi/cmd_vel/instance_1"
echo ""
echo "Check game status:"
echo "   ros2 topic echo /voronoi/game_status"
echo ""
echo "Monitor capture events:"
echo "   ros2 topic echo /voronoi/capture"
echo ""
echo "List all ROS2 topics:"
echo "   ros2 topic list | grep voronoi"
echo ""
echo "Kill all game processes:"
echo "   pkill -f \"voronoi_computation_node\\|drone_agent\""
echo ""
echo "════════════════════════════════════════════════════════"
echo "📖 Full documentation: README.md"
echo "════════════════════════════════════════════════════════"
echo ""
