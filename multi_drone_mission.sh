#!/bin/bash
# Launch multiple drones concurrently with different missions
#
# Usage: ./multi_drone_mission.sh
#
# This will start 3 drones with different altitudes and hover times

echo "🚁 Starting multi-drone mission..."
echo "   Drone 1: 50m altitude, 5s hover"
echo "   Drone 2: 30m altitude, 3s hover"
echo "   Drone 3: 40m altitude, 4s hover"
echo ""

# Launch each drone in background with different parameters
python3 hybrid_offboard_control.py --instance 1 --altitude 50.0 --hover 5.0 &
DRONE1_PID=$!

python3 hybrid_offboard_control.py --instance 2 --altitude 30.0 --hover 3.0 &
DRONE2_PID=$!

python3 hybrid_offboard_control.py --instance 3 --altitude 40.0 --hover 4.0 &
DRONE3_PID=$!

echo "Launched drones with PIDs: $DRONE1_PID, $DRONE2_PID, $DRONE3_PID"
echo "Press Ctrl+C to stop all missions"
echo ""

# Wait for all background jobs
wait $DRONE1_PID $DRONE2_PID $DRONE3_PID

echo ""
echo "✅ All missions completed!"
