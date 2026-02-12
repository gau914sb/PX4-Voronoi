#!/usr/bin/env python3
import asyncio
import sys
from mavsdk import System

NUM_DRONES = int(sys.argv[1]) if len(sys.argv) > 1 else 3

print(f"Starting Ground Station mimic for {NUM_DRONES} drones")
print(f"Python executable: {sys.executable}")

async def connect_drone(drone_id):
    """Connect to a single drone and keep the connection alive with heartbeats"""
    port = 14540 + drone_id
    drone = System()
    
    print(f"[Drone {drone_id}] Connecting to udp://:{port}...")
    await drone.connect(system_address=f"udp://:{port}")
    
    print(f"[Drone {drone_id}] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[Drone {drone_id}] Connected successfully on port {port}!")
            break
    
    # Send heartbeats to keep connection alive and prevent failsafe
    print(f"[Drone {drone_id}] Starting heartbeat loop...")
    while True:
        # Heartbeat is sent automatically by mavsdk
        # Just need to keep the connection alive
        await asyncio.sleep(1)

async def run():
    """Run ground station for all drones"""
    tasks = []
    
    # Create connection tasks for all drones
    for i in range(NUM_DRONES):
        task = asyncio.create_task(connect_drone(i))
        tasks.append(task)
    
    # Wait for all connections to stay alive
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(run())
