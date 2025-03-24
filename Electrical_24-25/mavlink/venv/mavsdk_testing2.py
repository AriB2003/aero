#!/usr/bin/env python3
import asyncio
from mavsdk import System

async def run():
    # Create the drone system and connect via UDP
    drone = System()
    await drone.connect(system_address="udp://:14550")
    
    # Wait until the drone is connected
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break


    # Get initial telemetry: altitude and heading
    # (attitude_euler() returns pitch, roll, and yaw in degrees)
    position = await drone.telemetry.position().__aiter__().__anext__()
    attitude = await drone.telemetry.attitude_euler().__aiter__().__anext__()
    print(f"Initial telemetry -> Altitude: {position.absolute_altitude_m:.2f} m, Heading: {attitude.yaw_deg:.2f}°")
    
    # Arm the drone
    print("Arming the drone...")
    await drone.action.arm()
    print("Drone armed!")
    
    # Wait a few seconds to allow telemetry to update
    await asyncio.sleep(5)
    
    # Get telemetry again after arming
    position = await drone.telemetry.position().__aiter__().__anext__()
    attitude = await drone.telemetry.attitude_euler().__aiter__().__anext__()
    print(f"Post-arm telemetry -> Altitude: {position.absolute_altitude_m:.2f} m, Heading: {attitude.yaw_deg:.2f}°")
    
    # Disarm for safety
    print("Disarming the drone...")
    await drone.action.disarm()
    print("Drone disarmed.")

if __name__ == "__main__":
    asyncio.run(run())
