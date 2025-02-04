import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Arming...")
    await drone.action.arm()

    print("Waiting for drone to be ready...")
    await asyncio.sleep(5)  # Wait for 5 seconds after arming

    print("Taking off...")
    drone.action.set_takeoff_altitude(3.0)
    await drone.action.takeoff()

    await asyncio.sleep(10)  # Wait for 10 seconds

    print("Landing...")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())