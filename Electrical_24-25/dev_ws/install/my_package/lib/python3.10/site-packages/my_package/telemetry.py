#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
from mavsdk import System

class MyNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        self.position_publisher_ = self.create_publisher(String, 'position', 10)
        self.altitude_publisher_ = self.create_publisher(String, 'altitude', 10)
        timer_period = 0.1  # 100ms for faster updates
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.drone = System()
        self.loop = asyncio.get_event_loop()
        # Initialize drone connection
        self.loop.run_until_complete(self.setup_drone())

    async def setup_drone(self):
        print("Connecting to drone...")
        await self.drone.connect(system_address="udp://:14550")
        self.get_logger().info("Waiting for drone connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break

    async def get_telemetry(self):
        position = await self.drone.telemetry.position().__aiter__().__anext__()
        attitude = await self.drone.telemetry.attitude_euler().__aiter__().__anext__()
        return position, attitude

    def timer_callback(self):
        try:
            # Run the async code in the sync callback using the existing event loop
            position, attitude = self.loop.run_until_complete(self.get_telemetry())
            
            # Publish position
            pos_msg = String()
            pos_msg.data = f"Position: Lat={position.latitude_deg:.6f}, Lon={position.longitude_deg:.6f}"
            self.position_publisher_.publish(pos_msg)
            
            # Publish altitude
            alt_msg = String()
            alt_msg.data = f"Altitude: {position.absolute_altitude_m:.2f}m, Heading: {attitude.yaw_deg:.2f}°"
            self.altitude_publisher_.publish(alt_msg)
            
            self.get_logger().info(f'Published position and altitude data')
        
        except Exception as e:
            self.get_logger().error(f'Error getting telemetry: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
