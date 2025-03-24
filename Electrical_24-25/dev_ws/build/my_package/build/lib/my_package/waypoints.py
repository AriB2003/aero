# This receives telemetry data from telemetry.py,
# and then calculates waypoints with telemetry, gps, heading, and altitude data
# every time you make a change, you need to:
#  colcon build
# source install/setup.bash
# ros2 run my_package waypoints(because thats what its called in setup.py)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')
        # Initialize class variables
        self.position = None
        self.altitude = None
        
        self.position_subscriber = self.create_subscription(String, 'position', self.position_callback, 10)
        self.altitude_subscriber = self.create_subscription(String, 'altitude', self.altitude_callback, 10)
        self.waypoints_publisher = self.create_publisher(String, 'waypoints', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def position_callback(self, msg: String):
        self.position = msg.data
        self.get_logger().info('Received position: "%s"' % self.position)

    def altitude_callback(self, msg: String):
        self.altitude = msg.data
        self.get_logger().info('Received altitude: "%s"' % self.altitude)

    def timer_callback(self):
        if self.position is not None and self.altitude is not None:
            self.example_waypoint_calculator()

    def example_waypoint_calculator(self):  # Added self parameter
        # Calculate waypoints here
        waypoints_msg = String()
        waypoints_msg.data = f"{self.position} | {self.altitude}"  # Fixed string concatenation

        self.waypoints_publisher.publish(waypoints_msg)
        self.get_logger().info('Published waypoints: "%s"' % waypoints_msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()