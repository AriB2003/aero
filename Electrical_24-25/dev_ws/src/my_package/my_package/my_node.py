import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# This publishes a message with the topic aero_topic, 
# and this can be subscribed to by subscriber_test_function
# every time you make a change, you need to:
#  colcon build
# source install/setup.bash
# ros2 run my_package talker(talked because thats what its called in setup.py)
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(String, 'aero_topic', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = 'Hi ari can you count with me?: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
