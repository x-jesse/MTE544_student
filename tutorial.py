### ROS 2 Tutorial (pub/sub for TurtleBot) for MTE 544
### By: Jeffrey Lee

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time

# Import the needed message types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class ExampleMobileRobotNode(Node):
    
    def __init__(self):
        super().__init__("example_mobile_robot_node")
        
        # Define a QoS Profile for the subscriber
        sub_qos_profile=QoSProfile(reliability=2, durability=2, history=1, depth=10)
        
        # Create subscription(s) to whatever topics you need
        self.create_subscription(Odometry, "/odom", self.sub_callback, qos_profile=sub_qos_profile)

        # Create a publisher for velocity control
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10)

        # Create a timer with a callback for the publisher
        timer_period = 0.1
        self.create_timer(timer_period, self.timer_callback)
    

    def sub_callback(self, msg: Odometry):
        # Get timestamp from message (timestamp is always in the message header)
        timestamp = Time.from_msg(msg.header.stamp).nanoseconds

        # Get message data
        odom_orientation = msg.pose.pose.orientation
        odom_x_pos = msg.pose.pose.position.x
        odom_y_pos = msg.pose.pose.position.y

        print(f'Message Timestamp = {timestamp}')
        print(f'Current Robot Orientation = {odom_orientation}')
        print(f'Current Robot X Position = {odom_x_pos}')
        print(f'Current Robot Y Position = {odom_y_pos}')
        
    def timer_callback(self):
        # Create Twist message
        msg=Twist()
        
        # Populate the message with desired linear and angular velocity, respectively
        msg.linear.x=0.5
        msg.angular.z = 0.7

        # Publish this message 
        self.vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    example_mobile_robot_node = ExampleMobileRobotNode()

    # Spin the node
    rclpy.spin(example_mobile_robot_node)

    # Destroy the node
    example_mobile_robot_node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()
    