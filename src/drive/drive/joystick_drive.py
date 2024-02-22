from math import radians
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from interfaces.msg import LiveTune
from interfaces.msg import SixFloats
from interfaces.msg import PIDmsg

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

class joystickDrive(Node):
    def __init__(self):
        super().__init__("test")
        self.twist = Twist()
        self.pidMode = 0
        self.statePub = self.create_publisher(
            Int8, "cmd_drive_state", 10)

        self.setTwistPub = self.create_publisher(
            Twist, "/cmd_vel", 1)
        
        self.cmd_move_subscriber = self.create_subscription(
            Joy,"/joy", self.cmd_joy_callback, 10) 
        
    def cmd_joy_callback(self, msg: Joy):
        if(self.pidMode == 0):
            if(msg.buttons[0] == 1):
                self.pidMode = 1
                self.statePub.publish(Int8(data=self.pidMode))
            if(msg.axes[1] == 0.0 and msg.axes[0] == 0.0):
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.setTwistPub.publish(self.twist)
            else:
                self.twist.linear.x = map_range(msg.axes[1], -1, 1, -6, 6) 
                self.twist.angular.z = map_range(-msg.axes[0], -1, 1, -12, 12)
                self.setTwistPub.publish(self.twist)



def main(args=None):
    rclpy.init(args=args)
    node = joystickDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
