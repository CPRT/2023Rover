from math import radians
import rclpy
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Int8, Float32, Bool
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
        super().__init__("Joystick Drive Node")
        self.twist = Twist()
        self.estop = Bool()
        self.estop.data = False
        self.estopTimeout = Bool()
        self.estopTimeout.data = False
        self.lastTimestamp = 0

        self.declare_parameter("PID", 0)
        self.pidMode = (
            self.get_parameter("PID").get_parameter_value().integer_value
        )
        self.declare_parameter("PID_max_speed", 1.0)
        self.MAX_PID_SPEED = (
            self.get_parameter("PID_max_speed").get_parameter_value().double_value
        )
        self.declare_parameter("PID_max_turn", 1.0)
        self.MAX_PID_TURN = (
            self.get_parameter("PID_max_turn").get_parameter_value().double_value
        )
        self.declare_parameter("voltage_max_speed", 1.0)
        self.MAX_VOLTAGE_SPEED = (
            self.get_parameter("voltage_max_speed").get_parameter_value().double_value
        )
        self.declare_parameter("voltage_max_turn", 1.0)
        self.MAX_VOLTAGE_TURN = (
            self.get_parameter("voltage_max_turn").get_parameter_value().double_value
        )

        self.setTwistPub = self.create_publisher(
            Twist, "/drive/cmd_vel", 1)
        self.setEstop = self.create_publisher(
            Bool, "/drive/estop", 1)
        self.cmd_move_subscriber = self.create_subscription(
            Joy,"/joy", self.cmd_joy_callback, 10) 
        self.setEstop.publish(self.estop) #init as not estoped

        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.joystick_timeout_handler)

    def joystick_timeout_handler(self):
        if(rclpy.time.Time.nanoseconds - self.lastTimestamp > 500_000_000):
            self.estopTimeout.data = True
            self.setEstop.publish(self.estopTimeout)
        elif(rclpy.time.Time.nanoseconds - self.lastTimestamp <= 500_000_000 and (self.estopTimeout.data and not self.estop)):
            self.estopTimeout.data = False
            self.setEstop.publish(self.estopTimeout)

    def cmd_joy_callback(self, msg: Joy):
        self.lastTimestamp = msg.header.stamp.nanosec
        if(msg.buttons[0] == 1): #able to change buttons later on
            self.estop.data = True
            self.setEstop.publish(self.estop)
        if(msg.buttons[1] == 1): #have 2 different buttons to avoid double presses not stopping
            self.estop.data = False
            self.setEstop.publish(self.estop)

        if(self.pidMode == 1):
            self.twist.linear.x = map_range(msg.axes[1], -1, 1, -self.MAX_PID_SPEED, self.MAX_PID_SPEED) 
            self.twist.angular.z = map_range(-msg.axes[0], -1, 1, -self.MAX_PID_TURN, self.MAX_PID_TURN)
            self.setTwistPub.publish(self.twist)
        else:
            self.twist.linear.x = map_range(msg.axes[1], -1, 1, -self.MAX_VOLTAGE_SPEED, self.MAX_VOLTAGE_SPEED) 
            self.twist.angular.z = map_range(-msg.axes[0], -1, 1, -self.MAX_VOLTAGE_TURN, self.MAX_VOLTAGE_TURN)
            self.setTwistPub.publish(self.twist)

            



def main(args=None):
    rclpy.init(args=args)
    node = joystickDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
