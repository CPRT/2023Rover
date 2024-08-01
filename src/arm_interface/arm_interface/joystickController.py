from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Bool
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

def joystick_to_motor_control(vertical, horizontal):
    vertical = max(min(vertical, 1.0), -1.0)
    horizontal = max(min(horizontal, 1.0), -1.0)
    
    left_motor = vertical + horizontal
    right_motor = vertical - horizontal
    
    left_motor = max(min(left_motor, 1.0), -1.0)
    right_motor = max(min(right_motor, 1.0), -1.0)
    
    return -left_motor, -right_motor

class joystickController(Node):
    def __init__(self):
        super().__init__("joystickControl")
        self.base = MotorControl()
        self.diff1 = MotorControl()
        self.diff2 = MotorControl()
        self.elbow = MotorControl()
        self.wristTilt = MotorControl()
        self.wristTurn = MotorControl()
        self.estop = Bool()
        self.estopTimestamp = 0.0
        self.lastTimestamp = 0

        self.baseCommand = self.create_publisher(
            MotorControl, "/base/set", 1)
        self.diff1Command = self.create_publisher(
            MotorControl, "/diff1/set", 1)
        self.diff2Command = self.create_publisher(
            MotorControl, "/diff2/set", 1)
        self.elbowCommand = self.create_publisher(
            MotorControl, "/elbow/set", 1)
        self.wristTiltCommand = self.create_publisher(
            MotorControl, "/wristTilt/set", 1)
        self.wristTurnCommand = self.create_publisher(
            MotorControl, "/wristTurn/set", 1)
        
        self.joystick = self.create_subscription(
            Joy, "/joy", self.joy_callback, 5)
        
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.controlPublisher)



    def controlPublisher(self):
        if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2 or self.estop.data == True):
            return
        self.baseCommand.publish(self.base)
        self.diff1Command.publish(self.diff1)
        self.diff2Command.publish(self.diff2)
        self.elbowCommand.publish(self.elbow)
        self.wristTiltCommand.publish(self.wristTilt)
        self.wristTurnCommand.publish(self.wristTurn)

    def joy_callback(self, msg: Joy):
        self.lastTimestamp = msg.header.stamp.sec
        self.base.mode = 0
        self.diff1.mode = 0
        self.diff2.mode = 0
        self.elbow.mode = 0
        self.wristTilt.mode = 0
        self.wristTurn.mode = 0

        if(msg.buttons[5]):#RIGHT BUMPER IDK THE VALUE
            self.base.value = 0.5
        elif(msg.buttons[4]): #LEFT BUMPER
            self.base.value = -0.5
        else:
            self.base.value = 0.0

        if(msg.buttons[7]):#RIGHT TRIGGER IDK THE VALUE
            self.wristTurn.value = 1.0
        elif(msg.buttons[6]): #LEFT TRIGGER
            self.wristTurn.value = -1.0
        else:
            self.wristTurn.value = 0.0
        
        if(msg.buttons[1]):#A IDK THE VALUE
            self.wristTilt.value = 1.0
        elif(msg.buttons[0]): #B
            self.wristTilt.value = -1.0
        else:
            self.wristTilt.value = 0.0
        self.elbow.value = msg.axes[3] #LEFT VERTICAL
        diff1, diff2 = joystick_to_motor_control(msg.axes[0], msg.axes[1])
        # self.get_logger().info(f'diff1: {self.diff1.value}, diff2: {self.diff2.value}')
        self.diff1.value = float(diff1)
        self.diff2.value = float(diff2)
        if(msg.buttons[9]):
            self.estop.data = True
            self.estopTimestamp = msg.header.stamp.sec
        if(msg.buttons[8] and msg.header.stamp.sec - self.estopTimestamp > 2):
            self.estop.data = False


def main(args=None):
    rclpy.init(args=args)
    node = joystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
