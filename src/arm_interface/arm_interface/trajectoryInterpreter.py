from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Int8, Float32, Bool
from interfaces.msg import SixFloats
from trajectory_msgs.msg import JointTrajectoryPoint
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value


class trajectoryInterpreter(Node):
    def __init__(self):
        super().__init__("trajectoryInt")
        self.expectedTraj = JointTrajectoryPoint()
        self.motorCommand = MotorControl()



        self.elbowAngle = 0.0
        self.elbowZeroPoint = 0.0
        self.base1Cont = 0.0
        self.base1Angle = 0.0
        self.base1ZeroPoint = 0.0
        self.base2Cont = 0.0
        self.base2Angle = 0.0
        self.base2ZeroPoint = 0.0



        self.estop = Bool()
        self.lastTimestamp = 0

        # self.declare_parameter("PID", 0)
        # self.pidMode = (
        #     self.get_parameter("PID").get_parameter_value().integer_value
        # )
        

        self.currentTrajPub = self.create_publisher(
            JointTrajectoryPoint, "/arm/currentTraj", 1)
        self.motoercontrol = self.create_publisher(
            MotorControl, "/arm/motorcontrol", 1)
        self.setEstop = self.create_publisher(
            Bool, "/drive/estop", 1)
        
        self.tempAnglepub = self.create_publisher(
            SixFloats, "/elbow/Angle" , 1)
        
        self.traj_sub = self.create_subscription(
            JointTrajectoryPoint, "/arm/expectedTraj", self.cmd_traj_callback, 10) 
        
        self.base_motor = self.create_subscription(
            MotorStatus, "/arm/base", self.base_callback, 5)
        self.diff_motor1 = self.create_subscription(
            MotorStatus, "/base1/status", self.diff1_callback, 5)
        self.diff_motor2 = self.create_subscription(
            MotorStatus, "/base2/status", self.diff2_callback, 5)
        self.elbow_motor = self.create_subscription(
            MotorStatus, "/elbow/status", self.elbow_callback, 5)
        self.wrist_vert_motor = self.create_subscription(
            MotorStatus, "/arm/wrist_vert", self.wrist_vert_callback, 5)
        
        self.setEstop.publish(self.estop) #init as not estoped
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.anglepublisher)

    def base_callback(self, msg: MotorStatus):
        msg
    def diff1_callback(self, msg: MotorStatus):
        self.base1Cont = self.base1ZeroPoint + msg.position * ((2*pi)/1000 * 1/83 * 1/100)
        self.base1Angle = (self.base1Cont - self.base2Cont) * 4
        print("bruh1" + str(self.base1Angle))
    def diff2_callback(self, msg: MotorStatus):
        self.base2Cont = self.base2ZeroPoint + msg.position * ((2*pi)/1000 * 1/83 * 1/100)
        self.base2Angle = (self.base1Cont + self.base2Cont) * 4
        print("bruh2" + str(self.base2Angle))
    def elbow_callback(self, msg: MotorStatus):
        self.elbowAngle = (self.elbowZeroPoint + msg.position * ((2*pi)/1000 * ((1/83) * (1/100) * (16/13)))) * 4
    def wrist_vert_callback(self, msg: MotorStatus):
        msg

    def anglepublisher(self):
        out = SixFloats()
        out.m0 = (self.elbowAngle) * (180/pi)
        out.m1 = (self.base1Angle) * (180/pi)
        out.m2 = (self.base2Angle) * (180/pi)
        self.tempAnglepub.publish(out)

    def cmd_traj_callback(self, msg: JointTrajectoryPoint):
        self.expectedTraj = msg;
        self.lastTimestamp = Node.get_clock(self).now().seconds_nanoseconds()[0]
    # def controlPublisher(self):
    #     self.motoercontrol.publish(self.motorCommand)

    def trajToTalon(pt: JointTrajectoryPoint):
        pt.positions

    def baseToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    def diff1ToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    def diff2ToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    def elbowToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    def wristToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    # def joystick_timeout_handler(self):
    #     if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2):
    #         self.estopTimeout.data = True
    #         self.setEstop.publish(self.estopTimeout)
    #     elif(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp <= 2 and (self.estopTimeout.data and not self.estop)):
    #         self.estopTimeout.data = False
    #         self.setEstop.publish(self.estopTimeout)


def main(args=None):
    rclpy.init(args=args)
    node = trajectoryInterpreter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
