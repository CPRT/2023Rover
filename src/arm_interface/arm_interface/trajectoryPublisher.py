from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Int8, Float32, Bool
from interfaces.msg import SixFloats
from interfaces.srv import ArmPos
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

def elbow_rad_to_pos(rad):
    return (rad*8300*4000*13/16)/(2*3.14159)

class trajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectoryPub")
        self.expectedTraj = JointTrajectoryPoint()
        self.motorCommand = MotorControl()


        self.baseAngle = 0.0
        self.baseZeroPoint = 0.0
        self.diff1Cont = 0.0
        self.diff1Angle = 0.0
        self.diff1ZeroPoint = 0.0
        self.diff2Cont = 0.0
        self.diff2Angle = 0.0
        self.diff2ZeroPoint = 0.0
        self.elbowAngle = 0.0
        self.elbowZeroPoint = 0.0
        self.wristTurnAngle = 0.0
        self.wristTurnZeroPoint = 0.0
        self.wristTiltAngle = 0.0
        self.wristTiltZeroPoint = 0.0
        
        self.base = MotorControl()
        self.diff1 = MotorControl()
        self.diff2 = MotorControl()
        self.elbow = MotorControl()
        self.wristTilt = MotorControl()
        self.wristTurn = MotorControl()
        
        self.trajIndex = 0

        self.estop = Bool()
        self.lastTimestamp = 0

        self.setEstop = self.create_publisher(
            Bool, "/drive/estop", 1)
        
        self.anglePub = self.create_publisher(
            SixFloats, "/arm/Angle" , 1)
        
        self.traj_sub = self.create_subscription(
            JointTrajectory, "/arm_trajectory", self.cmd_traj_callback, 10) 
        self.hasTraj = False
        self.points = JointTrajectory()
        
        self.base_motor = self.create_subscription(
            MotorStatus, "/base/status", self.base_callback, 5)
        self.diff_motor1 = self.create_subscription(
            MotorStatus, "/diff1/status", self.diff1_callback, 5)
        self.diff_motor2 = self.create_subscription(
            MotorStatus, "/diff2/status", self.diff2_callback, 5)
        self.elbow_motor = self.create_subscription(
            MotorStatus, "/elbow/status", self.elbow_callback, 5)
        self.wrist_turn_motor = self.create_subscription(
            MotorStatus, "/wristTurn/status", self.wrist_turn_callback, 5)
        self.wrist_tilt_motor = self.create_subscription(
            MotorStatus, "/wristTilt/status", self.wrist_tilt_callback, 5)
        
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
        
        self.setEstop.publish(self.estop) #init as not estoped
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.anglepublisher)
        self.executeTraj = self.create_timer(period, self.executeTraj)
        self.timer2 = self.create_timer(period, self.controlPublisher)

    def base_callback(self, msg: MotorStatus):
        self.baseAngle = 0.0
    def diff1_callback(self, msg: MotorStatus):
        #self.diff1Cont = self.diff1ZeroPoint + msg.position * ((2*pi)/1000 * 1/83 * 1/100)
        self.diff1Cont = self.diff1ZeroPoint + msg.position * ((2*pi)/2000 * 1/83 * 1/100)
        self.diff1Angle = (self.diff1Cont - self.diff2Cont)
        print("bruh1" + str(self.diff1Angle))
    def diff2_callback(self, msg: MotorStatus):
        #self.diff2Cont = self.diff2ZeroPoint + msg.position * ((2*pi)/1000 * 1/83 * 1/100)
        self.diff2Cont = self.diff2ZeroPoint + msg.position * ((2*pi)/2000 * 1/83 * 1/100) #diff drive: 83 100
        self.diff2Angle = (self.diff1Cont + self.diff2Cont)
        print("bruh2" + str(self.diff2Angle))
    def elbow_callback(self, msg: MotorStatus):
        #self.elbowAngle = (self.elbowZeroPoint + msg.position * ((2*pi)/1000 * ((1/83) * (1/100) * (16/13)))) * 4
        self.elbowAngle = (self.elbowZeroPoint + msg.position * ((2*pi)/2000 * ((1/83) * (1/100) * (16/13))))
    def wrist_turn_callback(self, msg: MotorStatus):
        self.wristTurnAngle = 0.0
    def wrist_tilt_callback(self, msg: MotorStatus):
        self.wristTiltAngle = 0.0

    def anglepublisher(self):
        out = SixFloats()
        #out.m0 = self.elbowAngle 
        out.m0 = self.baseAngle 
        out.m1 = self.diff1Angle 
        out.m2 = self.diff2Angle
        out.m3 = self.elbowAngle
        self.m4 = self.wristTiltAngle
        self.m5 = self.wristTurnAngle
        self.anglePub.publish(out)
    
    def get_arm_pos_callback(self, request, response):
        #self.get_logger().info(f'Elbow: {self.elbowAngle}')
        #response.base = self.elbowAngle 
        response.base = self.baseAngle
        response.diff1 = self.diff1Angle 
        response.diff2 = self.diff2Angle
        response.elbow = self.elbowAngle
        response.wristturn = self.wristTurnAngle
        response.wristtilt = self.wristTiltAngle
        return response
    
    def controlPublisher(self):
        # if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2 or self.estop.data == True):
        #     return
        if (self.hasTraj):
          self.get_logger().info("Im publishing")
          self.baseCommand.publish(self.base)
          self.diff1Command.publish(self.diff1)
          self.diff2Command.publish(self.diff2)
          self.elbowCommand.publish(self.elbow)
          self.wristTiltCommand.publish(self.wristTilt)
          self.wristTurnCommand.publish(self.wristTurn)

    def cmd_traj_callback(self, msg: JointTrajectory):
        self.expectedTraj = msg;
        self.lastTimestamp = Node.get_clock(self).now().seconds_nanoseconds()[0]
        self.get_logger().info(f'Traj with {len(msg.points)} points received')
        if (len(msg.points) > 0):
          self.points = msg.points
          self.hasTraj = True
          self.trajIndex = 0
        
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
    
    def executeTraj(self):
        if self.hasTraj:
            #execute trajectory
            elbowTarget = self.points[self.trajIndex].positions[3];
            self.get_logger().info(f'On traj {self.trajIndex} with elbow target {elbowTarget} and current pos {self.elbowAngle}')
            self.elbow.mode = 1;
            self.elbow.value = elbow_rad_to_pos(elbowTarget)
            if (abs(self.elbowAngle - elbowTarget) < 0.05): #within 3 deg
              self.trajIndex += 1
              if (self.trajIndex >= len(self.points)):
                self.hasTraj = False
                self.trajIndex = 0
    
    def keyboard_callback(self, msg):
        if (msg == "j"):
            self.hasTraj = False
        elif (msg == "k"):
            self.hasTraj = True
            

def main(args=None):
    rclpy.init(args=args)
    node = trajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
