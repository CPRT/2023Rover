from distutils.log import error
from pickletools import uint8
import rclpy
from rclpy.node import Node

from math import pi

import diagnostic_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from interfaces.msg import LiveTune
from interfaces.msg import SixFloats
from interfaces.msg import PIDmsg

from .roboclaw_driver import Roboclaw
from .PidController import PidController

from std_msgs.msg import Int8, Float32

class DriveNode(Node):
    def __init__(self):
        super().__init__("drive_node")
        # self.get_logger().info("This node just says 'Hello'")

        self.ERRORS = {0x000000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x000001: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "E-Stop"),
                       0x000002: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x000004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x000008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main Voltage High"),
                       0x000010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic Voltage High"),
                       0x000020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic Voltage Low"),
                       0x000040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Driver Fault"),
                       0x000080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Driver Fault"),
                       0x000100: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Speed"),
                       0x000200: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Speed"),
                       0x000400: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Position"),
                       0x000800: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Position"),
                       0x001000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M1 Current"),
                       0x002000: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "M2 Current"),
                       0x010000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 Over Current"),
                       0x020000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 Over Current"),
                       0x040000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main Voltage High"),
                       0x080000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main Voltage Low"),
                       0x100000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x200000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x400000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "S4 Signal Triggered"),
                       0x800000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "S5 Signal Triggered"),
                       0x01000000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Speed Error Limit"),
                       0x02000000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Position Error Limit")}

        ## 
        ## Defining stuff
        ##

        # Core variables to run this node
        self.loopCalculatePID = False
        self.wheelBaseLength = 0.5
        


        self.get_logger().info("Connecting to roboclaws")

        dev_name = self.declare_parameter("~dev", "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x30A-if00").value
        baud_rate = self.declare_parameter("~baud", 115200).value

        self.addresses = self.declare_parameter("~addresses", [128,129,130]).value

        self.roboclaw = Roboclaw(dev_name, baud_rate)

        errorCode = -1
        try:
            
            errorCode = self.roboclaw.Open()
            self.get_logger().info("Connected to roboclaws")

        except Exception as e:

            self.get_logger().fatal("Could not connect to Roboclaw")
            self.get_logger().debug(e)
            rclpy.shutdown()

        if (errorCode == 0):
            self.get_logger().fatal("Could not connect to Roboclaw")
            rclpy.shutdown()

        versions = []
        pid_kP = 0
        pid_kI = 0
        pid_kD = 0
        
        for address in self.addresses:
            try:
                self.get_logger().info("checking address " + address)
                
                versions.append(self.roboclaw.ReadVersion(address))
                # self.roboclaw.SetM1VelocityPID(address, pid_kP,pid_kI,pid_kD, 1)
                # self.roboclaw.SetM2VelocityPID(address, pid_kP,pid_kI,pid_kD, 1)
            except Exception as e:
                self.get_logger().warn("Problem getting roboclaw version for " + f"[{address}]")
                # self.get_logger().warn(e)
                pass

        for version in versions:
            if not version[0]:
                self.get_logger().warn("Problem getting roboclaw version for " + f"[{address}]")
            else:
                self.get_logger().warn("driveCode: " + repr(version[1]))



        # Setup subscribers

        self.cmd_move_subscriber = self.create_subscription(
            SixFloats,"cmd_move", self.cmd_move_callback, 10) 
        
        self.cmd_drive_state = self.create_subscription(
            Int8,"cmd_drive_state", self.drive_state_callback, 10)

        # self.cmd_liveTune_subscriber = self.create_subscription(
        #     LiveTune, "drive_liveTune", self.callback_liveTune, 10)
        
        self.cmd_voltage_drive_subscriber = self.create_subscription(
            SixFloats, "cmd_voltage_drive", self.cmd_setVoltage_callback, 10)        
            
        self.cmd_twist_drive = self.create_subscription(
            Twist, "cmd_twist_drive", self.cmd_twist_callback, 10)
        
        self.pidUpdate = self.create_subscription(
            PIDmsg, "pid_update", self.pid_update_callback, 10)

        # Setup publishers
        self.angles_publisher = self.create_publisher(
            SixFloats, "drive_speed", 10)

        self.encoder_raw_publisher = self.create_publisher(
            SixFloats, "drive_encoder_raw", 10)
        self.twist_publish = self.create_publisher(
            Twist, "Twist_Publish", 10)


        # Setup timers
        self.mainLoopTimer = self.create_timer(0.25, self.runMainLoop)
        # TODO: This needs forward kinematics equations but that's lower priority
        # self.pose_publisher = self.create_publisher(
        #     PUTMSGHERE, "drive_pose", 10)



       
        # self.create_rate(1).sleep()  # This sleep thing caused problems

        # Give some info after roboclaw has initialized
        self.get_logger().debug("DriveCode: dev %s" % dev_name)
        self.get_logger().debug("DriveCode: baud %d" % baud_rate)

        strAddress = []
        for address in self.addresses:
            strAddress += [str(address)]

        self.get_logger().debug("DriveCode: addresses %s" % str(",".join(strAddress)))


    def drive_state_callback(self, state: Int8):
        self.get_logger().info("DriveNode: state change to " + str(state.data))
        if (state.data == 0):
            self.loopCalculatePID = False
        elif (state.data == 1):
            self.loopCalculatePID = True

    def cmd_move_callback(self, msg: SixFloats):
        i = 0
        self.get_logger().info("pid moving")

        for address in self.addresses:
            try:
                self.roboclaw.SpeedM1M2(address, int(msg.m0), int(msg.m1))
            except OSError as e:
                self.get_logger().warn("" + f"[{address}] SetM1/2VelocityPID OSError: {e.errno}")
                self.get_logger().debug("" + str(e))
            i += 2

    def cmd_setVoltage_callback(self, volts: SixFloats): # CHANGE POSE
        voltages = [volts.m0, volts.m1, volts.m2, volts.m3, volts.m4, volts.m5]
        self.get_logger().info("voltage moving")

        i = 0
        for address in self.addresses:

            try:
                # MainBatteryVoltage/10 to get volts
                mainBatteryVoltage = self.roboclaw.ReadMainBatteryVoltage(address)[1] / 10 # EDGECASE: mainBatteryVoltage
            except OSError as e:
                self.get_logger().warn("" + f"[{address}] roboclaw.ReadMainBatteryVoltage OSError: {e.errno}")
                self.get_logger().debug(str(e))
                return # EDGECASE_HANDLED_TEMP

            # 32767 is 100% duty cycle (15 bytes)
            dutyCycle1 = int(voltages[i] / 10 * 32767) #mainBatteryVoltage * 32767
            dutyCycle2 = int(voltages[i+1] / 10 * 32767)

            # Send the command to the roboclaw
            try:
                self.roboclaw.DutyM1M2(address, dutyCycle1, dutyCycle2)
            except OSError as e:
                self.get_logger().warn("" + f"[{address}] DutyM1M2 OSError: {e.errno}")
                self.get_logger().debug("" + str(e))
            i += 2

    def cmd_twist_callback(self, twist: Twist):
        linear = twist.linear.x
        length = self.wheelBaseLength
        angular = -twist.angular.z  
        leftWheels = (linear - angular * length / 2)
        rightWheels = (linear + angular * length / 2)
        six = SixFloats()
        six.m0 = leftWheels
        six.m1 = rightWheels
        six.m2 = leftWheels
        six.m3 = rightWheels
        six.m4 = leftWheels
        six.m5 = rightWheels
        if(self.loopCalculatePID == False):
            self.cmd_setVoltage_callback(six)
        else:
            self.cmd_move_callback(six)

    def pid_update_callback(self, pid: PIDmsg):
        pid_kP = int(pid.p)
        pid_kI = int(pid.i)
        pid_kD = int(pid.d)
        for address in self.addresses:
            try:
                self.roboclaw.SetM1VelocityPID(address, pid_kP,pid_kI,pid_kD, 8)
                self.roboclaw.SetM2VelocityPID(address, pid_kP,pid_kI,pid_kD, 8)
            except Exception as e:
                self.get_logger().warn("Problem getting roboclaw version for " + f"[{address}]")
                self.get_logger().warn(e)
                pass

    # Stops all motors until the topic receives a new value
    def stopMotors(self):
        for address in self.addresses:
            self.stopMotor(address)
    def runMainLoop(self):
        out = SixFloats()
        temp = 1
        for address in self.addresses:
            try:
                enc1 = self.roboclaw.ReadEncM1(address)
                enc2 = self.roboclaw.ReadEncM2(address)
                if(temp == 1):
                    out.m0 = float(enc1[1])
                    out.m1 = float(enc2[1])
                if(temp == 2):
                    out.m2 = float(enc1[1])
                    out.m3 = float(enc2[1])
                if(temp == 3):
                    out.m4 = float(enc1[1])
                    out.m5 = float(enc2[1])
                temp += 1
            except OSError as e:
                self.get_logger().warn("" + f"[{address}] roboclaw.ReadError OSError: {e.errno}")
                self.get_logger().debug("" + str(e))
        self.encoder_raw_publisher.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

