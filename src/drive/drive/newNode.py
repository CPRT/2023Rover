#!/usr/bin/env python
from math import pi, cos, sin

import rclpy

import diagnostic_msgs
import diagnostic_updater
from . import roboclaw_driver as roboclaw
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry

from rclpy.node import Node

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width, parent_clock, parent_logger):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.clock = parent_clock
        self.logger = parent_logger
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = self.clock.now().nanoseconds
    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = self.clock.now().nanoseconds
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            self.get_logger().error("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            self.get_logger().error("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = self.clock.now().nanoseconds

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, -cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


class RoboclawNode(Node):
    def __init__(self):
        super().__init__("roboclaw_node")

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        self.get_logger().info("Connecting to roboclaw")

        self.declare_parameter("dev", "/dev/ttyACM0")
        dev_name = self.get_parameter("dev").get_parameter_value().string_value
        self.get_logger().info(dev_name)        
        
        self.declare_parameter("baud", 115200)
        baud_rate = self.get_parameter("baud").get_parameter_value().integer_value

        self.declare_parameter("address", 128)
        self.address = self.get_parameter("address").get_parameter_value().integer_value
        if self.address > 0x87 or self.address < 0x80:
            self.get_logger().fatal("Address out of range")
            self.shutdown("Address out of range")

        # TODO need someway to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            self.get_logger().fatal("Could not connect to RoboClaw")
            self.get_logger().debug(e)
            self.shutdown("Could not connect to RoboClaw")

        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            self.get_logger().warn("Problem getting roboclaw version")
            self.get_logger().debug(e)
            pass

        if not version[0]:
            self.get_logger().warn("Could not get version from roboclaw")
        else:
            self.get_logger().debug(repr(version[1]))

        roboclaw.SpeedM1M2(self.address, 0, 0)
        roboclaw.ResetEncoders(self.address)

        self.declare_parameter("max_speed", 2.0)
        self.MAX_SPEED = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )
        self.declare_parameter("ticks_per_meter", 819)
        self.TICKS_PER_METER = (
            self.get_parameter("ticks_per_meter").get_parameter_value().integer_value
        )
        self.declare_parameter("base_width", 0.9144)
        self.BASE_WIDTH = (
            self.get_parameter("base_width").get_parameter_value().double_value
        )

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH, self.get_clock(), self.get_logger())
        self.last_set_speed_time = self.get_clock().now().nanoseconds

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 1
        )

        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))

        self.get_logger().debug(f"dev {dev_name}")
        self.get_logger().debug(f"baud {baud_rate}")
        self.get_logger().debug(f"address {self.address}")
        self.get_logger().debug(f"max_speed {self.MAX_SPEED}")
        self.get_logger().debug(f"ticks_per_meter {self.TICKS_PER_METER}")
        self.get_logger().debug(f"base_width {self.BASE_WIDTH}")


    def run(self):
        self.get_logger().info("Starting motor drive")
        r_time = self.create_rate(10)
        
        while rclpy.ok():
            current_time = self.get_clock().now()
            if (current_time.nanoseconds - self.last_set_speed_time.nanoseconds) / 1e9 > 1:
                self.get_logger().info("Did not get command for 1 second, stopping")
                try:
                    self.roboclaw.ForwardM1(self.address, 0)
                    self.roboclaw.ForwardM2(self.address, 0)
                except OSError as e:
                    self.get_logger().error("Could not stop")
                    self.get_logger().debug(str(e))

            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None

            try:
                status1, enc1, crc1 = self.roboclaw.ReadEncM1(self.address)
            except ValueError:
                pass
            except OSError as e:
                self.get_logger().warn(f"ReadEncM1 OSError: {e.errno}")
                self.get_logger().debug(str(e))

            try:
                status2, enc2, crc2 = self.roboclaw.ReadEncM2(self.address)
            except ValueError:
                pass
            except OSError as e:
                self.get_logger().warn(f"ReadEncM2 OSError: {e.errno}")
                self.get_logger().debug(str(e))

            if enc1 is not None and enc2 is not None:
                self.get_logger().debug(f"Encoders {enc1} {enc2}")
                self.encodm.update_publish(enc1, enc2)

                self.updater.update()
            
            r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = self.get_clock().now().nanoseconds

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        self.get_logger().debug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if vr_ticks == 0 and vl_ticks == 0:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            else:
                roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)
        except OSError as e:
            self.get_logger().warn("SpeedM1M2 OSError: %d", e.errno)
            self.get_logger().debug(e)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: %d", e.errno)
            self.get_logger().debug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: %d", e.errno)
            self.get_logger().debug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        self.get_logger().info("Shutting down")
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        except OSError:
            self.get_logger().error("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                self.get_logger().error("Could not shutdown motors!!!!")
                self.get_logger().debug(e)


def main(args=None):
    rclpy.init(args=args)

    roboclaw_node = RoboclawNode()
    rclpy.spin(roboclaw_node)
    roboclaw_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
