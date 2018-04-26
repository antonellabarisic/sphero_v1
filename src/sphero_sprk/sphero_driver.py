#!/usr/bin/env python
import rospy
import sys
import math

from sphero_sprk import Sphero

from std_msgs.msg import ColorRGBA, Float32, Bool
from geometry_msgs.msg import Twist

# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
# from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SpheroNode(object):

    def __init__(self, default_update_rate=50.0):
        self.update_rate = default_update_rate
        self.sampling_divisor = int(400 / self.update_rate)

        self.is_connected = False
        self._node_name = rospy.get_name()
        self._namespace = rospy.get_namespace()
        self._address = rospy.get_param("adresa")
        self.robot = Sphero(self._address)

        self._init_pubsub()
        self._init_params()
        self.cmd_heading = 0
        self.cmd_speed = 0
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_diagnostics_time = rospy.Time.now()
        self.power_state_msg = "No Battery Info"
        self.power_state = 0

    def _init_pubsub(self):
        self.heading_sub = rospy.Subscriber('set_heading_', Float32, self.set_heading, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel_', Twist, self.cmd_vel, queue_size=1)
        self.color_sub = rospy.Subscriber('set_color_', ColorRGBA, self.set_color, queue_size=1)
        self.stabilization_sub = rospy.Subscriber(
            'disable_stabilization_', Bool, self.set_stabilization, queue_size=1)
        self.angular_velocity_sub = rospy.Subscriber(
            'set_angular_velocity_', Float32, self.set_angular_velocity, queue_size=1)

    def _init_params(self):
        self.connect_color_red = rospy.get_param('~connect_red', 0)
        self.connect_color_blue = rospy.get_param('~connect_blue', 0)
        self.connect_color_green = rospy.get_param('~connect_green', 255)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))

        # program
    def start(self):
        try:
            self.is_connected = self.robot.connect()
            rospy.loginfo("Connected to Sphero %s" % self._namespace +
                          "\n with address: %s" % self._address)
        except:
            rospy.logerr("Failed to connect to Sphero.")
            sys.exit(1)

        self.robot.set_rgb_led(self.connect_color_red, self.connect_color_green,
                               self.connect_color_blue, 0, False)
        self.robot.start()

    def spin(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
                if self.cmd_heading != 0 or self.cmd_speed != 0:
                    self.cmd_heading = 0
                    self.cmd_speed = 0
                    self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)
            r.sleep()

    def stop(self):
        # tell the ball to stop moving before quiting
        self.robot.roll(int(0), int(0), 1, False)
        self.robot.shutdown = True
        rospy.sleep(1.0)
        self.is_connected = self.robot.disconnect()
        self.robot.join()

    # commands
    def cmd_vel(self, msg):
        if self.is_connected:
            self.last_cmd_vel_time = rospy.Time.now()
            self.cmd_heading = self.normalize_angle_positive(
                math.atan2(msg.linear.x, msg.linear.y)) * 180 / math.pi
            print(self.cmd_heading)
            self.cmd_speed = math.sqrt(math.pow(msg.linear.x, 2) + math.pow(msg.linear.y, 2))
            print(self.cmd_speed)
            self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)

    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(int(msg.r * 255), int(msg.g * 255), int(msg.b * 255), 0, False)

    def set_stabilization(self, msg):
        if self.is_connected:
            if not msg.data:
                self.robot.set_stabilization(1, False)
            else:
                self.robot.set_stabilization(0, False)

    def set_heading(self, msg):
        if self.is_connected:
            heading_deg = int(self.normalize_angle_positive(msg.data) * 180.0 / math.pi)
            self.robot.set_heading(heading_deg, False)

    def set_angular_velocity(self, msg):
        if self.is_connected:
            rate = int((msg.data * 180 / math.pi) / 0.784)
            self.robot.set_rotation_rate(rate, False)

    def normalize_angle_positive(self, angle):
        return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi)


if __name__ == '__main__':
    rospy.init_node('sphero')

    try:
        s = SpheroNode()
    except rospy.ROSInterruptException:
        pass

    s.start()
    s.spin()
    s.stop()
