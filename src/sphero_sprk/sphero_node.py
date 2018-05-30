#!/usr/bin/env python
import rospy
import sys
import tf
import math
import PyKDL

from sphero_sprk import sphero_driver

from sphero_sprk.msg import name_address
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import ColorRGBA, Float32, Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class SpheroNode(object):

	battery_state = {1:"Battery Charging",
						2:"Battery OK",
						3:"Battery Low",
						4:"Battery Critical"}

	ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
							0, 1e-3, 0, 0, 0, 0,
							0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]


	ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                             0, 1e-3, 0, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e3]
	
	def __init__(self,default_update_rate=50.0):
		self.update_rate = default_update_rate
		self.sampling_divisor = int(400/self.update_rate)

		self.is_connected = False
		self._node_name=rospy.get_name()
		self._namespace = rospy.get_namespace()
		self._address = rospy.get_param(self._node_name + "/adresa")
		self.robot=sphero_driver.Sphero(self._address)

		self._init_pubsub()
		self._init_params()

		self.imu = Imu()
		self.imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
		self.imu.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
		self.imu.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]

		self.cmd_heading=0
		self.cmd_speed=0
		self.last_cmd_vel_time = rospy.Time.now()
		self.last_diagnostics_time = rospy.Time.now()
		self.power_state_msg = "No Battery Info"
		self.power_state = 0


	def _init_pubsub(self):
		self.heading_sub = rospy.Subscriber('set_heading', Float32, self.set_heading, queue_size = 1)
		self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel, queue_size = 1)
		self.color_sub = rospy.Subscriber('set_color', ColorRGBA, self.set_color, queue_size = 1)
		self.stabilization_sub = rospy.Subscriber('disable_stabilization', Bool, self.set_stabilization, queue_size = 1)
		self.angular_velocity_sub = rospy.Subscriber('set_angular_velocity', Float32, self.set_angular_velocity, queue_size = 1)
		
		self.diag_pub = rospy.Publisher('diagnostics', DiagnosticArray,queue_size=1)
		self.ble_name_pub=rospy.Publisher('ble_name',name_address,queue_size=1)
		self.odom_pub = rospy.Publisher('odom', Odometry,queue_size=1)
		self.imu_pub = rospy.Publisher('imu', Imu,queue_size=1)
		self.transform_broadcaster = tf.TransformBroadcaster()

	def _init_params(self):
		self.connect_color_red = rospy.get_param('~connect_red',0)
		self.connect_color_blue = rospy.get_param('~connect_blue',0)
		self.connect_color_green = rospy.get_param('~connect_green',255)
		self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
		self.diag_update_rate = rospy.Duration(rospy.get_param('~diag_update_rate', 1.0))


### program
	def start(self):
		try:
			self.is_connected=self.robot.connect()
			rospy.loginfo("Connected to Sphero %s" %self._namespace+ "\n with address: %s" % self._address)
		except:
			rospy.logerr("Failed to connect to Sphero.")
			sys.exit(1)

		#setup locator
		self.robot.set_locator(True)

        #setup streaming    
		self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, False) #True
		self.robot.add_async_callback(chr(0x03), self.parse_data_strm)

		#setup power notification

		self.robot.set_power_notify(True, True)
		self.robot.add_async_callback(chr(0x01), self.parse_power_notify) #chr0x01, IDCODE


		self.robot.set_rgb_led(self.connect_color_red,self.connect_color_green,self.connect_color_blue,0,False)
		self.robot.start()

	def spin(self):
		r = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			now = rospy.Time.now()
			self.get_pos_vel()
			self.get_device_name_and_address(now)

			if  (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
				if self.cmd_heading != 0 or self.cmd_speed != 0:
					self.cmd_heading = 0
					self.cmd_speed = 0
					self.robot.roll(int(self.cmd_speed), int(self.cmd_heading),1,False)
			if (now - self.last_diagnostics_time) > self.diag_update_rate:
			 	self.last_diagnostics_time = now
			 	self.publish_diagnostics(now)
			r.sleep()

	def stop(self):    
        #tell the ball to stop moving before quiting
		self.robot.roll(int(0), int(0),1, False)
		self.robot.shutdown = True
		rospy.sleep(1.0)
		self.is_connected = self.robot.disconnect()
		self.robot.join()

### commands to set
	def cmd_vel(self, msg):
		if self.is_connected:
			self.last_cmd_vel_time = rospy.Time.now()
			self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
			print(self.cmd_heading)
			self.cmd_speed = math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2))
			print(self.cmd_speed)
			self.robot.roll(int(self.cmd_speed), int(self.cmd_heading),1,False)

	def set_color(self, msg):
		if self.is_connected:
			self.robot.set_rgb_led(int(msg.r*255),int(msg.g*255),int(msg.b*255),0,False)

	def set_stabilization(self, msg):
		if self.is_connected:
			if not msg.data:
				self.robot.set_stabilization(1, False)
			else:
				self.robot.set_stabilization(0, False)

	def set_heading(self, msg):
		if self.is_connected:
			heading_deg = int(self.normalize_angle_positive(msg.data)*180.0/math.pi)
			self.robot.set_heading(heading_deg, False)

	def set_angular_velocity(self, msg):
		if self.is_connected:
			rate = int((msg.data*180/math.pi)/0.784)
			self.robot.set_rotation_rate(rate, False)

	def normalize_angle_positive(self, angle):
		return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi);

	#commands to get

	def parse_power_notify(self, data):
		if self.is_connected:
			self.power_state = data
			self.power_state_msg = self.battery_state[data]

	def parse_data_strm(self, data):
		if self.is_connected:
			now = rospy.Time.now()
			imu = Imu(header=rospy.Header(frame_id="imu_link"))
			imu.header.stamp = now
			imu.orientation.x = data["QUATERNION_Q0"]
			imu.orientation.y = data["QUATERNION_Q1"]
			imu.orientation.z = data["QUATERNION_Q2"]
			imu.orientation.w = data["QUATERNION_Q3"]
			imu.linear_acceleration.x = data["ACCEL_X_FILTERED"]/4096.0*9.8
			imu.linear_acceleration.y = data["ACCEL_Y_FILTERED"]/4096.0*9.8
			imu.linear_acceleration.z = data["ACCEL_Z_FILTERED"]/4096.0*9.8
			imu.angular_velocity.x = data["GYRO_X_FILTERED"]*10*math.pi/180
			imu.angular_velocity.y = data["GYRO_Y_FILTERED"]*10*math.pi/180
			imu.angular_velocity.z = data["GYRO_Z_FILTERED"]*10*math.pi/180

			self.imu = imu
			self.imu_pub.publish(self.imu)

	def get_device_name_and_address(self,time):
			data=self.robot.get_device_name(True)
			self._name=name_address()
			self._name.header.stamp=time
			self._name.name=data['name']
			self._name.address=data['bta']
			self.ble_name_pub.publish(self._name)

	def get_pos_vel(self):
		data=self.robot.read_locator(True)
		now = rospy.Time.now()
		odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
		odom.header.stamp = now
		odom.pose.pose = Pose(Point(data["ODOM_X"],data["ODOM_Y"],0.0), Quaternion(0.0,0.0,0.0,1.0))
		odom.twist.twist = Twist(Vector3(data["VELOCITY_X"], data["VELOCITY_Y"], 0), Vector3(0, 0, 0))

		odom.pose.covariance =self.ODOM_POSE_COVARIANCE                
		odom.twist.covariance =self.ODOM_TWIST_COVARIANCE
		self.odom_pub.publish(odom)



	def publish_diagnostics(self, time):
		self.robot.get_power_state(False)
		diag = DiagnosticArray()
		diag.header.stamp = time

		stat = DiagnosticStatus(name="Battery Status", level=DiagnosticStatus.OK, message=self.power_state_msg)
		if self.power_state == 3:
			stat.level=DiagnosticStatus.WARN
		if self.power_state == 4:
			stat.level=DiagnosticStatus.ERROR
		diag.status.append(stat)

		self.diag_pub.publish(diag)

if __name__ == '__main__':
	rospy.init_node('sphero')

	try:
		s = SpheroNode()
	except rospy.ROSInterruptException:
		pass

	s.start()
	s.spin()
	s.stop()

