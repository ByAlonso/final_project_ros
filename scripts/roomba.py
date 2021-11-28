#!/usr/bin/python
# -*- coding: utf-8 -*-

import random
import rospy
import tf_conversions
import tf2_ros
from std_msgs.msg import Int8, String
from geometry_msgs.msg import PoseStamped, PointStamped,TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf2_msgs.msg

global QR_code
global Robot
global final_message
rospy.init_node('final_project')
final_message = {}
transform_qr_pos_map_buffer = tf2_ros.Buffer()
transform_qr_pos_map = tf2_ros.TransformListener(transform_qr_pos_map_buffer)
state_change_time = rospy.Time.now() + rospy.Duration(1)
rate = rospy.Rate(60)

class QR_code:
	def __init__(self,current_x,current_y,next_x,next_y,number,letter):
		self.current_x = float(current_x)
		self.current_y = float(current_y)
		self.next_x = float(next_x)
		self.next_y = float(next_y)
		self.number = number
		self.letter = letter
		self.pose = None
		self.pose_in_map = None

	def set_pose(self, pose):
		self.pose = pose

	def set_frame(self, frame):
		self.frame = frame


class Final_Robot:
	def __init__(self):
		self.exploration = False
		self.navigation = True
		self.driving = True
		self.range_ahead = 1
		self.direction = [-1, 1]
		self.twist = Twist()
		self.linear_velocity = 0.4
		self.angular_velocity = 0.3
		self.current_QR = None
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=1)

	def explore(self):
		self.driving = not self.range_ahead < 1
		if self.driving:
			self.twist.linear.x = self.linear_velocity
			self.twist.angular.z = 0.0
			self.random_number = random.randint(0, 1)
		else:
			self.twist.linear.x = 0.0
			self.twist.angular.z = self.angular_velocity * self.direction[self.random_number]
			self.cmd_vel_pub.publish(self.twist)
		if self.current_QR is not None and len(globals()['final_message']) >= 2:
			self.exploration = False
			self.navigation = True

	def navigate(self):
		
		if self.current_QR is not None and self.current_QR.pose is not None and self.current_QR.number not in final_message:
			self.stop()
			'''if len(globals()['final_message']) == 0:
													try:
														transform_qr_pos_map_buffer.lookup_transform("map", "qr_frame_1", rospy.Time())
														print("success")
													except:
														print("failll")'''
				

			if len(globals()['final_message']) == 1:
				globals()['final_message'][self.current_QR.number] = self.current_QR

	def stop(self):
		self.twist.angular.z = 0.0
		self.twist.linear.x = 0.0
		self.cmd_vel_pub.publish(self.twist)

	def set_QR(self,qr_code):
		self.current_QR = qr_code


def scan_callback(msg):
	tmp = [msg.ranges[0]]
	for i in range(1, 21):
		tmp.append(msg.ranges[i])
	for i in range(len(msg.ranges) - 21, len(msg.ranges)):
		tmp.append(msg.ranges[i])
	globals()['Robot'].range_ahead = min(tmp)

current_QR_code = None

def message_listener(data):
	final_data = data.data.replace('\r\n', '=').split('=')
	if final_data[0] is not '':
		globals()['current_QR_code'] = QR_code(final_data[1],final_data[3],final_data[5],final_data[7],final_data[9],final_data[11])
		globals()['Robot'].set_QR(globals()['current_QR_code'])
		

def generate_frame(parent_frame,child_frame,current_code):	  
		qr_frame_msg = TransformStamped()
		qr_frame_msg.header.stamp = rospy.Time.now()
		qr_frame_msg.header.frame_id = parent_frame
		qr_frame_msg.child_frame_id = child_frame
		qr_frame_msg.transform.translation.x = current_code.pose.position.z
		qr_frame_msg.transform.translation.y = current_code.pose.position.y
		qr_frame_msg.transform.translation.z = current_code.pose.position.x
		qr_frame_msg.transform.rotation = current_code.pose.orientation
		globals()['qr_frame_broadcaster'].sendTransform(qr_frame_msg)


def pose_listener(data):
	if data is not None and globals()['current_QR_code'] is not None and globals()['current_QR_code'].pose is None:
		globals()['current_QR_code'].pose = data.pose
		globals()['Robot'].set_QR(globals()['current_QR_code'])
		if len(globals()['final_message']) < 2:
			if globals()['current_QR_code'].number not in globals()['final_message']:
					try:
						generate_frame("camera_link","qr_frame_" + globals()['current_QR_code'].number,globals()['current_QR_code'])
						globals()['current_QR_code'].pose_in_map  = transform_qr_pos_map_buffer.lookup_transform("map", "qr_frame_" + globals()['current_QR_code'].number, rospy.Time())
						globals()['final_message'][globals()['current_QR_code'].number] = globals()['current_QR_code']
						success = True
						print("Success creating frame and transformation")
					except:
						print("Error creating frame and transformation")
		else:
			print("I already have 2 frames in position")
			for key in globals()['final_message']:
				print(key,globals()['final_message'][key].pose_in_map)
			rospy.sleep(3)

		
			'''for key in globals()['final_message']:
													generate_frame("camera_link","qr_frame_" + key,globals()['final_message'][key])
													globals()['current_QR_code'].pose_in_map  = transform_qr_pos_map_buffer.lookup_transform("map", "qr_frame_" + globals()['current_QR_code'].number, rospy.Time())
													print(globals()['current_QR_code'].pose_in_map)
											else:
												#Here we have to generate the hidden_frame to then operate	
												pass	'''



message_sub = rospy.Subscriber('visp_auto_tracker/code_message',String, message_listener)
position_sub = rospy.Subscriber('visp_auto_tracker/object_position',PoseStamped, pose_listener)
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
qr_frame_broadcaster = tf2_ros.TransformBroadcaster()
Robot = Final_Robot()


if __name__ == '__main__':
	while not rospy.is_shutdown() and len(final_message) < 5:
		while Robot.exploration:
			Robot.explore()
			rate.sleep()
		while Robot.navigation:
			Robot.navigate()
			rate.sleep()
	rospy.spin()
	print("----------------------------------------")
	for key in range(0,5):
		print(final_message[key].letter)
	print("----------------------------------------")