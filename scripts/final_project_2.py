#!/usr/bin/env python
# BEGIN ALL

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
import tf2_ros
import tf_conversions
import threading


def qr_message_callback(qr_message):
	final_data = qr_message.data.replace("\r\n","=").split("=")
	if final_data[0] is not '':
		current_qr_x = final_data[1]
		current_qr_y = final_data[3]
		next_qr_x = final_data[5]
		next_qr_y = final_data[7]
		qr_number = final_data[9]
		qr_letter = final_data[11]
	#print(current_qr_x, current_qr_y, next_qr_x, next_qr_y)
		return current_qr_x, current_qr_y, next_qr_x, next_qr_y, qr_number, qr_letter
	pass

#def qr_status_callback(qr_status):
#	pass

def qr_position_callback(qr_position):
	qr_pos = qr_position
	pass
	qr_frame_broadcaster = tf2_ros.TransformBroadcaster()
	qr_frame_msg = geometry_msgs.msg.TransformStamped()

	qr_frame_msg.header.stamp = rospy.Time.now()
	qr_frame_msg.header.frame_id = "camera_link"
	qr_frame_msg.child_frame_id = "qr_frame"
	#qr_frame_msg.transform.translation = qr_pos.pose.position
	qr_frame_msg.transform.translation.x = qr_pos.pose.position.z
	qr_frame_msg.transform.translation.y = qr_pos.pose.position.y
	qr_frame_msg.transform.translation.z = qr_pos.pose.position.x
	qr_frame_msg.transform.rotation = qr_pos.pose.orientation
	qr_frame_broadcaster.sendTransform(qr_frame_msg)
	#print(qr_pos)
	pass
'''
def lookup_position():
	while not rospy.is_shutdown():
		try:
			trans = transform_qr_pos_map_buffer.lookup_transform("map", "qr_pos_camera_frame", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue
		print(trans)
	pass
'''
'''def qr_broadcast():
	qr_frame_broadcaster = tf2_ros.TransformBroadcaster()
	qr_frame_msg = geometry_msgs.msg.TransformStamped()

	qr_frame_msg.header.stamp = rospy.Time.now()
	qr_frame_msg.header.frame_id = "camera_link"
	qr_frame_msg.child_frame_id = "qr_frame"
	qr_frame_msg.transform.translation.x = 0.0
	qr_frame_msg.transform.translation.y = 0.0
	qr_frame_msg.transform.translation.z = 0.0
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
	qr_frame_msg.transform.rotation.x = q[0]
	qr_frame_msg.transform.rotation.y = q[1]
	qr_frame_msg.transform.rotation.z = q[2]
	qr_frame_msg.transform.rotation.w = q[3]

	qr_frame_broadcaster.sendTransform(qr_frame_msg)
	#print(qr_frame_msg)
	pass
'''

def main():
	rospy.init_node("Final_test")
	rate = rospy.Rate(10)


	qr_message_sub = rospy.Subscriber('visp_auto_tracker/code_message', String, qr_message_callback)
	#qr_status_sub = rospy.Subscriber("visp_auto_tracker/status", Int8, qr_status_callback)
	qr_position_sub = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, qr_position_callback)
	#tf_thread = threading.Thread(target=qr_broadcast)
	transform_qr_pos_map_buffer = tf2_ros.Buffer()
	transform_qr_pos_map = tf2_ros.TransformListener(transform_qr_pos_map_buffer)
	#static_transform_map_qr = tf2_ros.StaticTransformBroadcaster()
	#static_transform_msg_map_qr = geometry_msgs.msg.TransformStamped()
	#qr_positions = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

	#tf_thread.start()
	while not rospy.is_shutdown():
		try:
			trans = transform_qr_pos_map_buffer.lookup_transform("map", "qr_frame", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue
		print(trans)
		#qr_number = qr_message_callback().qr_number
		#qr_positions[qr_number][trans.transform.translation.x, qr_message_sub[0], trans.transform.translation.y, qr_message_sub[1]]

	rospy.spin()

	pass




if __name__ == '__main__':
	main()