#!/usr/bin/env python
# BEGIN ALL
import rospy
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

direction = [1,-1]
random_number = 1
def scan_callback(msg):
  global g_range_ahead
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  g_range_ahead = min(tmp)
#######################################################
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
def message_listener(data):
  final_data = data.data
  current_qr_x = final_data.X
  current_qr_y = final_data.Y
  next_qr_x = final_data.X_next
  next_qr_y = final_data.Y_next
  qr_number = final_data.N
  qr_letter = final_data.L


def pose_listener(data):
  print data

def qr_detector_listener(data):
  print data
 
message_sub = rospy.Subscriber('visp_auto_tracker/code_message', String, message_listener)
#position_sub = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, pose_listener)
qr_detector_sub = rospy.Subscriber('visp_auto_tracker/status', Int8, qr_detector_listener)
#######################################################
g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now() + rospy.Duration(1)
driving_forward = True
rate = rospy.Rate(60)
while not rospy.is_shutdown():
  continue
'''while not rospy.is_shutdown():
  #print g_range_ahead
  if g_range_ahead < 0.8:
    # TURN
    driving_forward = False
    random_number = random.randint(0,1)
    #print "Turn"
   
  else: # we're not driving_forward
    driving_forward = True # we're done spinning, time to go forward!
    #DRIVE
    #print "Drive"
   
  twist = Twist()

  if driving_forward:
    twist.linear.x = 0.4
    twist.angular.z = 0.0

  else:
    twist.linear.x = 0.0
    twist.angular.z = 0.4 * direction[random_number] 
  cmd_vel_pub.publish(twist)
 
  rate.sleep()'''
# END ALL