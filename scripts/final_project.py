#!/usr/bin/env python
# BEGIN ALL

import random
import rospy
import tf_conversions
import tf
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import time
import math
import numpy as np

global QR_codee
global Robot
global tfBuffer
global listener
global br

def scan_callback(msg):
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  globals()['Robot'].range_ahead = min(tmp)
 
def status_listener(data):
  if data.data == 3 or data.data == 4:
    if globals()['Robot'].exploration:
      globals()['Robot'].exploration = False
      globals()['Robot'].navigation = True
  else:
      globals()['Robot'].QR_detected = None

def message_listener(data):
  final_data = data.data.replace("\r\n","=").split("=")
  if final_data[0] is not '':
    globals()['QR_codee'] = QR_code(final_data[1],final_data[3],final_data[5], final_data[7], final_data[9], final_data[11])
    globals()['Robot'].set_QR(QR_codee)
  

def pose_listener(data):
  if data is not None and globals()['QR_codee'] is not None:
    globals()['QR_codee'].pose = data.pose
    globals()['Robot'].set_QR(QR_codee)
    

global random_number

random_number = 1
class Final_Robot():
  def __init__(self):
    self.exploration = True
    self.navigation = False
    self.QR_detected = None
    self.message = {}
    self.driving = True
    self.range_ahead = 1
    self.direction = [-1,1]
    self.random_number = 1
    self.twist = Twist()
    self.linear_velocity = 0.4
    self.angular_velocity = 0.3
    self.translation = None
    self.rotation = None
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.message_sub = rospy.Subscriber('visp_auto_tracker/code_message', String, message_listener)
    self.position_sub = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, pose_listener)
    self.status_sub = rospy.Subscriber("visp_auto_tracker/status", Int8, status_listener)
    self.scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    


  def explore(self):
    self.driving = not self.range_ahead < 1
    if self.driving:
      self.twist.linear.x = self.linear_velocity
      self.twist.angular.z = 0.0
      self.random_number = random.randint(0,1)
    else:
      self.twist.linear.x = 0.0
      self.twist.angular.z = self.angular_velocity * self.direction[self.random_number]
    self.cmd_vel_pub.publish(self.twist)

  def stop(self):
    self.twist.angular.z = 0.0
    self.twist.linear.x = 0.0
    self.cmd_vel_pub.publish(self.twist)

  def navigate(self):
    #and self.QR_detected.number not in self.message
    if self.QR_detected is not None and self.QR_detected.pose is not None and self.QR_detected.number not in self.message:
      self.stop()
      if len(self.message) == 0:
          #Conseguir los broadcasts y tal
          self.generate_frames(self.QR_detected)
      pt = PointStamped()
      pt.header.stamp = rospy.Time(0)
      pt.header.frame_id = "qr_frame"
      pt.point.x = self.QR_detected.next_x
      pt.point.y = self.QR_detected.next_y
      pt.point.z = 0.0
      pt_to_map = globals()['listener'].transformPoint("/map",pt)
      print(self.translation)
      print("...............")
      print(pt_to_map)
      '''globals()['listener'].waitForTransform("/map", "/qr_frame", rospy.Time(0), rospy.Duration(4.0))
      pt2 = globals()['listener'].transformPoint("/map",pt)

      while(pt2 == 0):
                          try:
                  
                          except:
                            print("fail setting point")
                            rospy.sleep(0.3)'''
      '''print(self.translation)
                        print("-----------------")
                        print(pt2)'''
      goal_pose = MoveBaseGoal()
      goal_pose.target_pose.header.frame_id = 'map'
      goal_pose.target_pose.pose.position.x =  pt_to_map.point.x
      goal_pose.target_pose.pose.position.y =  pt_to_map.point.y
      goal_pose.target_pose.pose.position.z =  pt_to_map.point.z
      goal_pose.target_pose.pose.orientation.x = 0.0
      goal_pose.target_pose.pose.orientation.y = 0.0
      goal_pose.target_pose.pose.orientation.z = -0.64
      goal_pose.target_pose.pose.orientation.w = -0.76
      self.client.send_goal(goal_pose)
      self.client.wait_for_result()
      rospy.sleep(3)
      
      #plan
      #move to next

  def unsubscribe(self,subscriber):
    subscriber.unregister()

  def set_QR(self,QR_codee):
    self.QR_detected = QR_codee

  def generate_frames(self,QR_code):
    frame_detected = False
    now = rospy.Time.now()
    t = TransformStamped()
    t.header.stamp = rospy.Time(0)
    t.header.frame_id = "camera_link"
    t.child_frame_id = "qr_frame"
    t.transform.translation.x = QR_code.pose.position.x
    t.transform.translation.y = QR_code.pose.position.y
    t.transform.translation.z = QR_code.pose.position.z
    t.transform.rotation.x = QR_code.pose.orientation.x
    t.transform.rotation.y = QR_code.pose.orientation.y
    t.transform.rotation.z = QR_code.pose.orientation.z
    t.transform.rotation.w = QR_code.pose.orientation.w
    globals()['br'].sendTransform((t.transform.translation.x,t.transform.translation.y,t.transform.translation.z)
      ,(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w),
      now,"qr_frame","camera_link")
    rospy.sleep(3)
    globals()['listener'].waitForTransform("/map", "/qr_frame", rospy.Time(0), rospy.Duration(4.0))
    (self.translation, self.orientation) = globals()['listener'].lookupTransform('/map', '/qr_frame', rospy.Time(0))
    '''while not frame_detected:
      try:
        globals()['listener'].waitForTransform("/map", "/qr_frame", now, rospy.Duration(4.0))
        (self.translation, self.orientation) = globals()['listener'].lookupTransform('/map', '/qr_frame', now)
        print('Frame detected!!')
        frame_detected = True
      except:
        print("No frame detected!!")
        now = rospy.Time.now()
        rospy.sleep(0.3)'''
    


class QR_code():
  def __init__(self, current_x, current_y, next_x, next_y, number, letter):
    self.current_x = float(current_x)
    self.current_y = float(current_y)
    self.next_x = float(next_x)
    self.next_y = float(next_y)
    self.number = number
    self.letter = letter
    self.pose = None
    if self.pose is not None:
      self.transform_to_map_frame()

  def set_pose(self,pose):
    self.pose = pose

rospy.init_node('final_project')
Robot = Final_Robot()
QR_codee = None
listener = tf.TransformListener()
br = tf.TransformBroadcaster()
state_change_time = rospy.Time.now() + rospy.Duration(1)
rate = rospy.Rate(60)

if __name__ == '__main__':
  while Robot.exploration:
    Robot.explore()
    rate.sleep()
  while Robot.navigation:
    Robot.navigate()
    rate.sleep()
  





# END ALL
#explore room to find first qr

#listener to topic /visp_auto_tracker/object_position -> will give pose and distance of qr code
# ->function to decode message
#listener to topic /visp_auto_tracker/code_message -> will give message
# ->function to decode message

#transform qr frame to map frame/odom frame -> tf function of ROS
#create goal position in map frame
#publish goal to move_base
#repeat for each qr
#print array of letters

#make report

#get drunk
