#!/usr/bin/env python
# BEGIN ALL

import random
import rospy
import tf2_ros
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

global QR_codee
global Robot
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
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.message_sub = rospy.Subscriber('visp_auto_tracker/code_message', String, message_listener)
    self.position_sub = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, pose_listener)
    self.status_sub = rospy.Subscriber("visp_auto_tracker/status", Int8, status_listener)
    self.scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)


    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    #print(self.tfBuffer.lookup_transform("base_link", 'map', rospy.Time()))
    #print(rospy.get_param('/spawn_markers_urdf/markers'))

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
    if self.QR_detected.pose is not None and self.QR_detected.number not in self.message:
      self.stop()
      print(self.QR_detected.current_x, self.QR_detected.current_y, self.QR_detected.pose)
      print(self.QR_detected.next_x, self.QR_detected.next_y)
      self.message[self.QR_detected.number] = self.QR_detected.letter
      print(self.message)
      print(self.QR_detected.current_x, self.QR_detected.number)
      #plan
      #move to next

  def unsubscribe(self,subscriber):
    subscriber.unregister()

  def set_QR(self,QR_codee):
    self.QR_detected = QR_codee

class QR_code():
  def __init__(self, current_x, current_y, next_x, next_y, number, letter):
    self.current_x = current_x
    self.current_y = current_y
    self.next_x = next_x
    self.next_y = next_y
    self.number = number
    self.letter = letter
    self.pose = None
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)


  def transform_to_map_frame(self):
    trans = self.tfBuffer.lookup_transform("map", 'base_link', rospy.Time())
    print(trans)

  def set_pose(self,pose):
    self.pose = pose

rospy.init_node('final_project')
Robot = Final_Robot()
QR_codee = None
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
