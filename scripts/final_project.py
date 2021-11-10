#!/usr/bin/env python
# BEGIN ALL

import random
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def scan_callback(msg):
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  Robot.range_ahead = min(tmp)
 
def status_listener(data):
  if data.data == 3 or data.data == 4:
    if Robot.exploration:
      Robot.exploration = False
      Robot.navigation = True
  else:
      Robot.QR_detected = None

def message_listener(data):
  final_data = data.data.replace("\r\n","=").split("=")
  if final_data:
    QR_codee = QR_code(final_data[1],final_data[3],final_data[5], final_data[7], final_data[9], final_data[11])
    Robot.set_QR(QR_codee)

def pose_listener(data):
  pass
  if data:
    #point geometry_msgs.position
    pose = data.pose
    #orientation = data.pose.orientation

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
    self.angular_velocity = 0.4
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.message_sub = rospy.Subscriber('visp_auto_tracker/code_message', String, message_listener)
    self.position_sub = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, pose_listener)
    self.status_sub = rospy.Subscriber("visp_auto_tracker/status", Int8, status_listener)
    self.scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

  def explore(self):
    self.driving = not self.range_ahead < 0.8
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
    
    if self.QR_detected != None:
      self.stop()
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

  def transform_to_map_frame(self):
    pass


Robot = Final_Robot()
QR_codee = None
rospy.init_node('final_project')
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
