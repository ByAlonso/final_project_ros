#!/usr/bin/env python
# BEGIN ALL
import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import int8
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

direction = [-1,1]
random_number = 1


def scan_callback(msg):
  global g_range_ahead
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  g_range_ahead = min(tmp)
 
def data_callback(data)
  if data = 3 or 4
    random_navigation = False
    status_sub.unregister()

def message_listener(data):
  

def pose_listener(data):
  #point geometry_msgs.position
  pose = data.pose
  #orientation = data.pose.orientation



g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.init_node('final_project')
state_change_time = rospy.Time.now() + rospy.Duration(1)
driving_forward = True
rate = rospy.Rate(60)

message_sub = rospy.Subscriber('visp_auto_tracker/code_message', String, message_listener)
position_sub = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, pose_listener)
status_sub = rospy.Subscriber("/visp_auto_tracker/status", std_msgs, data_callback) 

while random_navigation:
  print g_range_ahead
  if g_range_ahead < 0.8:
    # TURN
    driving_forward = False

    print "Turn"
   
  else: # we're not driving_forward
    driving_forward = True # we're done spinning, time to go forward!
    random_number = random.randint(0,1)
    #DRIVE
    print "Drive"
   
  twist = Twist()
  if driving_forward:
    twist.linear.x = 0.4
    twist.angular.z = 0.0
  else:
    twist.linear.x = 0.0
    twist.angular.z = 0.4 * direction[random_number]
  cmd_vel_pub.publish(twist)

  rate.sleep()





while not random_navigation:
  





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
