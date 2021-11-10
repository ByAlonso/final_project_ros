import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
global QR_codee
def scan_callback(msg):
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  Robot.set_range(min(tmp))
 
def status_listener(data):
  if data == 3 or data == 4:
    if final_project.Robot.exploration:
      Robot.exploration = False
      Robot.navigation = True
    else:
      Robot.QR_detected = QR_codee
  else:
      Robot.QR_detected = None

def message_listener(data):
  QR_codee = QR_code(data.data.X, data.data.Y, data.data.X_next, data.data.Y_next, data.data.N, data.data.L)

def pose_listener(data):
  #point geometry_msgs.position
  pose = data.pose
  #orientation = data.pose.orientation

class Final_Robot():
	def __init__(self):
		self.exploration = True
		self.navigation = False
		self.QR_detected = False
		self.message = {}
		self.driving = True
		self.range_ahead = 1
		self.direction = [-1,1]
		self.twist = Twist()
		self.linear_velocity = 0.4
		self.angular_velocity = 0.4
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.message_sub = rospy.Subscriber('visp_auto_tracker/code_message', String, message_listener)
		self.position_sub = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, pose_listener)
		self.status_sub = rospy.Subscriber("visp_auto_tracker/status", Int8, status_listener)

	def explore(self):
		self.driving = self.range_ahead < 0.8
		random_number = random.randint(0,1)
		if self.driving:
			self.twist.linear.x = self.linear_velocity
			self.twist.angular.z = 0.0
		else:
			self.twist.linear.x = 0.0
			self.twist.angular.z = self.angular_velocity * self.direction[random_number]
		self.cmd_vel_pub.publish(self.twist)

	def set_range(range_ahead):
		print(range_ahead)
		self.range_ahead = range_ahead

	def stop(self):
		self.twist.angular.z = 0.0
		self.twist.linear.x = 0.0
		self.cmd_vel_pub.publish(self.twist)

	def navigate(self):
		if self.QR_detected != None:
			message[self.QR_detected.number] = self.QR_detected.letter
			#plan
			#move to next
		self.QR_detected = None

	def unsubscribe(self,subscriber):
		subscriber.unregister()

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
