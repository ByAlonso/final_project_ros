#from final_project import Robot
#from final_robot import QR_code
global QR_codee

def scan_callback(msg):
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  final_project.Robot.range_ahead = min(tmp)
 
def status_listener(data):
  print("a")
  if data == 3 or data == 4:
    if final_project.Robot.exploration:
      Robot.exploration = False
      Robot.navigation = True
    else:
      Robot.QR_detected = QR_codee
  else:
      Robot.QR_detected = None

def message_listener(data):
  print("b")
  QR_codee = QR_code(data.data.X, data.data.Y, data.data.X_next, data.data.Y_next, data.data.N, data.data.L)

def pose_listener(data):
  #point geometry_msgs.position
  pose = data.pose
  #orientation = data.pose.orientation