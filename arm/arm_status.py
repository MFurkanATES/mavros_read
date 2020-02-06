import rospy
from mavros_msgs.msg import State

mode = 'null'
armed_status = 'null'

def autopilot_state(state):
  global armed_status
  global mode
  mode = state.mode
  if state.armed == False:
    armed_status = 'DISARMED'
  else:
    armed_status = 'ARMED'
  print (armed_status,mode)
  #otopilotun arm-disarm durumunu verir

def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/state',State,autopilot_state)

while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()