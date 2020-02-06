import rospy
from mavros_msgs.msg import VFR_HUD

heading_compass = 0
air_speed = 0.0
ground_speed = 0.0
throttle = 0.0

def autopilot_hud_status(hud_status):
  global heading_compass 
  global air_speed 
  global ground_speed 
  global throttle
  heading_compass = hud_status.heading 
  air_speed = round(hud_status.airspeed,2)
  ground_speed = round(hud_status.groundspeed,2)
  throttle = round((hud_status.throttle*100),2)
  print (heading_compass,air_speed,ground_speed,throttle)
 

def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/vfr_hud',VFR_HUD,autopilot_hud_status)

while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()