import rospy 
from std_msgs.msg import UInt32

sat_num = 0

def autopilot_gps_sat(sat_status):
  global sat_num 
  sat_num = sat_status.data
  print (sat_num)
  #otopilotun kac uydu baglantisinda oldugunu veri

def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/global_position/raw/satellites',UInt32,autopilot_gps_sat)

while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()