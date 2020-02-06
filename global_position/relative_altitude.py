import rospy 
from std_msgs.msg import Float64

relative_altitude = 0.0

def autopilot_rel_alt_status(alt_status):
  global relative_altitude
  relative_altitude = round(alt_status.data,1)
  print (relative_altitude)
  #otopilotun bulunan irtifadan  itibaren yuksekligini verir


def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/global_position/rel_alt',Float64,autopilot_rel_alt_status)

while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()