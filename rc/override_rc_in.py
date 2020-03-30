import rospy
from mavros_msgs.msg import OverrideRCIn

i = 1000
msg = OverrideRCIn()
pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size = 10)

def override():
	global i 
	global msg
	i=i+100	
	if (i >= 2000):
		i = 1000
		
	msg.channels[0] = i
	print (msg)
	pub.publish(msg)


def autopilot_talker():
  rospy.init_node('kontrol_uav',anonymous=True)
  rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size = 10)

while not rospy.is_shutdown(): 
  autopilot_talker()
  override()
  rospy.spin()
