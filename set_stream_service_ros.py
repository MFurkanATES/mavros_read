import rospy
from mavros_msgs.srv import StreamRate

def set_stream_rate():
	
        print("stream ayarlaniyor")
	rospy.wait_for_service('/mavros/set_stream_rate')
	try:
		stream_service = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
		stream_response = stream_service(0,10,1)
		
		rospy.loginfo(stream_response)
	except rospy.ServiceException as e:
		print("set_stream servisi hatayla karsilasti: %s" %e)
	




