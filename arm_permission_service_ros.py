import rospy
from mavros_msgs.srv import CommandBool
import time


def arm(): 
  print("ARM izni veriliyor")
  rospy.wait_for_service('/mavros/cmd/arming')
  try:
	armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
	armResponse = armService(True)
	print armResponse
	rospy.loginfo(armResponse)
  except rospy.ServiceException as e:
	print("ARM Servisi hatayla karsilasti: %s" %e)


def disarm(): 
  print("DISARM ediliyor")
  rospy.wait_for_service('/mavros/cmd/arming')
  try:
	armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
	armResponse = armService(False)
	print (armResponse)
	rospy.loginfo(armResponse)
  except rospy.ServiceException as e:
	print("DISARM Servisi hatayla karsilasti: %s" %e)


disarm()
time.sleep(3)
arm()