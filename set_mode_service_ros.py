import rospy
from mavros_msgs.srv import SetMode

#0,'MANUAL'
#1,'CIRCLE'
#2,'STABILIZE'
#3,'TRAINING'
#4,'ACRO'
#5,'FBWA'
#6,'FBWB'
#7,'CRUISE'
#8,'AUTOTUNE'
#10,'AUTO'
#11,'RTL'
#12,'LOITER'
#14,'LAND' 
#15,'GUIDED'
#16,'INITIALISING'
#17,'QSTABILIZE'
#18,'QHOVER'
#19,'QLOITER'
#20,'QLAND'
#21,'QRTL' 



def set_mode():	
  print("MOD ayarlaniyor")
  rospy.wait_for_service('/mavros/set_mode')
  try:
    modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    modeResponse = modeService(14,'LAND')		
    rospy.loginfo(modeResponse)
  except rospy.ServiceException as e:
    print("Servis hatayla karsilasti: %s" %e)
 

