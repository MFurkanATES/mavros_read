import rospy 
from mavros_msgs.srv import CommandHome
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import numpy as np
from mavros_msgs.srv import CommandBool,CommandTOL
import time
from mavros_msgs.srv import SetMode
import tf
pi = np.pi
pi_2 = pi / 2.0

latitude = 0.0 
longitude = 0.0
altitude = 0.0
gps_service = 9
gps_fix_status = 9

theta = np.linspace(0, 2*np.pi, 100)
mav_pose = PoseStamped()
pub = rospy.Publisher('mavros/setpoint_attitude/attitude',PoseStamped, queue_size = 10)
timestamp = rospy.Time()



def set_mode_stabilize():	
  print("MOD ayarlaniyor")
  rospy.wait_for_service('/mavros/set_mode')
  try:
    modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    modeResponse = modeService(0,'STABILIZE')		
    rospy.loginfo(modeResponse)
  except rospy.ServiceException as e:
    print("Servis hatayla karsilasti: %s" %e)

def set_mode_guided():	
  print("MOD ayarlaniyor")
  rospy.wait_for_service('/mavros/set_mode')
  try:
    modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    modeResponse = modeService(4,'GUIDED')		
    rospy.loginfo(modeResponse)
  except rospy.ServiceException as e:
    print("Servis hatayla karsilasti: %s" %e)
 

def set_home_coordinate():
  global latitude
  global longitude
  global altitude
  print("Home noktasi ayarlaniyor")
  rospy.wait_for_service('/mavros/cmd/set_home')
  try:
    home_service = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
    home_response = home_service(latitude = latitude,longitude = longitude,altitude = altitude)
    print (home_response)
    rospy.loginfo(home_response)
  except rospy.ServiceException as e:
    print("Set_home Servisi hatayla karsilasti: %s" %e)
    


   
def arm(): 
  print("ARM izni veriliyor")
  rospy.wait_for_service('/mavros/cmd/arming')
  try:
    armService = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    armResponse = armService(True)
    print(armResponse)
    rospy.loginfo(armResponse)
  except rospy.ServiceException as e:
	  print("ARM Servisi hatayla karsilasti: %s" %e)

def take_off(alt):
  print("hedef yukseklige cikiliyor")
  rospy.wait_for_service('/mavros/cmd/takeoff')
  try:
    take_off_service = rospy.ServiceProxy('/mavros/cmd/takeoff',CommandTOL)
    take_off_response = take_off_service(altitude = alt)
    print(take_off_response)  
    rospy.loginfo(take_off_response)
  except rospy.ServiceException as e:
    print("ARM Servisi hatayla karsilasti: %s" %e)




def autopilot_gps_status(gps_status):
  global latitude 
  global longitude 
  global altitude
  global gps_service
  global gps_fix_status
  latitude = round(gps_status.latitude,7)
  longitude = round(gps_status.longitude,7) 
  altitude = round(gps_status.altitude,2)
  gps_service = gps_status.status.service
  gps_fix_status = gps_status.status.status
  print (latitude,longitude,altitude,gps_service,gps_fix_status)
  #otopilotun gpsinin lat,lon,alt ve durum bilgilerini verir
  #alt bilgisi deniz seviyesine goredir 


def autopilot():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Rate(30)
  rospy.Subscriber('mavros/global_position/global',NavSatFix,autopilot_gps_status)
  rospy.Publisher('mavros/setpoint_attitude/attitude',PoseStamped, queue_size = 10)
  order()




def follow_order(x,y,z,roll,pitch,yaw):  
  global timestamp
  mav_pose.header.stamp = timestamp
  mav_pose.pose.position.x = x
  mav_pose.pose.position.y = y
  mav_pose.pose.position.z = z
  quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw + pi_2)
  mav_pose.pose.orientation.x = quat[0]
  mav_pose.pose.orientation.y = quat[1]
  mav_pose.pose.orientation.z = quat[2]
  mav_pose.pose.orientation.w = quat[3]
  pub.publish(mav_pose)
  print(mav_pose)
  #rospy.sleep(0.1)

def circle(radius):
  R = float(radius)
  for i in range (180):
    theta = i * 2.0 * pi / 180.0
    x = 0.0 + R * np.cos(theta)
    y = 0.0 + R * np.sin(theta)
    z = 10
    follow_order(x,y,z,0,0,theta)
  

def order():
  set_home_coordinate()
  rospy.sleep(0.1)
  set_mode_stabilize()
  arm()
  rospy.sleep(0.1)
  take_off(610.0)
  rospy.sleep(7)
  #set_mode_guided()
  #rospy.sleep(2)   
  #circle(100)



while not rospy.is_shutdown(): 
  autopilot()  
  rospy.spin()
  