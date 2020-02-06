import rospy
from sensor_msgs.msg import Image,NavSatFix,BatteryState


latitude = 0.0 
longitude = 0.0
altitude = 0.0
gps_service = 9
gps_fix_status = 9

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


def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/global_position/global',NavSatFix,autopilot_gps_status)
  


while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()