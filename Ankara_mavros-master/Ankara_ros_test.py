
import rospy
import cv2
import math
import tf
import kalmanfilter_class as kalman
import pid_class as pid
import numpy as np
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import *
from sensor_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError


track_status = "CHECK RC"
armed_status = 'CHECK VEHICLE'
mode = 'NULL'
voltage = 0.0
current = 0.0
percentage = 0.0
sat_num = 0
latitude = 0.0
longitude = 0.0 
service = 9
gps_fix_status = 9
relative_altitude = 0.0
heading_compass = 0
air_speed = 0.0
ground_speed = 0.0
throttle = 0.0
flag_set_mode = 0
flag_track = 0
flag_kalman = 0 
blind_track = 0

#orientation_covariance=0.0#ek
X = 0 
Y = 0
Z = 0

pixel_yatay = 640
pixel_dikey = 480

Zt=np.zeros((2,2),dtype=float)
center_point = []
kalman_x = 0 
kalman_y = 0


#pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size = 10)
#msg = OverrideRCIn()

#ek1---------------------------------------------------
#def imu_data(imu_variables):
#  global orientation_covariance
#  orientation_covariance = round(imu_variables.orientation_covariance,2)
####-------------------------------------------------



def set_stream_rate():
	
  print("stream ayarlaniyor")
  rospy.wait_for_service('/mavros/set_stream_rate')
  try:
    stream_service = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
    stream_response = stream_service(0,10,1)
    rospy.loginfo(stream_response)
  except rospy.ServiceException as e:
    print("set_stream servisi hatayla karsilasti: %s" %e)

def set_mode():
  global flag_set_mode_back
  global flag_set_mode
  print("mode here")
  if flag_set_mode == 1 and flag_set_mode_back == 0: 
    rospy.wait_for_service('/mavros/set_mode')
    try:
      modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
      modeResponse = modeService(4,'GUIDED')		
      rospy.loginfo(modeResponse)
    except rospy.ServiceException as e:
      print("Mod servisi hatayla karsilasti: %s" %e)
  if flag_set_mode == 0 and flag_set_mode_back == 1:
    rospy.wait_for_service('/mavros/set_mode')
    try:
      modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
      modeResponse = modeService(0,'STABILIZE')		
      rospy.loginfo(modeResponse)
    except rospy.ServiceException as e:
      print("Mod servisi hatayla karsilasti: %s" %e)
  else:
    pass
    
    

        #self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

def autopilot_rcin_status(rc):
  global pilot_conf
  global track_status

  pilot_conf =  rc.channels[5]
  if pilot_conf > 1500:
    track_status = "MANUAL"
  if pilot_conf < 1500:
    track_status = "AUTO"
 
    

def autopilot_state(state):
  global armed_status
  global mode
  mode = state.mode
  if state.armed == False:
    armed_status = 'DISARMED'
  else:
    armed_status = 'ARMED'
  #print armed_status
  #otopilotun arm-disarm durumunu verir

def autopilot_battery_status(batt_state):
  global voltage 
  global current
  global percentage

  voltage = round(batt_state.voltage,2)
  current = round(batt_state.current,2) 
  percentage = round((batt_state.percentage*100),2)
  #print voltage,current,percentage
  #otopilotun pil degerlerini verir


def autopilot_gps_sat(sat_status):
  global sat_num 
  sat_num = sat_status.data
  #print sat
  #otopilotun kac uydu baglantisinda oldugunu verir

def autopilot_gps_status(gps_status):
  global latitude 
  global longitude 
  global service
  global gps_fix_status
  latitude = round(gps_status.latitude,7)
  longitude = round(gps_status.longitude,7) 
  service = gps_status.status.service
  gps_fix_status = gps_status.status.status
  #print latitude,longitude,service,gps_fix_status
  #otopilotun gpsinin lat,lon ve durum bilgilerini verir

def autopilot_rel_alt_status(alt_status):
  global relative_altitude
  relative_altitude = round(alt_status.data,1)
  #print relative_altitude
  #otopilotun bulunan irtifadan  itibaren yuksekligini verir



def autopilot_hud_status(hud_status):
  global heading_compass 
  global air_speed 
  global ground_speed 
  global throttle
  heading_compass = hud_status.heading 
  air_speed = round(hud_status.airspeed,2)
  ground_speed = round(hud_status.groundspeed,2)
  throttle = round((hud_status.throttle*100),2)
  #print heading_compass,air_speed,ground_speed,throttle
  #print (hud_status.heading)
#-------------------------------------video--------------------------------------------------------
def callback(msg):
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
  #a = time.time()
  track(cv_image)
  info_on_screen(cv_image)
  #b = time.time()
  #print(b - a)
  cv2.imshow("frame", cv_image)
  if cv2.waitKey(1) & 0xFF == ord('q'):
    rospy.loginfo("finished.")
  
 
     

#----------------------------------takip-----------------------------------------------------------
def track(image):
  #global image_data 
  #a = time.time()
  global blind_track
  global track_status
  global flag_track 
  global flag_set_mode
  global flag_set_mode_back
  global flag_kalman
  global kalman_x
  global kalman_y
  global center_point
  global Zt
  #info_on_screen(image_data)
  if track_status == "MANUAL" and flag_set_mode == 1:
    flag_set_mode_back = 1
    flag_track = 0
    flag_set_mode = 0
    set_mode()
    print("mode back")
  if track_status == "AUTO":
    if flag_set_mode == 0 and track_status == "AUTO" and flag_track == 0:
      flag_set_mode = 1
      flag_track = 1
      flag_set_mode_back = 0
      set_mode()
    if flag_set_mode == 1 and track_status == "MANUAL" :
      flag_set_mode_back = 1
      flag_track = 0
      flag_set_mode = 0
      set_mode()
      print("mode back")
     

    image_gray = np.copy(image)    
    image_gray = np.clip(image_gray,0,255)
    image_gray = np.array(image_gray,np.uint8)
    image_gray = cv2.cvtColor(image_gray, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(image_gray,200,255,cv2.THRESH_BINARY)	
    cnts=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    #print (cnts)
    if len(cnts) > 0:
      flag_track = 1
      c=max(cnts,key=cv2.contourArea)
      ((x,y),radius)=cv2.minEnclosingCircle(c)
      M=cv2.moments(c)
      if M["m00"] != 0:
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
      else:
        x,y = 0,0
      Zt[0,0] = x
      Zt[0,1] = y
      #print (x,y)
      #cv2.circle(image,(int(x),int(y)),int(radius),(0,0,255))
      enemy(int(x),int(y),int(radius),image)      
      if flag_track == 1 and flag_kalman == 0:
        
        flag_kalman = 1
        kalman_filter = kalman.KalmanFilter(Zt) 
        
      center_point = kalman_filter.KalmanUpdate(Zt)
      kalman_x = center_point[0,0]
      kalman_y = center_point[1,0]
      #takip kodu buraya
    else:
      flag_track = 0
    
    if flag_track == 0 and flag_kalman == 1 and blind_track < 100:
      global kalman_filter   
      blind_track += 1
      Zt[0,0] = kalman_x
      Zt[0,1] = kalman_y
      center_point = kalman_filter.KalmanUpdate(Zt)
      kalman_x = center_point[0,0]
      kalman_y = center_point[1,0]
      print (blind_track)
      #takip kodu buraya
      if blind_track >= 100:
        flag_kalman = 0 
        blind_track = 0
        flag_set_mode_back = 1
        flag_set_mode = 0
        del kalman_filter
      else:
        pass
  #b = time.time()
  #print(b-a)

   

#-------------------------------------ekran--------------------------------------------------------
def target_line_on_screen(image):

  cv2.line(image,((pixel_yatay/2)-10,pixel_dikey/2),((pixel_yatay/2)-30,pixel_dikey/2),(150,255,50),2)
  cv2.line(image,((pixel_yatay/2)+10,pixel_dikey/2),((pixel_yatay/2)+30,pixel_dikey/2),(150,255,50),2)
  cv2.line(image,(pixel_yatay/2,(pixel_dikey/2)-10),(pixel_yatay/2,(pixel_dikey/2)-30),(150,255,50),2)
  cv2.line(image,(pixel_yatay/2,(pixel_dikey/2)+10),(pixel_yatay/2,(pixel_dikey/2)+30),(150,255,50),2)

def autopilot_imu_orientation(imu_orientation):
  
  global X
  global Y
  global Z
  #x = imu_orientation.orientation.x
  #y = imu_orientation.orientation.y
  #z = imu_orientation.orientation.z
  #w = imu_orientation.orientation.w
  quaternion =(imu_orientation.orientation.x,imu_orientation.orientation.y,imu_orientation.orientation.z,imu_orientation.orientation.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  #roll 
  
  X = euler[0]
  Y = euler[1]
  Z = euler[2]


def draw_half_circle_rounded(image):
  global X
  # Ellipse parameters
  
  degree =int( math.degrees(X))
  #print (degree)
  radius = 100
  center = (pixel_yatay / 2, pixel_dikey /2)
  axes = (radius, radius)
  angle = 0
  startAngle = -140 + degree
  endAngle = -40 + degree
  thickness = 1
  cv2.ellipse(image, center, axes, angle, startAngle, endAngle, (150,255,50), thickness)
  degree_text_x = int((pixel_yatay/2) - ((radius + 15) * math.sin(-1 * X)))
  degree_text_y = int((pixel_dikey/2) - ((radius + 10) * math.cos(-1 * X)))
  cv2.putText(image,str(degree),(degree_text_x,degree_text_y),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

def roll_hud_screen(image):
  global X
  new_dot_yatay = int( 70 * math.cos(X))
  new_dot_dikey = int( 70 * math.sin(X))
  new_dot_yatay_hud = int( 30 * math.cos(X))
  new_dot_dikey_hud = int( 30 * math.sin(X))
  #print new_dot_yatay
  pixel_new_yatay = pixel_yatay/2 + new_dot_yatay
  pixel_new_dikey = pixel_dikey/2 + new_dot_dikey
  pixel_new_yatay_a = pixel_yatay/2 - new_dot_yatay
  pixel_new_dikey_a = pixel_dikey/2 - new_dot_dikey
  screen_angle = int(abs(math.degrees(X))) 
  cv2.line(image,(pixel_yatay/2+new_dot_yatay_hud ,pixel_dikey/2 + new_dot_dikey_hud),(pixel_new_yatay,pixel_new_dikey),(150,255,50),1)
  cv2.line(image,(pixel_yatay/2-new_dot_yatay_hud ,pixel_dikey/2 - new_dot_dikey_hud),(pixel_new_yatay_a,pixel_new_dikey_a),(150,255,50),1)
  cv2.putText(image,str(screen_angle),(pixel_new_yatay_a - 20,pixel_new_dikey_a ),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,str(screen_angle),(pixel_new_yatay + 5,pixel_new_dikey ),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

def pitch_line_on_screen(image):
  #yan cizgiler
  cv2.line(image,(210,160),(210,320),(150,255,50),1)
  cv2.line(image,(430,160),(430,320),(150,255,50),1)
  #orta cizgiler+ (math.pi/2)
  cv2.line(image,(210,240),(220,240),(150,255,50),1)
  cv2.line(image,(420,240),(430,240),(150,255,50),1)
  #uc cizgiler
  cv2.line(image,(190,320),(210,320),(150,255,50),1)
  cv2.line(image,(190,160),(210,160),(150,255,50),1)
  cv2.line(image,(430,320),(450,320),(150,255,50),1)
  cv2.line(image,(430,160),(450,160),(150,255,50),1)
  #cv2.putText(image,str(0),(195,245 ),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  #cv2.putText(image,str(0),(415,245 ),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

  
def pitch_on_screen(image):
  global Y
  screen_angle_pitch = -1 * int(math.degrees(Y))
  #print screen_angle_pitch
  set_pixel = (screen_angle_pitch * 2)  
  cv2.line(image,(440,(240 + set_pixel)),(430,(240 + set_pixel)),(150,255,50),2)
  cv2.line(image,(200,(240 + set_pixel)),(210,(240 + set_pixel)),(150,255,50),2)
  cv2.putText(image,str(screen_angle_pitch),(440,240 + set_pixel),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,str(screen_angle_pitch),(180,240 + set_pixel),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  
def status_on_screen(image):
  global mode
  global armed_status

  cv2.putText(image,("MOD:" + str(mode)),(5,20),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("IZIN:" + str(armed_status)),(5,35),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

def sat_on_screen(image):
  global sat_num
  global longitude
  global latitude

  cv2.putText(image,("GPS UYDU:" + str(sat_num)),(5,50),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("LAT:"     + str(latitude)),(5,65),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("LON:"     + str(longitude)),(5,80),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

def hud_on_screen(image):
  global air_speed
  global ground_speed
  global throttle
  global heading_compass
  global relative_altitude

  cv2.putText(image,("YER HIZI:"  + str(ground_speed)),(5,95),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("HAVA HIZI:" + str(air_speed)),(5,110),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("GAZ:%"      + str(throttle)),(5,125),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("PUSULA:"    + str(heading_compass)),(5,140),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("YUKSEKLIK:" + str(relative_altitude)),(5,155),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

def batt_on_screen(image):
  global voltage
  global percentage
  global current

  cv2.putText(image,("VOLTAJ:" + str(voltage)),(5,170),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("AKIM:"   + str(current)),(5,185),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
  cv2.putText(image,("PIL:%"   + str(percentage)),(5,200),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

#ek1----------------------------------------------
#def imu_data_on_screen(image):
#  global orientation_covariance
#  cv2.putText(image,("IMU:" + str(orientation_covariance)),(5,220),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
#----------------------------------------------------

def enemy(x,y,radius,image):
  edge_distance = int((radius + radius / 3.0) * math.sin(45))
  #print("here")
  #print(x,y,radius)
  #1-2 cizgisi  
  line_lenght = int (radius / 1.2 )
  #print edge_distance ,line_lenght ,x,y
  border_line = 2
  #cv2.circle(image,(int(x),int(y)),int(radius),(0,0,255))
  #print x_offset_back,y_offset_back,x_offset,y_offset
  #sag ust kose
  cv2.line(image,(x + edge_distance,y - edge_distance),(x + edge_distance - line_lenght,y - edge_distance),(150,255,50),border_line)
  cv2.line(image,(x + edge_distance,y - edge_distance),(x + edge_distance,y - edge_distance + line_lenght),(150,255,50),border_line)
  #sag alt kose
  cv2.line(image,(x + edge_distance,y + edge_distance),(x + edge_distance - line_lenght,y + edge_distance),(150,255,50),border_line)
  cv2.line(image,(x + edge_distance,y + edge_distance),(x + edge_distance,y + edge_distance - line_lenght),(150,255,50),border_line)
  #sol alt kose
  cv2.line(image,(x - edge_distance,y + edge_distance),(x - edge_distance + line_lenght,y + edge_distance),(150,255,50),border_line)
  cv2.line(image,(x - edge_distance,y + edge_distance),(x - edge_distance ,y + edge_distance - line_lenght),(150,255,50),border_line)
  #sol ust kose
  cv2.line(image,(x - edge_distance,y - edge_distance),(x - edge_distance + line_lenght,y - edge_distance),(150,255,50),border_line)
  cv2.line(image,(x - edge_distance,y - edge_distance),(x - edge_distance ,y - edge_distance + line_lenght),(150,255,50),border_line)

def camera_target_track(image):
  global track_status
  cv2.putText(image,("TAKIP MODU:" + track_status),(pixel_yatay-220,20),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)	
	

def info_on_screen(image):
  target_line_on_screen(image)
  status_on_screen(image)
  sat_on_screen(image)
  hud_on_screen(image)
  batt_on_screen(image)
  #roll_hud_screen(image)
  draw_half_circle_rounded(image)
  pitch_on_screen(image)
  pitch_line_on_screen(image)
  camera_target_track(image)
  #imu_data_on_screen(image)#ek---------------------------------------------------------
  

def levelfive_listener():
 #global image_data
  rospy.init_node('mikoto',anonymous=False)   
  rospy.Subscriber('line', Image, callback)
  rospy.Subscriber('mavros/vfr_hud',VFR_HUD,autopilot_hud_status)
  rospy.Subscriber('mavros/global_position/global',NavSatFix,autopilot_gps_status)
  rospy.Subscriber('mavros/battery',BatteryState,autopilot_battery_status)
  rospy.Subscriber('mavros/global_position/rel_alt',Float64,autopilot_rel_alt_status)
  rospy.Subscriber('mavros/state',State,autopilot_state)
  rospy.Subscriber('mavros/global_position/raw/satellites',UInt32,autopilot_gps_sat)
  rospy.Subscriber('mavros/imu/data',Imu,autopilot_imu_orientation)
  rospy.Subscriber('mavros/rc/in',RCIn,autopilot_rcin_status)
  #rospy.Subscriber('mavros/data',Imu,imu_data)#ek------------------------------------
 
  #track()
  rospy.spin()


set_stream_rate()

if __name__ == '__main__':
  levelfive_listener()
  #rospy.spin()
  


"""
cap = cv2.VideoCapture("a.mp4")

if (cap.isOpened()== False): 
  print("Error opening video stream or file")
 
# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  levelfive_listener()
  if ret == True:
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break 
    
    info_on_screen(frame)
    
    
 
    # Display the resulting frame
    cv2.imshow('Frame',frame)
 
    # Press Q on keyboard to  exit
    
 
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()"""

