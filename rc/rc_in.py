import rospy
from mavros_msgs.msg import RCIn

roll_pwm = 0.0
pitch_pwm = 0.0
throttle_pwm = 0.0
yaw_pwm = 0.0
switch_1_pwm = 0.0
switch_2_pwm = 0.0
switch_3_pwm = 0.0
switch_4_pwm = 0.0
rssi = 0

def autopilot_rc_in_status(rc_data):

  global roll_pwm
  global pitch_pwm
  global throttle_pwm
  global yaw_pwm
  global switch_1_pwm
  global switch_2_pwm
  global switch_3_pwm
  global switch_4_pwm

  roll_pwm = rc_data.channels[0]
  pitch_pwm = rc_data.channels[1]
  throttle_pwm = rc_data.channels[2]
  yaw_pwm = rc_data.channels[3]
  switch_1_pwm = rc4_data.channels[4]
  switch_2_pwm = rc_data.channels[5]
  switch_3_pwm = rc_data.channels[6]
  switch_4_pwm = rc_data.channels[7]
  rssi = rc_data.rssi

  print (roll_pwm,pitch_pwm,throttle_pwm,yaw_pwm,switch_1_pwm,switch_2_pwm,switch_3_pwm,switch_4_pwm,rssi)


def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True)
  rospy.Subscriber('mavros/rc/in',RCIn,autopilot_rc_in_status)


while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()
