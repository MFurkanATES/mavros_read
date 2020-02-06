import rospy
from sensor_msgs.msg import BatteryState

voltage = 0.0
current = 0.0
percentage = 0.0

def autopilot_battery_status(batt_state):
  global voltage 
  global current
  global percentage
  voltage = round(batt_state.voltage,2)
  current = round(batt_state.current,2) 
  percentage = round((batt_state.percentage*100),2)
  print (voltage,current,percentage)
  #otopilotun pil degerlerini verir


def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/battery',BatteryState,autopilot_battery_status)

while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()