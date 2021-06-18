import mav_class as UAV
import rospy
import time


mav = UAV.AUTOPILOT()

while True:
    time.sleep(1)
    a = mav.pose.position.x
    print (a)