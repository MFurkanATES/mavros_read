import rospy
import mav_class as UAV
import numpy as np
import math

theta = np.linspace(0, 2*np.pi, 100)
MAV = UAV.AUTOPILOT()




def order():
    rospy.sleep(2)
    MAV.set_home()
    MAV.arm()
    MAV.takeoff(7)
    rate.sleep()
    
   
    
    
   
    

if __name__=="__main__":
    order()