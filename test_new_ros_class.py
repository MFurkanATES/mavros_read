import rospy
import mav_class as UAV
import numpy as np
import math
import time

MAV = UAV.AUTOPILOT()


def order():
    MAV = UAV.AUTOPILOT()
    rospy.sleep(1)
    #MAV.set_home()
    #rospy.sleep(1)
    MAV.arm()
    time.sleep(1)
    MAV.takeoff(trgt_height=1)
    #a = time.time()
    time.sleep(6)
    #b = time.time()
    #print("time",b-a)
    R = 1.0
    for i in range (180):              
        print(i)
        theta = float(i) * 2.0 * np.pi / 180.0
        x = R * np.cos(theta)
        y = R * np.sin(theta)
        z = 1
        tangent = math.atan2(y,x)
        MAV.goto_xyz_rpy(x,y,z,0,0,(tangent + np.pi/2))
        #MAV.set_vel(x,y,0,0,0,0)
        print("to"+str(x)+str(y)+str(z))
        if i == 0:
            time.sleep(3)
        rospy.sleep(1.8)
        
    
   
    print("land")
    MAV.land()
    

if __name__=="__main__":
    order()
