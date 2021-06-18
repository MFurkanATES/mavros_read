import rospy
import mav_class as UAV
import numpy as np
import math
import time




MAV = UAV.AUTOPILOT()

def circle(radius):
    
    

    R = float(radius)
    for i in range (180):
        theta = i * 2.0 * np.pi / 180.0
        x = 10.0 + R * np.cos(theta)
        y = 10.0 + R * np.sin(theta)
        z = 10.0
        tangent = math.atan2(y,x)
        MAV.goto_xyz_rpy(int(x),int(y),int(z),0,0,(tangent))
        #MAV.set_vel(x,y,0,0,0,0)
        rospy.sleep(1.0)


def sin():
    t = np.arange(0.0, 6.0, 0.01)
    s = np.sin(2.0 * np.pi * t)
    for i in range(len(t) - 1):
        x = 100.0 * t[i]
        y = 100.0 * s[i]
        z = 10.0
        #h = np.sqrt((t[i+1] - t[i])**2 + (s[i+1] - s[i])**2)
        #sin = (s[i+1] - s[i]) / h
        heading = math.atan2(y,x)
        x = MAV.pose.position.x + x
        y = MAV.pose.position.y + y
        z = MAV.pose.position.z + z
        
        #MAV.set_vel(x,y,2,0,0,0)
        MAV.goto_xyz_rpy(x,y,z,0,0,heading)
        rospy.sleep(0.5)


def test():
    theta = np.linspace(0, 4 * np.pi, 100)
    z = np.linspace(0, 2*5, 100)
    r = (z**2 + 1 )
    x = (r * np.sin(theta)) / 15 
    y = (r * np.cos(theta)) / 15
    
    for i in range (len(x)-1):
        x_new = x[i]
        y_new = y[i]
        z_new = z[i]
        print (x_new,y_new,z_new)
        MAV.goto_xyz_rpy(x_new,y_new,z_new,0,0,0)
        rospy.sleep(2)
        

def order():
    MAV = UAV.AUTOPILOT()
    #rospy.sleep(1)
    #MAV.set_home()
    rospy.sleep(1)
    MAV.arm()
    rospy.sleep(1)
    MAV.takeoff(trgt_height=1.0)
    #MAV.goto_xyz_rpy(0,0,1.5,0,0,0)
    a = time.time()
    time.sleep(5)
    b = time.time()
    print("time",b-a)
    R = 1.0
    for i in range (180):              
        print(i)
        theta = float(i) * 2.0 * np.pi / 180.0
        x = R * np.cos(theta)
        y = R * np.sin(theta)
        z = 1.0
        tangent = math.atan2(y,x)
        MAV.goto_xyz_rpy(x,y,z,0,0,(tangent + np.pi/2))
        #MAV.set_vel(x,y,0,0,0,0)
        print("to",x,y,z)
        if i == 0:
            time.sleep(3)
        rospy.sleep(1.8)
        
    
    #sin()
    #test()
    #MAV.gps()
    print("land")
    MAV.land()
    

if __name__=="__main__":
    order()
