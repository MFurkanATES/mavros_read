import rospy 
import math
import tf
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

X = 0.0
Y = 0.0
Z = 0.0
X_degree = 0.0
Y_degree = 0.0
Z_degree = 0.0
quaternion_X = 0.0
quaternion_Y = 0.0
quaternion_Z = 0.0
quaternion_W = 0.0


def autopilot_imu_orientation(imu_orientation):  
  global X
  global Y
  global Z
  global X_degree
  global Y_degree
  global Z_degree
  global quaternion_X
  global quaternion_Y
  global quaternion_Z
  global quaternion_W

  quaternion_X = imu_orientation.orientation.x
  quaternion_Y = imu_orientation.orientation.y
  quaternion_Z = imu_orientation.orientation.z
  quaternion_W = imu_orientation.orientation.w

  quaternion =(imu_orientation.orientation.x,imu_orientation.orientation.y,imu_orientation.orientation.z,imu_orientation.orientation.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)

  #radian
  X = euler[0]
  Y = euler[1]
  Z = euler[2]
  #degree
  X_degree = int(abs(math.degrees(X)))
  Y_degree = int(abs(math.degrees(Y)))
  Z_degree = int(abs(math.degrees(Z)))
  
  print(X,Y,Z,X_degree,Y_degree,Z_degree,quaternion_X,quaternion_Y,quaternion_Z,quaternion_W)

def autopilot_listener():
  rospy.init_node('kontrol_uav',anonymous=True) 
  rospy.Subscriber('mavros/imu/data',Imu,autopilot_imu_orientation)


while not rospy.is_shutdown(): 
  autopilot_listener()  
  rospy.spin()
