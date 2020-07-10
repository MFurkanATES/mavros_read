#bu ornek buradan alinmistir
#https://edu.gaitech.hk/gapter/gapter-ros-moving-in-shapes.html

import rospy
from geometry_msgs.msg import Vector3,TwistStamped


def moveSquare():
   square_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
   square = TwistStamped()

   
   side_length = float(10)

   flag_x = 1
   flag_y = 1
   for x in range(2,6):
       if 4%x == 0:
               square.twist.linear.x = side_length
               flag_x= -1
       else:
               square.twist.linear.y = side_length
               flag_y= -1

       square_pub.publish(square)
       rospy.sleep(5)

       square.twist.linear.x=0;
       square.twist.linear.y=0;
       square_pub.publish(square);
       rospy.sleep(2);

       if flag_x == -1 and flag_y == -1:
               side_length *= -1
               flag_x = 1
               flag_y = 1

def publish_veri():
    rospy.init_node("test",anonymous=True)
   
   

if __name__ == "__main__":
    publish_veri()
    while not rospy.is_shutdown():
        moveSquare()
       


"""std_msgs/Header header

uint8 coordinate_frame
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9

uint16 type_mask
uint16 IGNORE_PX = 1 # Position ignore flags
uint16 IGNORE_PY = 2
uint16 IGNORE_PZ = 4
uint16 IGNORE_VX = 8 # Velocity vector ignore flags
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512 # Force in af vector flag
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate"""
       
        