import rospy
from mavros_msgs.msg import PositionTarget

def publish():
    pub = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=1)
    data = PositionTarget()
    data.header.stamp = rospy.Time.now()
    data.header.frame_id = "test"
    data.coordinate_frame = 8
    data.type_mask =  1991
    #type mask kisminda hangi veriler kullanilmayacaksa alttaki sayilar toplanip yazilmali kod yoksa calismaz
    data.velocity.x = int(10)
    data.velocity.y = 0
    data.velocity.z = 0
    data.yaw_rate = 0.0
    pub.publish(data)
    rospy.sleep(2)
    print(data)



def publish_veri():
    rospy.init_node("test",anonymous=True)
 
   

if __name__ == "__main__":
    publish_veri()
    while not rospy.is_shutdown():       
        publish()


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
       
        
