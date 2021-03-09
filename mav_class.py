import rospy
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import State
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import VFR_HUD
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import StreamRate
from mavros_msgs.srv import CommandHome
import tf

pi_2 = 3.141592654 / 2.0

class AUTOPILOT():

    def __init__(self):
        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/global_position/global",NavSatFix,self.gps_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber('mavros/state',State,self.arm_callback)
        rospy.Subscriber('mavros/battery',BatteryState,self.battery_callback)
        rospy.Subscriber('mavros/global_position/raw/satellites',UInt32,self.sat_num_callback)
        rospy.Subscriber('mavros/global_position/rel_alt',Float64,self.rel_alt_callback)
        rospy.Subscriber('mavros/imu/data',Imu,self.imu_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)
        rospy.Subscriber('mavros/vfr_hud',VFR_HUD,self.hud_callback)

        self.gps = NavSatFix()
        self.pose = PoseStamped()
        self.arm_status = State()
        self.battery_status = BatteryState()
        self.sat_num = UInt32()
        self.rel_alt = Float64()
        self.imu = Imu()
        self.rc = RCIn()
        self.hud = VFR_HUD()
        self.timestamp = rospy.Time()

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.takeoff_service = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self.stream_service = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
        self.home_service = rospy.ServiceProxy("/mavros/cmd/set_home", CommandHome)

    def gps_callback(self,data):
        self.gps = data
        #gps headerini komple duldurur
        #alt kisimda nesne icerisinden cagirilabilir

    def pose_callback(self,data):
        self.pose = data.pose
        self.timestamp = data.header.stamp
	 
        #pose icin headerini doldurur

    def arm_callback(self,data):
        self.arm_status = data
        #arm status icin header yapisini doldurur
        #mod ve arm durumu var

    def battery_callback(self,data):
        self.battery_status = data
        #batarya durumu icin header yapisini doldurur
        #voltaj,akim,ve yuzdelik doluluk var

    def sat_num_callback(self,data):
        self.sat_num = data
        #gps uydu numarasi icin header yapisini doldurur
        #uydu sayisi var

    def rel_alt_callback(self,data):
        self.rel_alt = data
        #relative altitude( bulunan konumdan yukseklik) icin header yapisini doldurur
        #rel_alt parametresi var bulunan yukseklikten itibaren irtifa

    def imu_callback(self,data):
        self.imu = data
        #imu icin hiz ivme ve quaternion pozisyonlarini verir
        #X,Y,Z,QUATERNION_X,QUATERNION_Y,QUATERNION_Z parametreleri ve acilar var
    def rc_callback(self,data):
        self.rc = data
        #manuel rc girisleri okumak icin header icini doldurur

    def hud_callback(self,data):
        self.hud = data
        #hud verisinin header yapisini doldurur
        #heading_compass,air_speed,ground_speed, throttle parametreleri var


    def arm(self):
        return self.arm_service(True)
        #Arm servisi icin True mesajini gonderir ve arm izni verilir

    def disarm(self):
        return self.arm_service(False)
         #Arm servisi icin False mesajini gonderir ve makina disarm edilir

    def takeoff(self,trgt_height = 1.0):  
        mode_resp = self.mode_service(custom_mode="0")
        #arm edebilmek icin stabilize mod
        #mode_resp = self.mode_service(custom_mode="4")
        # mod 4 guided
        self.height = trgt_height
        self.arm()       
        mode_resp = self.mode_service(custom_mode="4")
        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=self.height)
        #return takeoff_resp
        return mode_resp

    def land(self):     
        resp = self.mode_service(custom_mode="9")
        self.disarm()
        #inis icin modu land yapar ve inis sonu disarm eder

    def stream_rate(self):
        return self.stream_service(0,10,1)
        #yayinlanmayan mavros parametrelerini kod icerisinden baslangicta cagirmak icin
        #kullanilacak servis parametreleri(degistirme) bkz mavros/wiki
    
    def set_home(self):       
        lat = self.gps.latitude
        lon = self.gps.longitude
        alt = self.gps.altitude
        return self.home_service(latitude = lat, longitude = lon, altitude = alt)

    def set_home_fake(self):
	lat = 37.23423
	lon = 27.34523
	alt = 0.0 
	return self.home_service(latitude = lat, longitude = lon, altitude = alt)
        
    

    def go_to_pose(self, pose):       
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose
        self.cmd_pos_pub.publish(pose_stamped)
        #copter guided modda olmak zorunda
        #SET_POSITION_TARGET_LOCAL_NED mesajini doldurarak yeni pose noktasini verir
        #https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED

    def goto_xyz_rpy(self, x, y, z, roll, pitch, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        #bu parametreler metre
        #quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw + pi_2)
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw )

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.go_to_pose(pose)
    
    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)
        #vektorel bileske icin 
        #http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
        #http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html
        #site aciklamasi mavros/wiki
        # This represents a vector in free space. 
        # It is only meant to represent a direction. Therefore, it does not
        # make sense to apply a translation to it (e.g., when applying a 
        # generic rigid transformation to a Vector3, tf2 will only apply the
        # rotation). If you want your data to be translatable too, use the
        # geometry_msgs/Point message instead
        




