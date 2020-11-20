import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def show_camera():
	
  rospy.init_node('video_csi', anonymous=True)
  pub = rospy.Publisher('line', Image, queue_size=100)
  rate = rospy.Rate(30)
  bridge = CvBridge()
  cap = cv2.VideoCapture(0)
  if cap.isOpened():

    while not rospy.is_shutdown():

      ret_val, img = cap.read();
      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      image_ros = bridge.cv2_to_imgmsg(gray, encoding="mono8")
      pub.publish(image_ros)			
			
    cap.release()
    cv2.destroyAllWindows()
  else:
    print 'kamera acilamadi ! opecv_mst dosyasina bak'


if __name__ == '__main__':
  show_camera()
