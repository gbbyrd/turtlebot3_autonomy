import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

"""This script shows the camera feed of the turtlebot. It was used for testing
and troubleshooting.
"""
class LineFollower(object):
    def __init__(self):
        rospy.init_node("live_line_follower", anonymous=True)
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback)
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist)
        
    def camera_callback(self, msg):
        try:
            self.cv2_image = self.bridge_object.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e) 
        
    def convert_binary(self):
        hsvMin = (74, 0, 125)
        hsvMax = (114, 129, 219)
        
        # convert to hsv
        hsv_image = cv2.cvtColor(self.cv2_image, cv2.COLOR_BGR2HSV)
        
        # apply thresholds
        mask = cv2.inRange(hsv_image, hsvMin, hsvMax)
        res = cv2.bitwise_and(self.cv2_image, self.cv2_image, mask=mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        
        ret, binary = cv2.threshold(gray, 70, 255, 0)
        
        height, width = binary.shape
        cropped_binary = binary[int(9*height//10):]
        
        M = cv2.moments(cropped_binary)
        
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # put text and highlight the center
        cv2.circle(self.cv2_image, (cX, cY+int(9*height//10)), 5, (255, 255, 255), -1)
        
        cv2.imshow("Cropped Binary", cropped_binary)
        
        cv2.imshow("Binary", binary)
        cv2.imshow("Cropped Binary", cropped_binary)
        
        return cropped_binary
        
    def run(self):
        rospy.sleep(2)
        while not rospy.is_shutdown():
            # cropped_binary_img = self.convert_binary()
            cv2.imshow("Camera Image", self.cv2_image)
            cv2.waitKey(1)
        
def main():
    line_follower_object = LineFollower()
    line_follower_object.run()

if __name__=='__main__':
    main()