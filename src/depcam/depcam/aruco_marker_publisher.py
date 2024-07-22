import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ArUcoMarker(Node):
    def __init__(self):
        super().__init__('aruco_marker')
        self.subscription = self.create_subscription(Image, 'camera_image', self.image_callback, 10)        
        self.publisher_id = self.create_publisher(String, 'aruco_marker_id', 10)
        self.bridge = CvBridge()
        self.prev_marker_id = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            parameters =  cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:  # Check if any markers are detected
                selected_id = ids[0][0]
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)


                if self.prev_marker_id != selected_id:
                    self.get_logger().info(f"ID: {selected_id}")

                    msg = String()
                    msg.data = str(selected_id)
                    self.publisher_id.publish(msg)

                    self.prev_marker_id = selected_id

            cv2.imshow('frame', cv_image)
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    aruco_marker = ArUcoMarker()
    rclpy.spin(aruco_marker)
    cv2.destroyAllWindows()
    aruco_marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()