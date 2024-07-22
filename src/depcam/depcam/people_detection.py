import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class PersonDetection(Node):
    def __init__(self):
        super().__init__('person_detection')
        self.subscription = self.create_subscription(Image, 'camera_image', self.image_callback, 10)        
        self.publisher_d = self.create_publisher(String, 'direction', 10)
        self.publisher_p = self.create_publisher(String, 'people_detection_state', 10)
        self.bridge = CvBridge()
        self.prev_people_state = None

        self.CONFIDENCE_THRESHOLD = 0.8
        self.MIN_BBOX_WIDTH = 150
        self.GREEN = (0, 255, 0)
        self.WHITE = (255, 255, 255)

        coco128 = open('./src/depcam/coco128.txt', 'r')
        data = coco128.read()
        self.class_list = data.split('\n')
        coco128.close()

        self.model = YOLO('./src/depcam/yolov8n.pt')
        
    def image_callback(self, msg):
        try:
            color_i = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            detection = self.model(color_i)[0]

            person_detected = False
            PDS_msg = String()

            for data in detection.boxes.data.tolist():
                confidence = float(data[4])
                label = int(data[5])
                
                if confidence >= self.CONFIDENCE_THRESHOLD and self.class_list[label] == 'person':
                    xmin, ymin, xmax, ymax = map(int, data[:4])

                    bbox_width = xmax - xmin  # Calculate the bounding box width

                    if bbox_width >= self.MIN_BBOX_WIDTH:
                        stop_msg = String()
                        stop_msg.data = "S"
                        self.publisher_d.publish(stop_msg)

                        PDS_msg.data = "PD" #people ditection
                        # self.publisher_p.publish(PDS_msg)
                        person_detected = True

                    cv2.rectangle(color_i, (xmin, ymin), (xmax, ymax), self.GREEN, 2)
                    cv2.putText(color_i, self.class_list[label]+' '+str(round(confidence, 2)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, self.WHITE, 2)

            if not person_detected:
                PDS_msg.data = "NPD" #people ditection
                # self.publisher_p.publish(PDS_msg)
        
            if self.prev_people_state != PDS_msg.data:
                self.publisher_p.publish(PDS_msg)
                self.prev_people_state = PDS_msg.data

            cv2.imshow('frame', color_i)
            cv2.waitKey(1)

        except Exception as e:
            print(f"E")

def main(args=None):
    rclpy.init(args=args)
    person_detection_node = PersonDetection()
    rclpy.spin(person_detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
