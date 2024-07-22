import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class RealsenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')

        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
        self.timer = self.create_timer(0.1, self.process_image)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.found_rgb = False

        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                self.found_rgb = True
                break
        if not self.found_rgb:
            self.get_logger().error("require Depth / Color sensor")
            return

        self.config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 15)
        else:
            self.config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15)

        profile = self.pipeline.start(self.config)

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        self.get_logger().info(f"Depth Scale : {depth_scale}")

        # self.clip_dis_maxm = 1.3  # 최대 길이 범위 조정 (m)
        # self.clip_dis_max = self.clip_dis_maxm / depth_scale

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
    def process_image(self):
        try:
            fs = self.pipeline.wait_for_frames()
            align_fs = self.align.process(fs)
            align_color_f = align_fs.get_color_frame()  # 컬러 프레임을 가져옴

            if not align_color_f:  # 컬러 프레임이 없다면 반환
                return

            color_i = np.asanyarray(align_color_f.get_data())  # 컬러 이미지를 numpy 배열로 변환

            color_msg = self.bridge.cv2_to_imgmsg(color_i, encoding="bgr8")  # 컬러 이미지를 ROS 이미지 메시지로 변환
            color_msg.header.frame_id = 'camera publish'  # 프레임 ID를 설정 (원하는대로 변경)

            self.image_publisher.publish(color_msg)  # 컬러 이미지를 publish

            cv2.waitKey(1)

        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    realsense_node = RealsenseNode()
    rclpy.spin(realsense_node)
    realsense_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
