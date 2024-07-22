import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import pygame # 오디오 재생 라이브러리
import time

class alertNode(Node):
    def __init__(self):
        super().__init__('alert_node')
        self.subscription_a = self.create_subscription(String, 'aruco_marker_id', self.alert_callback, 10)
        self.subscription_p = self.create_subscription(String, 'people_detection_state', self.alert_callback, 10)
        self.aruco_ditection_state = None
        self.last_pd_message_time = 0

        robotyou = '/home/jisu/robotyou_ws/src/tts/sound/robotyou.mp3'

        self.get_logger().info('Hi')
        pygame.mixer.init()
        pygame.mixer.music.load(robotyou)
        pygame.mixer.music.play()

    def alert_callback(self,msg):
        D313 = '/home/jisu/robotyou_ws/src/tts/sound/D313.mp3'
        D316 = '/home/jisu/robotyou_ws/src/tts/sound/D316.mp3'
        finish = '/home/jisu/robotyou_ws/src/tts/sound/finish.mp3'
        nomarker = '/home/jisu/robotyou_ws/src/tts/sound/nomarker.mp3'
        restart = '/home/jisu/robotyou_ws/src/tts/sound/restart.mp3'
        stop = '/home/jisu/robotyou_ws/src/tts/sound/stop.mp3'

        if msg.data == '0':
            self.get_logger().info('finish')
            pygame.mixer.init()
            pygame.mixer.music.load(finish) # 소리 파일
            pygame.mixer.music.play() # 재생
        elif msg.data == '1':
            self.get_logger().info('1번 마커.')
            pygame.mixer.init()
            pygame.mixer.music.load(D313)
            pygame.mixer.music.play()
            self.aruco_ditection_state = True
        elif msg.data == '2':
            self.get_logger().info('2번 마커.')
            pygame.mixer.init()
            pygame.mixer.music.load(D316)
            pygame.mixer.music.play()
            self.aruco_ditection_state = True
        elif msg.data == '3':
            self.get_logger().info('원위치.')
            pygame.mixer.init()
            pygame.mixer.music.load(restart)
            pygame.mixer.music.play()
            self.aruco_ditection_state = False
        elif msg.data == 'PD' and self.aruco_ditection_state :
            current_time = time.time()
            if current_time - self.last_pd_message_time > 3:
                self.last_pd_message_time = current_time
                self.get_logger().info('stop')
                pygame.mixer.init()
                pygame.mixer.music.load(stop)
                pygame.mixer.music.play()
        elif msg.data == 'NPD' or msg.data == 'PD':
            pass
        else:
            self.get_logger().info(f"marker_id: {msg.data}  \n -> 등록되지 않은 마커")
            pygame.mixer.init()
            pygame.mixer.music.load(nomarker)
            pygame.mixer.music.play()

def main(args=None):
    rclpy.init(args=args)
    alert_node = alertNode()
    rclpy.spin(alert_node)
    alertNode.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()