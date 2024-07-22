import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS # 텍스트to음성 라이브러리
import os
import pygame

class GTTSNode(Node):
    def __init__(self):
        super().__init__('gtts_node')
        self.subscription = self.create_subscription(String, 'gtts_text', self.gtts_text_callback, 10)
        self.subscription  # ???
        self.declare_parameter('language', 'ko') # 언어, 한국어 설정

    def gtts_text_callback(self, msg):
        text = msg.data
        language = self.get_parameter('language').value

        # 오디오 파일 생성
        tts = gTTS(text=text, lang=language)
        audio_file = '/tmp/gtts_output.mp3'
        tts.save(audio_file)
        self.get_logger().info(f'Text: {text}')

        # 재생
        pygame.mixer.init()
        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()

        print('말하는 중...')
        while pygame.mixer.music.get_busy():
            continue
        print('말하기 끝났다.')
        self.get_input()

    def get_input(self):
        text = input('말할 말 입력 ("exit"=종료): ')
        alert1 = '/home/jisu/robotyou_ws/src/tts/sound/alert.mp3'
        alert2 = '/home/jisu/robotyou_ws/src/tts/sound/alert2.mp3'
        if text.lower() == 'exit': 
            self.get_logger().info('닫는 중...')
            rclpy.shutdown()
				# 아래가 꼭 필요할까?
        elif text.lower() == 'aruco':
            self.get_logger().info('마커 인식 테스트')
            pygame.mixer.init()
            pygame.mixer.music.load(alert1)
            pygame.mixer.music.play()
            self.get_input()
        elif text.lower() == 'obs':
            self.get_logger().info('장애물 인식 테스트')
            pygame.mixer.init()
            pygame.mixer.music.load(alert2)
            pygame.mixer.music.play()
            self.get_input()
        else:
            msg = String()
            msg.data = text
            self.gtts_text_callback(msg)

def main(args=None):
    rclpy.init(args=args)
    gtts_node = GTTSNode()
    gtts_node.get_input()
    rclpy.spin(gtts_node)
    gtts_node.destroy_node()

if __name__ == '__main__':
    main()