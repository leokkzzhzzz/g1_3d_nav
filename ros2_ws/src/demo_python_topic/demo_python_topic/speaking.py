import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import threading
from queue import Queue
import time
import espeakng

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novels_queue = Queue()
        self.novel_subscriber = self.create_subscription(
            String, 'novel', self.novel_callback, 10)
        self.speech_thread = threading.Thread(target=self.speak_thread)
        self.speech_thread.start()

    def novel_callback(self, msg):
        self.novels_queue.put(msg.data)

    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'zh'
        while rclpy.ok():
            if self.novels_queue.qsize() > 0:
                text = self.novels_queue.get()
                self.get_logger().info(f'正在朗读 {text}')
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = NovelSubNode('novel_read')
    rclpy.spin(node)
    rclpy.shutdown()
