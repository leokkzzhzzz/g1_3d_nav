import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 使用标准字符串消息类型

class thefirstPublisher(Node):
    def __init__(self):
        super().__init__('thefirst_publisher')  # 节点名称：thefirst_publisher
        # 创建发布者：发布到'topic_name'话题，消息类型String，队列长度10
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # 发布频率：0.5秒一次（2Hz）
        # 创建定时器，定时调用publish_msg函数
        self.timer = self.create_timer(timer_period, self.publish_msg)
        self.i = 0  # 计数器

    def publish_msg(self):
        msg = String()
        msg.data = f'你好ros2! Count: {self.i}'  # 消息内容
        self.publisher_.publish(msg)  # 发布消息
        self.get_logger().info(f'Publishing: "{msg.data}"')  # 打印日志
        self.i += 1

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS 2
    thefirst_publisher = thefirstPublisher()  # 创建节点实例
    rclpy.spin(thefirst_publisher)  # 循环运行节点（处理回调）
    # 退出时清理
    thefirst_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

