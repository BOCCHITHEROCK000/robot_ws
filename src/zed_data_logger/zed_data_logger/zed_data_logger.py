import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Odometry #토픽 타입
import os # 파일 경로 

class ZedDataLogger(Node):
    def __init__(self):
        super().__init__('zed_data_logger') #노드 이름
        #파일 저장 경로 설정
        self.file_path = os.path.join(os.path.expanduser("~"), "zed_velocity_data.txt")

        #선속도 각속도 초기화 
        self.latest_linear_velocity = None
        self.latest_angular_velocity = None

        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Odometry, #토픽 타입
            'zed/zed_node/odom', #zed 카메라의 odom 토픽 이름 -> 원하는 결과(선.각속도)를 얻기 위한 토픽 이름
            self.listener_callback,
            qos_profile 
        )
        #시간 조절 및 subscriber 설정 
        self.timer = self.create_timer(1.0, self.save_data_to_file)
        #저장할 파일 경로 
        self.file = open(self.file_path, 'w')

    #callback 함수 -> 받아온 값을 call 
    def listener_callback(self, msg):
        self.latest_linear_velocity = msg.twist.twist.linear
        self.latest_angular_velocity = msg.twist.twist.angular

    def save_data_to_file(self):
        #만약 이 두 값이 잘 받아지면 값을 txt에 저장
        if self.latest_linear_velocity and self.latest_angular_velocity:
            data_to_write = (
                f"linear Velocity: x = {self.latest_linear_velocity.x}, y = {self.latest_linear_velocity.y}, z = {self.latest_linear_velocity.z}\n"
                f"Angular Velocity: x = {self.latest_angular_velocity.x}, y = {self.latest_angular_velocity.y}, z = {self.latest_angular_velocity.z}\n"
                "--------------------------\n"
            )
            self.file.write(data_to_write) #위에꺼 적겠다
            self.get_logger().info('Data written to file')

    def destroy(self):
        self.file.close()
        super().destroy()

def main(args = None):
    rclpy.init(args = args)
    node = ZedDataLogger()

    try:
        rclpy.spin(node) #계속 돌아감
    except KeyboardInterrupt: #예외로 ctrl+c 누르면 꺼짐
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


    

