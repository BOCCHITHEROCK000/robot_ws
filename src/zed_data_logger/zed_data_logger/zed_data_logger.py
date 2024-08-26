import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Odometry #토픽 타입
import os # 파일 경로 

class ZedDataLogger(Node):
    def __init__(self):
        super().__init__('zed_data_logger') #노드 이름
        #파일 저장 경로 설정
        self.file_path = os.path.join('/home/kon/robot_ws/src/zed_data_logger', "zed_pose_data.txt")

        #위치, 방향 초기화 
        self.latest_position= None
        self.latest_orientation= None

        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Odometry, #토픽 타입
            'zed/zed_node/odom', #zed 카메라의 odom 토픽 이름 -> 원하는 결과(위치,방향)를 얻기 위한 토픽 이름
            self.listener_callback,
            qos_profile 
        )
        #시간 조절 및 subscriber 설정 
        self.timer = self.create_timer(1.0, self.save_data_to_file)
        #저장할 파일 경로 
        self.file = open(self.file_path, 'w')

    #callback 함수 -> 받아온 값을 call 
    def listener_callback(self, msg):
        self.latest_position= msg.pose.pose.position
        self.latest_orientation= msg.pose.pose.orientation

    def save_data_to_file(self):
        #만약 이 두 값이 잘 받아지면 값을 txt에 저장
        if self.latest_position and self.latest_orientation:
            data_to_write = (
                f"Position: x = {self.latest_position.x}, y = {self.latest_position.y}, z = {self.latest_position.z}\n"
                f"Orientation: x = {self.latest_orientation.x}, y = {self.latest_orientation.y}, z = {self.latest_orientation.z}\n"
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


    

    

