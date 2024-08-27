import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os

class ZedDataVelocity(Node):
    def __init__(self):
        super().__init__('zed_data_velocity')
        
        # 파일 저장 경로 설정
        self.file_path = os.path.join('/home/kon/robot_ws/src/zed_data_logger', "zed_velocity_data.txt")

        # 이전 위치 및 방향 초기화
        self.prev_position = None
        self.prev_orientation = None

        # 현재 시간 초기화
        self.prev_time = self.get_clock().now()

        # 현재 속도 초기화
        self.current_linear_velocity = None
        self.current_angular_velocity = None

        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Odometry,
            'zed/zed_node/odom',
            self.listener_callback,
            qos_profile
        )

        # 시간 간격을 1초로 설정하여 데이터 저장
        self.timer = self.create_timer(1.0, self.save_velocity_to_file)  # 이 메서드가 문제였음

        # 파일 열기
        self.file = open(self.file_path, 'w')

    def listener_callback(self, msg):
        # 현재 시간 업데이트
        current_time = self.get_clock().now()

        # 이전 위치와 방향이 설정되어 있을 때만 속도 계산
        if self.prev_position is not None and self.prev_orientation is not None:
            # 시간 차이 계산
            time_diff = (current_time - self.prev_time).nanoseconds / 1e9  # 초 단위로 변환

            # 위치 변화량 계산 (x, y, z)
            delta_x = msg.pose.pose.position.x - self.prev_position.x
            delta_y = msg.pose.pose.position.y - self.prev_position.y
            delta_z = msg.pose.pose.position.z - self.prev_position.z

            # 선속도 계산 (위치 변화량 / 시간 차이)
            linear_velocity_x = delta_x / time_diff
            linear_velocity_y = delta_y / time_diff
            linear_velocity_z = delta_z / time_diff

            # 방향 변화량 계산 (x, y, z)
            delta_orientation_x = msg.pose.pose.orientation.x - self.prev_orientation.x
            delta_orientation_y = msg.pose.pose.orientation.y - self.prev_orientation.y
            delta_orientation_z = msg.pose.pose.orientation.z - self.prev_orientation.z

            # 각속도 계산 (방향 변화량 / 시간 차이)
            angular_velocity_x = delta_orientation_x / time_diff
            angular_velocity_y = delta_orientation_y / time_diff
            angular_velocity_z = delta_orientation_z / time_diff

            # 현재 속도를 업데이트
            self.current_linear_velocity = (linear_velocity_x, linear_velocity_y, linear_velocity_z)
            self.current_angular_velocity = (angular_velocity_x, angular_velocity_y, angular_velocity_z)

        # 이전 위치, 방향, 시간 업데이트
        self.prev_position = msg.pose.pose.position
        self.prev_orientation = msg.pose.pose.orientation
        self.prev_time = current_time

    def save_velocity_to_file(self):
        # 현재 속도가 존재하면 데이터를 파일에 기록
        if self.current_linear_velocity and self.current_angular_velocity:
            data_to_write = (
                f"Linear Velocity: x = {self.current_linear_velocity[0]:.2f} m/s, y = {self.current_linear_velocity[1]:.2f} m/s, z = {self.current_linear_velocity[2]:.2f} m/s\n"
                f"Angular Velocity: x = {self.current_angular_velocity[0]:.2f} rad/s, y = {self.current_angular_velocity[1]:.2f} rad/s, z = {self.current_angular_velocity[2]:.2f} rad/s\n"
                "--------------------------\n"
            )
            self.file.write(data_to_write)
            self.get_logger().info('Velocity data written to file')

    def destroy(self):
        self.file.close()  # 파일 닫기
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = ZedDataVelocity()

    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:  # Ctrl+C로 종료 시 처리
        node.get_logger().info('Shutting down gracefully')
    finally:
        node.destroy()  # 노드 종료 및 파일 닫기
        rclpy.shutdown()

if __name__ == '__main__':
    main()
