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

        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Odometry,
            'zed/zed_node/odom',
            self.listener_callback,
            qos_profile
        )

        # 시간 간격을 1초로 설정
        self.timer = self.create_timer(1.0, self.save_velocity_to_file)

        # 파일 열기
        self.file = open(self.file_path, 'w')

    def listener_callback(self, msg):
        # 현재 시간 업데이트
        current_time = self.get_clock().now()

        # 이전 위치와 방향이 설정되어 있을 때만 미분 계산
        if self.prev_position is not None and self.prev_orientation is not None:
            # 시간 차이 계산
            time_diff = (current_time - self.prev_time).nanoseconds / 1e9  # 초 단위로 변환

            # 각 축(x, y, z)에 대한 위치 변화량 계산
            delta_x = msg.pose.pose.position.x - self.prev_position.x
            delta_y = msg.pose.pose.position.y - self.prev_position.y
            delta_z = msg.pose.pose.position.z - self.prev_position.z

            # 각 축(x, y, z)에 대한 선속도 계산 (위치 변화량 / 시간 차이)
            linear_velocity_x = delta_x / time_diff
            linear_velocity_y = delta_y / time_diff
            linear_velocity_z = delta_z / time_diff

            # 각 축(x, y, z)에 대한 방향 변화량 계산
            delta_orientation_x = msg.pose.pose.orientation.x - self.prev_orientation.x
            delta_orientation_y = msg.pose.pose.orientation.y - self.prev_orientation.y
            delta_orientation_z = msg.pose.pose.orientation.z - self.prev_orientation.z

            # 각 축(x, y, z)에 대한 각속도 계산 (방향 변화량 / 시간 차이)
            angular_velocity_x = delta_orientation_x / time_diff
            angular_velocity_y = delta_orientation_y / time_diff
            angular_velocity_z = delta_orientation_z / time_diff

            # 선속도와 각속도를 로그로 남기기 위해 저장
            data_to_write = (
                f"Linear Velocity: x = {linear_velocity_x:.2f} m/s, y = {linear_velocity_y:.2f} m/s, z = {linear_velocity_z:.2f} m/s\n"
                f"Angular Velocity: x = {angular_velocity_x:.2f} rad/s, y = {angular_velocity_y:.2f} rad/s, z = {angular_velocity_z:.2f} rad/s\n"
                "--------------------------\n"
            )
            self.file.write(data_to_write)
            self.get_logger().info('Velocity data written to file')

        # 이전 위치, 방향, 시간 업데이트
        self.prev_position = msg.pose.pose.position
        self.prev_orientation = msg.pose.pose.orientation
        self.prev_time = current_time

    def save_velocity_to_file(self):
        # 이 메서드는 1초마다 호출되며, 콜백에서 파일에 데이터를 기록합니다.
        pass  # 콜백 함수에서 모든 작업을 수행하므로 이 메서드는 비어 있습니다.

    def destroy(self):
        self.file.close()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = ZedDataVelocity()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully')
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
