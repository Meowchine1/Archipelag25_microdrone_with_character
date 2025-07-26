#!/usr/bin/env python3
import rclpy
import math
import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
import matplotlib.pyplot as plt
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint


class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        self.control_permission = False
        self.subscription = self.create_subscription(
            Bool,
            'control_permission',
            self.control_permission_callback,
            qos_profile
        ) 
        self.times = []
        self.linear_xs = []
        self.linear_ys = []
        self.angular_zs = []

        self.R = 4.0
        self.omega = 0.5
        self.yaw_rate = 0.5
        self.g = 9.81

        v = self.omega * self.R
        a_c = v**2 / self.R
        self.roll_angle = math.atan2(a_c, self.g)

        self.start_time = time.time()

        self.get_logger().info("CustomController starts")

        # Запускаем цикл управления в отдельном потоке
        self.control_timer = self.create_timer( 0.02,self.control_loop) 

    def control_permission_callback(self, msg):
        self.control_permission = msg.data
        #self.get_logger().info(f"Control permission received: {msg.data}")

    def control_loop(self):
        # Получаем текущее время в микросекундах
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Всегда публикуем OffboardControlMode (PX4 требует постоянный поток)
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = timestamp
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_mode_pub.publish(offboard_msg)

        # Создаём и публикуем TrajectorySetpoint
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = timestamp

        if not self.control_permission:
            self.start_time = time.time()
            # Пока запрет на управление — публикуем нулевой setpoint
            traj_msg.velocity = [0.0, 0.0, 0.0]
            traj_msg.yaw = 0.0
        else:
            # Разрешено управление — публикуем движение по кругу
            t = time.time() - self.start_time
            v = self.omega * self.R
            traj_msg.velocity = [
                v * math.cos(self.omega * t),
                v * math.sin(self.omega * t),
                0.0
            ]
            traj_msg.yaw = self.yaw_rate * t  # Можно просто 0.0 если не нужен разворот

            # Сохраняем данные для графиков
            self.times.append(t)
            self.linear_xs.append(traj_msg.velocity[0])
            self.linear_ys.append(traj_msg.velocity[1])
            self.angular_zs.append(self.yaw_rate)

        self.trajectory_setpoint_pub.publish(traj_msg)

    def plot_data(self):
        plt.figure(figsize=(10,6))

        plt.subplot(3,1,1)
        plt.plot(self.times, self.linear_xs, label='linear.x')
        plt.ylabel('Linear X (m/s)')
        plt.legend()

        plt.subplot(3,1,2)
        plt.plot(self.times, self.linear_ys, label='linear.y')
        plt.ylabel('Linear Y (m/s)')
        plt.legend()

        plt.subplot(3,1,3)
        plt.plot(self.times, self.angular_zs, label='angular.z')
        plt.ylabel('Angular Z (rad/s)')
        plt.xlabel('Time (s)')
        plt.legend()

        plt.tight_layout()
        plt.savefig('control_plot.png')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = CustomController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
