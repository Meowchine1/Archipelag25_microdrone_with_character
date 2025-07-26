#!/usr/bin/env python3
import rclpy
import math
import time
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import matplotlib.pyplot as plt

def main():
    rclpy.init()
    node = rclpy.create_node('auto_twist_controller')

    times = []
    linear_xs = []
    linear_ys = []
    angular_zs = []

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    pub = node.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile)

    node.get_logger().info("Moving in a circle...")
    start = time.time()

    R = 4.0         # Радиус круга (метры)
    omega = 0.5     # Угловая скорость (рад/с)
    yaw_rate = 0.5  # Скорость вращения по yaw (рад/с)
    g = 9.81        # Ускорение свободного падения (м/с^2)

    # Расчёт желаемого крена (наклона) в радианах
    # tan(roll) = a_c / g = (v^2 / R) / g
    v = omega * R
    a_c = v**2 / R
    roll_angle = math.atan2(a_c, g)  # Наклон к центру

    try:
        while True:
            t = time.time() - start
            twist = Twist()

            # Тангенциальные компоненты скорости
            twist.linear.x = v * math.cos(omega * t)
            twist.linear.y = v * math.sin(omega * t)
            twist.linear.z = 0.0

            # Добавим roll — наклон внутрь окружности
            twist.angular.x = 0.0 #roll_angle  # Roll 
            twist.angular.y = 0.0         # Pitch
            twist.angular.z = yaw_rate    # Yaw поворот

            pub.publish(twist)

            # Логирование
            times.append(t)
            linear_xs.append(twist.linear.x)
            linear_ys.append(twist.linear.y)
            angular_zs.append(twist.angular.z)

            time.sleep(0.1)

    except KeyboardInterrupt:
        plt.figure(figsize=(10,6))

        plt.subplot(3,1,1)
        plt.plot(times, linear_xs, label='linear.x')
        plt.ylabel('Linear X (m/s)')
        plt.legend()

        plt.subplot(3,1,2)
        plt.plot(times, linear_ys, label='linear.y')
        plt.ylabel('Linear Y (m/s)')
        plt.legend()

        plt.subplot(3,1,3)
        plt.plot(times, angular_zs, label='angular.z')
        plt.ylabel('Angular Z (rad/s)')
        plt.xlabel('Time (s)')
        plt.legend()

        plt.tight_layout()
        plt.savefig('control_plot.png')
        plt.show()

if __name__ == '__main__':
    main()
