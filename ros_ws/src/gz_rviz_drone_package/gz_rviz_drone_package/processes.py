#!/usr/bin/env python3

import subprocess
import signal
import os
import sys
import time
# Список процессов gnome-terminal
gnome_terminals = []
 

# in .bashrc I set:
# base_path="$HOME/archipelag25/gz_rviz_drone/gz_rviz_drone_package/resource/gz_aruco_world"
# export GZ_SIM_RESOURCE_PATH="$base_path/models:$base_path/worlds:$HOME/PX4-Autopilot/Tools/simulation/gz/models"
# export GZ_MODEL_PATH="$base_path/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models"

# Команды для запуска

commands = [
    "MicroXRCEAgent udp4 -p 8888",
    "gz sim -v 4 aruco_field.sdf",
    

##
# ros2 run ros_gz_image image_bridge /camera
# ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image
# ##

    # # PX4 drone 1
    # "export PX4_GZ_MODEL_NAME=x500_vision_0 && "
    # "export PX4_GZ_MODEL_POSE='3,3,0.5,0,0,0' && "
    # "cd ~/PX4-Autopilot && make px4_sitl gz_x500_vision",

    # # PX4 drone 2
    # "export PX4_GZ_MODEL_NAME=x500_vision_1 && "
    # "export PX4_GZ_MODEL_POSE='-3,-3,0.5,0,0,0' && "
    # "cd ~/PX4-Autopilot && make px4_sitl gz_x500_vision",
]

# commands = [ 
#     "MicroXRCEAgent udp4 -p 8888 ",
#     "gz sim -v 4 aruco_field.sdf",
#      #"make px4_sitl gz_x500"
#      "make px4_sitl gz_x500_vision"
#      #"make px4_sitl gz_x500_depth"
#      #"make px4_sitl gz_x500_lidar_down"
#      #"make px4_sitl gz_x500_lidar_2d"
#      #"make px4_sitl gz_x500_lidar_front"
   
# ]

def terminate_processes(signal_received=None, frame=None):
    print("\n[INFO] Завершаем все процессы...")
    for pid in gnome_terminals:
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
            time.sleep(2)
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    print("[INFO] Все процессы завершены.")
    sys.exit(0)

signal.signal(signal.SIGINT, terminate_processes)

# Запускаем каждый процесс в новом терминале
for command in commands:
    proc = subprocess.Popen(
        ["gnome-terminal", "--", "bash", "-c", command],
        preexec_fn=os.setsid
    )
    gnome_terminals.append(proc.pid)
    time.sleep(1)

# Бесконечное ожидание до Ctrl+C
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    terminate_processes()
