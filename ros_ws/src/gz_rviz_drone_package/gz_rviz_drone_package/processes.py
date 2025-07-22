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
    "MicroXRCEAgent udp4 -p 8888 ",
    "gz sim -v 4 aruco_field.sdf"
   # ,
   # "ros2 run ros_gz_bridge parameter_bridge   /world/aruco_world/model/x500_mono_cam/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image"
   
    #"cd ~/PX4-Autopilot && make px4_sitl gz_x500" 
]

def terminate_processes(signal_received=None, frame=None):
    """Завершает все gnome-terminal и их дочерние процессы."""
    print("\n[INFO] Завершаем все процессы...")

    for pid in gnome_terminals:
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)  # Отправляем SIGTERM всей группе
            time.sleep(2)
            os.killpg(os.getpgid(pid), signal.SIGKILL)  # Если не завершились, принудительно SIGKILL
        except ProcessLookupError:
            pass  # Если процесс уже завершен

    print("[INFO] Все процессы завершены.")
    sys.exit(0)

# Перехватываем SIGINT (Ctrl + C)
signal.signal(signal.SIGINT, terminate_processes)

# Запускаем каждый процесс в новом gnome-terminal
for command in commands:
    proc = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command], preexec_fn=os.setsid)
    gnome_terminals.append(proc.pid)
    time.sleep(1)

# Ждем завершения (не завершается, пока не нажмешь Ctrl + C)
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    terminate_processes()
