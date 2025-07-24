#!/usr/bin/env python3

import subprocess
import signal
import os
import sys
import time

processes = []

# in .bashrc I set:
# base_path="$HOME/archipelag25/gz_rviz_drone/gz_rviz_drone_package/resource/gz_aruco_world"
# export GZ_SIM_RESOURCE_PATH="$base_path/models:$base_path/worlds:$HOME/PX4-Autopilot/Tools/simulation/gz/models"
# export GZ_MODEL_PATH="$base_path/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models"

def terminate_processes(signal_received=None, frame=None):
    print("\n[INFO] Завершаем все процессы...")
    for p in processes:
        try:
            p.terminate()
            time.sleep(1)
            p.kill()
        except Exception:
            pass

    # Завершаем все процессы px4, если остались
    subprocess.run(["pkill", "-f", "px4"])
    subprocess.run(["pkill", "-9", "ruby"]) 
    print("[INFO] Все процессы завершены.")
    sys.exit(0)

signal.signal(signal.SIGINT, terminate_processes)

# Запускаем MicroXRCEAgent без терминала, вывод в /dev/null
p1 = subprocess.Popen(
    ["MicroXRCEAgent", "udp4", "-p", "8888"],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)
processes.append(p1)

# Запускаем PX4 в отдельном терминале, чтобы видеть вывод
p2 = subprocess.Popen(
    ["gnome-terminal", "--", "bash", "-c", "cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco_field make px4_sitl gz_x500_vision; exec bash"],
    preexec_fn=os.setsid
)
processes.append(p2)

# Можно добавить запуск других процессов по аналогии,
# либо запускать их в фоне с подавлением вывода

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    terminate_processes()
