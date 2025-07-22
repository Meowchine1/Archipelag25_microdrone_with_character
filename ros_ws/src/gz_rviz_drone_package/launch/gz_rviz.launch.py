#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name = 'gz_rviz_drone_package'
    package_dir = get_package_share_directory(package_name)
    # Пути к моделям и миру ArUco внутри текущего ROS пакета
    base_path = os.path.join(package_dir, 'resource', 'gz_aruco_world')
    world_path = os.path.join(base_path, 'worlds', 'aruco_field.sdf')
    #px4_models = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')
    px4_models = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/PX4-gazebo-models/models')
    return LaunchDescription([

        # Set GZ environment variables so Gazebo can find models/world
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=f'{base_path}/models:{base_path}/worlds:{px4_models}'),
        SetEnvironmentVariable(name='GZ_MODEL_PATH', value=f'{base_path}/models:{px4_models}'),

        # Запускаем мир в Gazebo Sim
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', world_path],
            output='screen'
        ), 
        Node(
            package=package_name,
            namespace=package_name,
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package=package_name,
            namespace=package_name,
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
  
    ])

