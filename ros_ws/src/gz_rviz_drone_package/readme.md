Сначала необходимо собрать
colcon build --packages-select px4_msgs


rqt  
rqt_graph (rqtи выбрав Plugins > Introspection > Node Graph .)



Все пакеты ROS 2 начинаются с выполнения команды

ros2 pkg create --license Apache-2.0 <pkg-name> --dependencies [deps]



base_path="$HOME/archipelag25/gz_rviz_drone/gz_rviz_drone_package/resource/gz_aruco_world"
export GZ_SIM_RESOURCE_PATH="$base_path/models:$base_path/worlds:$HOME/PX4-Autopilot/Tools/simulation/gz/models"
export GZ_MODEL_PATH="$base_path/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models"


sudo apt install gz-garden


Чтобы включить опцию продвинутого запуска (с параметрами), нужно один раз собрать SITL, но не запускать:
make px4_sitl
Это создаст папку build/px4_sitl_default/... с бинарниками. 



You need to create a bridge with either ros_gz_image or ros_gz_bridge.

ros2 run ros_gz_image image_bridge /camera
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image