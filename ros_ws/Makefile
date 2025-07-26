.PHONY: all kill

SHELL := /bin/bash
package_name := rviz_drone_package #gz_rviz_drone_package
deps_package := px4_msgs

all: check_ros_env build_deps build_main launch

check_ros_env:
	@bash -c '\
		if [ -f /opt/ros/humble/setup.bash ]; then \
			echo "Sourcing ROS 2 environment..."; \
			source /opt/ros/humble/setup.bash; \
		else \
			echo "WARNING: ROS 2 setup.bash not found at /opt/ros/humble/setup.bash"; \
		fi; \
		if [ -f install/setup.bash ]; then \
			echo "Sourcing local install setup..."; \
			source install/setup.bash; \
		else \
			echo "WARNING: Local install/setup.bash not found"; \
		fi; \
		if [[ "$$ROS_VERSION" != "2" || "$$ROS_PYTHON_VERSION" != "3" || "$$ROS_DISTRO" != "humble" ]]; then \
			echo "Неверные переменные окружения:"; \
			echo "  ROS_VERSION=$$ROS_VERSION (должно быть 2)"; \
			echo "  ROS_PYTHON_VERSION=$$ROS_PYTHON_VERSION (должно быть 3)"; \
			echo "  ROS_DISTRO=$$ROS_DISTRO (должно быть humble)"; \
			echo "  Проверьте установку ROS 2 согласно инструкции."; \
			exit 1; \
		fi'

build_deps:
	@echo ">>> Checking if $(deps_package) is built..."
	@if [ ! -d "build/$(deps_package)" ] || [ ! -d "install/$(deps_package)" ]; then \
		echo ">>> Building dependency package: $(deps_package)"; \
		colcon build --packages-select $(deps_package); \
	else \
		echo ">>> $(deps_package) already built."; \
	fi

build_main:
	@echo ">>> Building package $(package_name)"
	colcon build --packages-select $(package_name)

launch:
	@echo ">>> Launching..."
	@bash -c '\
		source /opt/ros/humble/setup.bash 2>/dev/null || echo "WARNING: ROS 2 setup.bash not found"; \
		source install/setup.bash 2>/dev/null || echo "WARNING: Local install/setup.bash not found"; \
		ros2 launch $(package_name) gz_rviz.launch.py'

kill:
	@echo ">>> Killing Gazebo, PX4 and related processes..."
	@sudo pkill -f gzserver || true
	@sudo pkill -f gz || true
	@sudo pkill -f px4_sitl || true
	@sudo pkill -f px4 || true
