SCRIPT_DIR=$(dirname "$(realpath "$0")")
MAKEFILE_DIR=$(realpath "${SCRIPT_DIR}/../..")
touch $MAKEFILE_DIR/Makefile
cat > $MAKEFILE_DIR/Makefile << "EOF"
SHELL := /bin/bash

args := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
map = $(filter %.yaml, $(args))
ifeq ($(map), )
#	By default, we run on the map test_real.yaml
	map = test_real.yaml
endif

COLCON_STAMP := .colcon.stamp

SRC = $(shell find src/ -name '*.py' ! -name "gen_mission.py" -o -name '*.cpp')

# Runs every only if a cpp or a python file is changed
$(COLCON_STAMP) : $(SRC)
	@ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
	@ touch $@

compile : $(COLCON_STAMP)

simu : compile
# 	Each command is run independantly so we need to source and launch ros2 at the same time
	@ source install/setup.bash && ros2 launch crazyflie_ros2_setpoint_follower rvo.launch.py mission_yaml:=$(map) backend:=sim

real_launcher :
	@ source install/setup.bash && ros2 launch crazyflie_ros2_setpoint_follower rvo.launch.py mission_yaml:=$(map) backend:=hardware

real_hardware :
#	sleep to be sure that this one is launched after the other one
	@ gnome-terminal -- bash -c 'sleep 3; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 launch crazyflie_ros2_setpoint_follower launch_cf_hardware.launch.py; exit'

real : compile
	@ $(MAKE) -j 2 real_launcher real_hardware $(map)

%::
	@true
EOF

echo "Makefile generated"
