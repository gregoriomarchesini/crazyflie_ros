SCRIPT_DIR=$(dirname "$(realpath "$0")")
MAKEFILE_DIR=$(realpath "${SCRIPT_DIR}/../..")
touch $MAKEFILE_DIR/Makefile
cat > $MAKEFILE_DIR/Makefile << "EOF"
SHELL := /bin/bash

MISSION_PATH = src/crazyflie_ros/setpoint_follower/crazyflie_startup/config/missions
args := $(wordlist 2,$(words $(MAKECMDGOALS)), $(MAKECMDGOALS))
var := $(wordlist 1,$(words $(MAKEOVERRIDES)), $(MAKEOVERRIDES))
missions := $(patsubst $(MISSION_PATH)/%, %, $(wildcard $(MISSION_PATH)/*.yaml))
mission_names = $(filter $(missions), $(patsubst %, %.yaml, $(args)))
map = $(filter $(missions), $(args))$(mission_names)

ifeq ($(map), )
	backend := cflib
else
	ifneq ($(shell grep "\"\*\"" $(MISSION_PATH)/$(firstword $(map))), )
		backend := cpp
	else
		backend := cflib
	endif
endif

SOLO = $(filter solo, $(args))
ifeq ($(SOLO), )
	FILE := rvo.launch.py
else
	FILE := solo.launch.py
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
	@ for mission in $(map); do\
		source install/setup.bash && ros2 launch crazyflie_ros2_setpoint_follower $(FILE) mission_yaml:=$$mission backend:=sim;\
	done

real_launcher :
	@ source install/setup.bash && ros2 launch crazyflie_ros2_setpoint_follower $(FILE) mission_yaml:=$(map) backend:=hardware

real_hardware :
#	sleep to be sure that this one is launched after the other one
	@ gnome-terminal -- bash -c 'echo "backend : $(backend)";\
								sleep 3;\
								source /opt/ros/humble/setup.bash;\
								source install/setup.bash;\
								ros2 launch crazyflie_ros2_setpoint_follower launch_cf_hardware.launch.py backend:=$(backend);\
								exit'

real : compile
	@ for mission in $(map); do\
		$(MAKE) -j 2 real_launcher real_hardware $$mission $(SOLO);\
	done

generate :
	@ ./src/crazyflie_ros/setpoint_follower/crazyflie_startup/config/missions/gen_mission.py $(var) $(args)

%::
	@true
EOF

echo "Makefile generated"

chmod u+x $MAKEFILE_DIR/src/crazyflie_ros/setpoint_follower/crazyflie_startup/config/missions/gen_mission.py

echo "Made gen_mission.py executable"
