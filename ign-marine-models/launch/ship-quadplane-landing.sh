#!/bin/bash

# Ship - Quadplane Landing Example
#
# 

# Kill all SITL binaries when exiting
trap "killall -9 arduplane & killall -9 ardurover" SIGINT SIGTERM EXIT

# edit the location of these directories if different
ARDUPILOT_ROOT="$HOME/Code/ardupilot/ardupilot"
SITL_MODELS_DIR="$HOME/Code/ardupilot/simulation/SITL_Models"
IGN_MARINE_MODELS_DIR="$HOME/Code/robotics/ign_wave_sim/wave_sim_ws/src/asv_wave_sim/ign-marine-models"

# assume we start the script from the root directory
ROOTDIR=$ARDUPILOT_ROOT
PLANE=$ROOTDIR/build/sitl/bin/arduplane
ROVER=$ROOTDIR/build/sitl/bin/ardurover

# drones will be located here
# Swansea Bay
HOMELAT=51.587587
HOMELONG=-3.954579
HOMEALT=0.0

# Default: CUAV
# HOMELAT=-35.363262
# HOMELONG=149.165237
# HOMEALT=584.0

#--------------------------------------------------------------------
# Ignition worlds

# ensure the worlds will be found
#export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$ROOTDIR/sitl/worlds

#--------------------------------------------------------------------
# ArduPilot SITL

# build binary if not present
[ -x "$PLANE" -a -x "$ROVER" ] || {
    ./waf configure --board sitl --debug
    ./waf plane rover
}

#--------------------------------------------------------------------
# Ship

mkdir -p sitl/ship

SHIP_DEFAULTS="$IGN_MARINE_MODELS_DIR/config/havyard.param"

# additional parameter file for the ship unit
cat <<EOF > sitl/ship/leader.param
SYSID_THISMAV 2
AUTO_OPTIONS 7
EOF

(cd sitl/ship && $ROVER -S --model JSON --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --speedup 1 --slave 0 --instance 1 --sysid 2 --defaults $SHIP_DEFAULTS,leader.param) &

#--------------------------------------------------------------------
# Quadplane

mkdir -p sitl/quadplane

QUADPLANE_DEFAULTS="$SITL_MODELS_DIR/Ignition/config/alti_transition_quad.param"

# additional parameter file for the quadplane
cat <<EOF > sitl/quadplane/follower.param
SYSID_THISMAV 1
AUTO_OPTIONS 7
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_Y 0
FOLL_OFS_Z 10
FOLL_OFS_TYPE 1
FOLL_SYSID 2
FOLL_DIST_MAX 1000
FOLL_YAW_BEHAVE 2
FOLL_ALT_TYPE 1
EOF

(cd sitl/quadplane && $PLANE -S --model JSON --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --speedup 1 --slave 0 --instance 0 --sysid 1 --defaults $QUADPLANE_DEFAULTS,follower.param) &

wait

