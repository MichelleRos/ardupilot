#!/bin/bash

# Usage - From ardupilot root directory, run - libraries/SITL/examples/multi-blimp.sh $GCS_IP
# $GCS_IP is the IP address of the system running the GCs, by default is 127.0.0.1
# Use "follow-mavproxy.sh" to run MAVProxy with all vehicles
# Or connect your GCS using multicast UDP
# If you can't use multicast, you can connect via UDP on vehicle 1, which will relay telemetry
# from the other vehicles

#stop script on any error (eg compile fail)
set -e

# Kill all SITL binaries when exiting
trap "killall -9 blimp" SIGINT SIGTERM EXIT

# Always recompile blimp
./waf configure --board sitl
./waf blimp

# Get the ArduPilot directory (ROOTDIR)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "SCRIPT_DIR = $SCRIPT_DIR"
ROOTDIR="$(dirname "$(dirname "$(dirname $SCRIPT_DIR)")")"
echo "ROOT_DIR = $ROOT_DIR"
BLIMP=$ROOTDIR/build/sitl/bin/blimp

# Drones will be located here
HOMELAT=-35.280252
HOMELONG=149.005821
HOMEALT=597.3

# Set number of blimps to be simulated
NBLIMPS="5"

# Set GCS_IP address
if [ -z $1 ]; then
    GCS_IP="127.0.0.1"
else
    GCS_IP=$1
fi

# Check if SITL blimp has been built
if [ -f "$BLIMP" ]
then
   echo "Found SITL executable"
else
   echo "SITL executable not found ($BLIMP). Exiting"
   exit
fi

# Check if Platform is Native Linux, WSL or Cygwin
# Needed for correct multicast addressing
unameOut="$(uname -s)"

if [ "$(expr substr $unameOut 1 5)" == "Linux" ]; then
    # Check for WSL
    if grep -q Microsoft /proc/version; then
        MCAST_IP_PORT="127.0.0.1:14550"

    # Native Linux
    else
        MCAST_IP_PORT=""                    # Use default IP, port
    fi

elif [ "$(expr substr $unameOut 1 6)" == "CYGWIN" ]; then
    MCAST_IP_PORT="0.0.0.0:14550"
fi

BASE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/blimp.parm"

#keep all files in Blimp/multi-blimp subdirectory
pushd Blimp
mkdir -p multi-blimp
pushd multi-blimp

#delete eeprom.bin to ensure the defaults get used.
#Comment this line out if parameters were set within MAVProxy that you would like to keep between runs.
rm -f */eeprom.bin

#delete dumpcore and dumpstack files sometimes created by force-exiting the SITL instances.
rm -f */dump*.out
#note that we do not generally want to delete the entire multi-blimp folder as that would also delete all logs.

# now start the blimps, using a separate directory to keep the eeprom.bin and logs separate
for SYSID in $(seq $NBLIMPS); do
    # SYSID=$(expr $i + 1)

    echo "Starting blimp $SYSID"
    mkdir -p blimp$SYSID

    # create default parameter file for the blimp
    #MAX* parameters are so that the sim blimps fly at about the same speed as the real one
    cat <<EOF > blimp$SYSID/param.parm
SYSID_THISMAV $SYSID
PSO_MIN_DIST     1.0
PSO_SPEED_LIMIT  0.4
PSO_W_AVOID      0.5
PSO_W_GLO_BEST   0.4
PSO_W_PER_BEST   0.1
PSO_W_VEL        0.5
PSO_REDUCE       0.05
PSO_SOURCE_FOUND 0.01
EK2_ENABLE       0
EK3_IMU_MASK     1.0
MAX_POS_XY       0.15
MAX_POS_YAW      0.3
MAX_POS_Z        0.12
MAX_VEL_XY       0.2
MAX_VEL_YAW      0.4
MAX_VEL_Z        0.15
EOF

    OFFSETX=$(echo "0.00001*$(expr $SYSID % 2)" | bc -l)
    OFFSETY=$(echo "0.00001*($SYSID - 1)" | bc -l)
    pushd blimp$SYSID
    LAT=$(echo "$HOMELAT + $OFFSETX" | bc -l)
    LONG=$(echo "$HOMELONG + $OFFSETY" | bc -l)
    echo "Launching blimp$SYSID at $LAT $LONG"
    $BLIMP --model blimp --home=$LAT,$LONG,$HOMEALT,0 --uartA tcp:0 --uartC mcast:$MCAST_IP_PORT --instance $SYSID  --defaults $BASE_DEFAULTS,param.parm &
    popd
done
wait

