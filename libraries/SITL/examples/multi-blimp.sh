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

# REPARM=1

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

[ -x "$BLIMP" ] || {
	./waf configure --board sitl
	./waf blimp
}

# Set number of extra blimps to be simulated, change this for increasing the count
NBLIMPS="2"

#keep all files in Blimp/multi-blimp subdirectory
pushd Blimp
mkdir -p multi-blimp
pushd multi-blimp

# start up main (leader) blimp in the subdir (blimp1)
echo "Starting blimp 1"
mkdir -p blimp1

# if $REPARM ; then
# create default parameter file for the leader
cat <<EOF > blimp1/leader.parm
SYSID_THISMAV 1
AUTO_OPTIONS 7
EOF
# fi

pushd blimp1
$BLIMP --model blimp --home=$HOMELAT,$HOMELONG,$HOMEALT,0 --uartA udpclient:$GCS_IP --uartC mcast:$MCAST_IP_PORT --defaults $BASE_DEFAULTS,leader.parm &
popd

# now start other blimps to follow the first, using
# a separate directory to keep the eeprom.bin and logs separate
# each blimp will have an offset starting location (5*SYSID,5*SYSID)m from leader blimp
# each blimp will follow at SYSID*5m in X dir from leader
for i in $(seq $NBLIMPS); do
    SYSID=$(expr $i + 1)

    echo "Starting blimp $SYSID"
    mkdir -p blimp$SYSID

# if $REPARM ; then
    # create default parameter file for the follower
    cat <<EOF > blimp$SYSID/follow.parm
SYSID_THISMAV $SYSID
GUID_OFS_X $(echo "-1*$SYSID" | bc -l)
EOF
# fi
# GUID_OFS_Y $(echo "-1*($NBLIMPS-$i)" | bc -l)
    pushd blimp$SYSID
    LAT=$(echo "$HOMELAT" | bc -l)
    LONG=$(echo "$HOMELONG + 0.00001*$SYSID" | bc -l)
    $BLIMP --model blimp --home=$LAT,$LONG,$HOMEALT,0 --uartA tcp:0 --uartC mcast:$MCAST_IP_PORT --instance $SYSID --defaults $BASE_DEFAULTS,follow.parm &
    popd
done
wait

