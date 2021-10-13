#!/bin/bash
# Enable motors and sending of measurements on a ODRI motor control board.
#
# This script sends a sequence of CAN messages to the specified CAN interface
# (e.g. "can0") which enable both motors and the sending of commands on a ODRI
# motor control board running the CAN-based firmware.  It is mostly meant for
# testing purposes (e.g. to check if the CAN communication is working and the
# proper firmware is running).
#
# For sending commands, `cansend` is used which on Ubuntu can be installed via
# the `can-utils` package.

# Copyright: 2021, Max Planck Gesellschaft
# License: BSD 3-clause


if [ $# != 1 ]
then
    echo "Usage: $0 <can_interface>"
    exit 1
fi

# Below is the command sequence that is also used by blmc_drivers to enable the
# board.

# set target current to zero (to make sure there is no old command active when
# the motor is enabled).
cansend $1 005#0000000000000000
sleep 0.5

# set CAN receive timeout to zero (= disable)
cansend $1 000#000000000000001E
sleep 0.5

# enable the system
cansend $1 000#0000000100000001
sleep 0.5

# enable sending all measurements
cansend $1 000#0000000100000014
sleep 0.5

# enable motor 1
cansend $1 000#0000000100000002
sleep 0.5

# enable motor 2
cansend $1 000#0000000100000003
