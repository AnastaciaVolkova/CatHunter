#!/bin/bash
echo iago | sudo -S su -c "source $1/devel/setup.bash; rosrun motion_driver motion_driver_node"

