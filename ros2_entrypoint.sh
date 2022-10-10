#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source app/install/setup.bash 

exec "$@"
