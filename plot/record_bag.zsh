#!/usr/bin/env zsh

###############################################################################
#2020/9/18 sundong
###############################################################################


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "${DIR}/install_isolated/setup.zsh"

function pose() {

  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
     rosbag record -o pose --split --duration=10m -b 2048  \
        /gpsdata \
        /GPSmsg \
        /gpsdata_sync \
        /GPSmsg_fix \
        /gps_distance \
        /imudata \
        /insvelocity \
        /vehiclestate_GPS \
        /lidar_odometry_for_mapping \
        /gps_by_lidar_odometry \
        /lidar_odometry_to_earth \
        /sensor_fusion_output \
        /lidar_preciseodometry_to_earth  
    fi
}

function all() {

  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
     rosbag record -o all --split --duration=3m -b 2048  \
        /gpsdata \
        /GPSmsg \
        /gpsdata_sync \
        /GPSmsg_fix \
        /gps_distance \
        /imudata \
        /insvelocity \
        /vehiclestate_GPS \
        /lidar_odometry_for_mapping \
        /gps_by_lidar_odometry \
        /lidar_odometry_to_earth \
        /sensor_fusion_output \
        /lidar_preciseodometry_to_earth \
        /lidar_cloud_calibrated
    fi
}

function stop() {
  pkill -SIGINT -f record
}

function help() {
  echo "Usage:"
  echo "$0 [pose]                     Record pose to data/bag."
  echo "$0 [all]                      Record all data to data/bag."
  echo "$0 stop                        Stop recording."
  echo "$0 help                        Show this help message."
}

case $1 in
  pose)
    shift
    pose $@
    ;;
  all)
    shift
    all $@
    ;;
  stop)
    shift
    stop $@
    ;;
  help)
    shift
    help $@
    ;;
  *)
    all $@
    ;;
esac
