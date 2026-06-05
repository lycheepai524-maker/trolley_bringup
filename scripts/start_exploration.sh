#!/bin/bash
# Manually start explore_lite after the bringup has had time to populate costmap
# Usage: ./start_exploration.sh

source /opt/ros/noetic/setup.bash
source ~/trolley_ws/devel/setup.bash

echo "Waiting 5 seconds for costmap to populate..."
sleep 5

echo "Launching explore_lite..."
rosrun explore_lite explore \
    _robot_base_frame:=base_link \
    _costmap_topic:=/move_base/global_costmap/costmap \
    _costmap_updates_topic:=/move_base/global_costmap/costmap_updates \
    _visualize:=false \
    _planner_frequency:=0.5 \
    _progress_timeout:=30.0 \
    _potential_scale:=3.0 \
    _orientation_scale:=0.0 \
    _gain_scale:=1.0 \
    _transform_tolerance:=0.5 \
    _min_frontier_size:=0.1

