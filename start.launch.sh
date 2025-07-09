#!/bin/bash

# --- LAUNCH TURTLEBOT3 GAZEBO WORLD ---
# Starts the TurtleBot3 simulation in Gazebo.
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
# Save the process ID (PID) so we can kill it later.
PIDS[0]=$!
# Give Gazebo time to fully load before starting the next component.
sleep 5

# --- LAUNCH NAV2 LOCALIZATION ---
# Starts the localization stack, using saved map.
ros2 launch nav2_bringup localization_launch.py map:=/workspace/data/src/my_cool_project/maps/my_map.yaml &
PIDS[1]=$!
# Wait for localization to get ready.
sleep 5

# --- LAUNCH FILTER NODE ---
# Starts custom filter node.
ros2 launch filters filters.launch.py &
PIDS[3]=$!
sleep 4

# --- LAUNCH NAVIGATION2 + RVIZ ---
# Starts the Navigation2 stack and RViz for visualization.
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/workspace/data/src/my_cool_project/maps/my_map.yaml rviz_config:=/workspace/data/config/tb3_navigation2.rviz &
PIDS[2]=$!
sleep 5

# --- RUN INITIAL POSE PUBLISHER ---
# Publishes the initial pose to AMCL, so localization can start.
ros2 run my_cool_project initial_pose_publisher &
PIDS[4]=$!
sleep 4

# --- RUN DEMO NODE ---
# Starts custom demo node.
ros2 run my_cool_project run_demo &
PIDS[5]=$!

# ---- Ctrl+C trap for clean shutdown ----
cleanup() {
    echo "Caught Ctrl+C, killing all launched processes..."
    # Kill processes in reverse order: last launched first
    for (( idx=${#PIDS[@]}-1 ; idx>=0 ; idx-- )) ; do
        kill ${PIDS[idx]} 2>/dev/null
    done
    # Extra: Kill anything that might be left behind
    pkill -9 -f ros2
    pkill -9 -f gzserver
    pkill -9 -f gzclient
    exit
}

# Set trap (catch SIGINT = Ctrl+C)
trap cleanup SIGINT

# ---- Wait for all child processes ----
wait
