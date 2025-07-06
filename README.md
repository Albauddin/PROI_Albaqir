# PROI_Albaqir
implementation of the KF, EKF and PF for PROI Lab

#Source it
source install/setup.bash

#how to launch the world:
ros2 launch my_cool_project myfile.launch.py

#how to launch teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

#kalman filter
ros2 run KF kalman_node


#from the documentation (https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/)



################## Tutorial 2 ##################

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz


################## Tutorial 3 ##################

docker exec -it PROI bash
cd workspace/data/
source /opt/ros/humble/setup.bash
source /workspace/data/install/setup.bash


ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch nav2_bringup localization_launch.py map:=/workspace/data/src/my_cool_project/maps/my_map.yaml

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=workspace/data/src/my_cool_project/maps/my_map.yaml rviz_config:='/hworkspace/data/PROI/config/tb3_navigation2.rviz'


ros2 launch filters filters.launch.py

ros2 run my_cool_project initial_pose_publisher

python3 /workspace/data/analysis/plot_ekf_vs_odom.py
or
python3 /workspace/data/analysis/plot_kf_vs_odom.py
or
python3 /workspace/data/analysis/plot_pf_vs_odom.py



ros2 run my_cool_project run_demo



#recording topics
ros2 bag record /kf_pose /noisy_pose /odom

#republish odometry so making jam effect
python3 republish_odometry.py


################## Killers ##################

pkill -f gazebo
pkill -f rviz
pkill -f ros2
