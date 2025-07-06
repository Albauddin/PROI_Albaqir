# PROI_Albaqir : implementation of the KF, EKF and PF for PROI Lab

## **Step-by-Step Instructions**

1. **Create environtment and workspace for this project**

   -follow this documentation to prepare your Project with Nav2 and ROS2 slam_toolbox
     [https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/#Next_steps_with_the_generated_map](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/#Next_steps_with_the_generated_map)

2. **Launch your Gazebo world**

   Source every terminal you newly open
   ```bash
   source /opt/ros/humble/setup.bash
   source /<your workspace>/install/setup.bash
   ```
   
   Launch the gazebo world 
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

3. **Starts the ROS 2 Navigation2 stack in localization mode, using your provided map file**

   adjust yourmap file path
   ```bash
   ros2 launch nav2_bringup localization_launch.py map:=/<your workspace>/src/my_cool_project/maps/my_map.yaml
   ```
   

   
5. **launches the navigation system**

   launches a complete navigation system for TurtleBot3 in simulation, using your specific map and RViz setup

   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<your workspace>/src/my_cool_project/maps/my_map.yaml rviz_config:='/h<your workspace>/PROI/config/tb3_navigation2.rviz'
   ```

   depends on your needs you can turn on vosualization of the filter and the ground truth "/odom"
   ![image](https://github.com/user-attachments/assets/5f4ecd00-36e4-4c94-9d66-f3174686fffa)

7. **Start all the filter nodes**

   ```bash
   ros2 launch filters filters.launch.py
   ```
   
   Start all the filter nodes you defined in your launch file, such as:
      - Kalman filter node 
      - Extended Kalman filter node
      - Particle filter node

   Starting position on the map for localization algorithms to tell where the robot really is
    ```bash
   ros2 run my_cool_project initial_pose_publisher
   ```  

9. **Plot the filters**

   To Plot the Kalman Filter compared to ground truth
   ```bash
   python3 /<your workspace>/analysis/plot_kf_vs_odom.py
   ```

   To Plot the Extended Kalman Filter compared to ground truth
   ```bash
   python3 /<your workspace>/analysis/plot_ekf_vs_odom.py
   ```

   To Plot the Particle Filter compared to ground truth
   ```bash
   python3 /<your workspace>/analysis/plot_pf_vs_odom.py
   ```

   press `Ctrl + c` to and the plotting and see the result, the result will be saved in `analysis` directory
   
10. **Demo: send robot to four poses**

   To create identical comparison between all the Filters, run this script to navigate the robot to 4 poses
   ```bash
   ros2 run my_cool_project run_demo
   ```


11. **Expected results**










#recording topics
ros2 bag record /kf_pose /noisy_pose /odom

#republish odometry so making jam effect
python3 republish_odometry.py


################## Killers ##################

pkill -f gazebo
pkill -f rviz
pkill -f ros2
