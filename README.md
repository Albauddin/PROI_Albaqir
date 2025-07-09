# PROI_Albaqir : implementation of the KF, EKF and PF for PROI Lab

## **Step-by-Step Instructions**

1. **Create environtment and workspace for this project**

   Follow this documentation to prepare your Project with Nav2 and ROS2 slam_toolbox
     [https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/#Next_steps_with_the_generated_map](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/#Next_steps_with_the_generated_map)

2. **Run the docker container** (optional)

   Make it executable and run the file
   ```bash
   chmod +x docker/run.sh
   ./docker/run.sh
   ```
   

4. **Launch your Gazebo world**

   Source every terminal you newly open
   ```bash
   source /opt/ros/humble/setup.bash
   source /<your workspace>/install/setup.bash
   ```
   
   Launch the gazebo world 
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

5. **Starts the ROS 2 Navigation2 stack in localization mode, using your provided map file**

   Adjust yourmap file path
   ```bash
   ros2 launch nav2_bringup localization_launch.py map:=/<your workspace>/src/my_cool_project/maps/my_map.yaml
   ```
   

   
6. **launches the navigation system**

   Launches a complete navigation system for TurtleBot3 in simulation, using your specific map and RViz setup

   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<your workspace>/src/my_cool_project/maps/my_map.yaml rviz_config:='/h<your workspace>/PROI/config/tb3_navigation2.rviz'
   ```

   depends on your needs you can turn on vosualization of the filter and the ground truth "/odom"
   ![Screenshot from 2025-07-09 03-44-33](https://github.com/user-attachments/assets/a1c00ff5-3ed3-46ed-8d14-569a4b57b661)

   ![image](https://github.com/user-attachments/assets/5f4ecd00-36e4-4c94-9d66-f3174686fffa)


8. **Start all the filter nodes**

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

   The comparison between Kalman FIlter with the ground truth should look like this
   <!--![kf_vs_odom (2)](https://github.com/user-attachments/assets/886ae70e-26eb-4599-a711-0187946cffa8)-->
   <img src="https://github.com/user-attachments/assets/886ae70e-26eb-4599-a711-0187946cffa8" width="50%">

   The comparison between Extended Kalman FIlter(with simulated drift noises) with the ground truth should look like this
   <!--![ekf_vs_odom_cov (7)](https://github.com/user-attachments/assets/a03ae07a-bb68-4452-b2e6-fd31724be1e8)-->
   <img src="https://github.com/user-attachments/assets/a03ae07a-bb68-4452-b2e6-fd31724be1e8" width="50%">

   The comparison between Particle FIlter with the ground truth should look like this
   <!--![pf_vs_odom_particles (7)](https://github.com/user-attachments/assets/3df0ec62-53cc-493a-9315-fafa0b09bf1f)-->
   <img src="https://github.com/user-attachments/assets/3df0ec62-53cc-493a-9315-fafa0b09bf1f" width="50%">




# Video submission

## Kalman Filter VS Ground Truth
https://github.com/user-attachments/assets/c9689890-ff80-49c3-b96b-898f70e6106c


## Extended Kalman Filter VS Ground Truth
https://github.com/user-attachments/assets/6653d4b8-f16c-4270-be9a-694c1ad3ed9c

Without correction

https://github.com/user-attachments/assets/c6108717-c4b0-4923-8fbe-d996d6e7a1c4



## Particle Filter VS Ground Truth

https://github.com/user-attachments/assets/c2fd4ee0-acec-466b-a807-229e18731376




## **Troubleshoot**

   When the steps are not working as wexpected:
   - wait a little bit when you are sending command from one terminal to the other.
   - reset the gazebo, RViz and ROS2
   ```bash
   pkill -f gazebo
   pkill -f rviz
   pkill -f ros2
   ```




#republish odometry so making jam effect
python3 republish_odometry.py


