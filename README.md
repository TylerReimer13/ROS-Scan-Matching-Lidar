# ROS-Scan-Matching-Lidar
ROS node that uses ICP to match successive lidar scans for position and orientation estimates

Use:
1. Start roscore
2. Add 'my_project' and 'rp_lidar' to your catkin_ws, and run 'catkin_make' at top level of catkin_ws
3. In catkin_ws, run 'source devel/setup.bash'
4. Launch rp_lidar view with 'roslaunch rplidar_ros view_rplidar.launch'
5. In 'my_project/src', Start Lidar subscriber node with 'python3 lidar_listener.py'

![](https://github.com/TylerReimer13/ROS-Scan-Matching-Lidar/blob/main/Viz/LidarScanRoom.png)


![](https://github.com/TylerReimer13/ROS-Scan-Matching-Lidar/blob/main/Viz/LidarNodeOutput.png)

