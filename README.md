# ROS-Scan-Matching-Lidar
ROS node that uses ICP to match successive lidar scans for position and orientation estimates

Use:
1. Start roscore
2. In catkin_ws, run 'source devel/setup.bash'
3. Launch rp_lidar view with 'roslaunch rplidar_ros view_rplidar.launch'
4. In 'my_project/src', Start Lidar subscriber node with 'python3 lidar_listener.py'

![]()

