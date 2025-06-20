sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# restart

# Build
cd ~/ws/
colcon build
source install/setup.bash


###################################
# Run : IMU+GNSS
# Terminal 1
cd ~/ws/
source install/setup.bash
ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py

# Terminal 2 : IMU & GNSS
cd ~/ws/
source install/setup.bash
ros2 launch imu_visualizer imu.launch.py






###################################
# Run : IMU+GNSS+LIDAR
# Terminal 1
cd ~/ws/
source install/setup.bash
ros2 launch imu_visualizer imu.launch.py



# TF tree pdf
ros2 run tf2_tools view_frames