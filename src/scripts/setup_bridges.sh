echo "Starting ROS2-Gazebo bridges..."

# Bridge for cmd_vel (ROS2 -> Gazebo)
echo "Starting cmd_vel bridge..."
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
BRIDGE1_PID=$!

sleep 1

# Bridge for odometry (Gazebo -> ROS2)
echo "Starting odometry bridge..."
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry &
BRIDGE2_PID=$!

sleep 1

# Bridge for lidar (Gazebo -> ROS2) 
echo "Starting lidar bridge..."
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &
BRIDGE3_PID=$!

sleep 1

# Bridge for camera (Gazebo -> ROS2) 
echo "Starting camera bridge..."
ros2 run ros_gz_bridge parameter_bridge /camera/bottom@sensor_msgs/msg/Image@gz.msgs.Image &
BRIDGE4_PID=$!

echo ""
echo "All bridges started!"
echo "Bridge PIDs: $BRIDGE1_PID, $BRIDGE2_PID, $BRIDGE3_PID, $BRIDGE4_PID"
echo ""


trap "echo 'Stopping bridges...'; kill $BRIDGE1_PID $BRIDGE2_PID $BRIDGE3_PID $BRIDGE4_PID 2>/dev/null; exit" SIGINT SIGTERM

wait