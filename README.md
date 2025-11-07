# VIO ROS2 Wrapper

ROS2 wrapper for the [lightweight VIO](lightweight_vio/) system.

## Features

- ✅ Stereo camera image synchronization
- ✅ IMU data subscription
- ✅ Time-synchronized sensor fusion ready
- ✅ Configurable topic names via parameters
- ✅ EuRoC dataset compatible

## Dependencies

- ROS2 (tested on Galactic)
- OpenCV
- Eigen3
- cv_bridge
- message_filters

## Building

```bash
# Navigate to your workspace
cd /home/eugene/real_source/vio_ws

# Build the package
colcon build --packages-select vio_ros_wrapper

# Source the workspace
source install/setup.bash
```

## Running

### With EuRoC Dataset

```bash
# Terminal 1: Launch VIO node
ros2 launch vio_ros_wrapper euroc.launch.py

# Terminal 2: Play rosbag
ros2 bag play room3_stereo_imu_raw.bag
```

### With Custom Topics

```bash
# Launch with default topics
ros2 launch vio_ros_wrapper vio.launch.py

# Launch with custom topics
ros2 launch vio_ros_wrapper vio.launch.py \
    left_image_topic:=/my_camera/left/image \
    right_image_topic:=/my_camera/right/image \
    imu_topic:=/my_imu/data
```

## Topic Subscriptions

### EuRoC Dataset (euroc.launch.py)
- `/cam0/image_raw` (sensor_msgs/Image) - Left camera image
- `/cam1/image_raw` (sensor_msgs/Image) - Right camera image  
- `/imu0` (sensor_msgs/Imu) - IMU measurements

### Default (vio.launch.py)
- `/camera/left/image_raw` (sensor_msgs/Image) - Left camera image
- `/camera/right/image_raw` (sensor_msgs/Image) - Right camera image  
- `/imu/data` (sensor_msgs/Imu) - IMU measurements

## Parameters

- `left_image_topic` (string) - Left camera topic name
- `right_image_topic` (string) - Right camera topic name
- `imu_topic` (string) - IMU topic name
- `queue_size` (int, default: 10) - Subscriber queue size

## TODO

- [ ] Integrate lightweight_vio estimator
- [ ] Publish odometry output (nav_msgs/Odometry)
- [ ] Add TF broadcaster
- [ ] Add visualization markers
- [ ] Add configuration for VIO parameters

## License

MIT License
