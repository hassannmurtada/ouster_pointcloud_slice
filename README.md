# Ouster Point Cloud Slice

A ROS2 package for real-time filtering(C++) and configuration(Python) of Ouster LiDAR sensors. This package provides efficient point cloud processing with range and azimuth filtering, plus comprehensive sensor parameter management through an integrated configuration system.

## Overview

This package combines two main capabilities:

1. **Point Cloud Filtering**: Real-time range and azimuth filtering of Ouster LiDAR data
2. **Sensor Configuration**: Direct management of Ouster sensor parameters via ouster-cli integration

The filtering node processes point clouds at sensor frequency with minimal latency, while the configuration system allows you to adjust sensor settings without external tools.

## Features

- Real-time point cloud filtering with ~2-3 ms latency
- Live parameter reconfiguration without node restarts
- Comprehensive sensor configuration management
- Multiple launch options for different use cases
- Integration with standard ROS2 navigation and SLAM stacks
- Support for all major Ouster sensor parameters

## Dependencies

- ROS2 Humble
- PCL (Point Cloud Library)
- pcl_ros
- sensor_msgs
- rclcpp
- std_srvs
- ouster-sdk: `pip install ouster-sdk`

## Installation

1. Install the ouster-sdk (required for sensor configuration):
```bash
pip install ouster-sdk
```

2. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/hassannmurtada/ouster_pointcloud_slice.git
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ouster_pointcloud_slice
source install/setup.bash
```

## Quick Start

### Complete Setup (Recommended)
Launch both sensor configuration and point cloud filtering, as well as Ouster ROS driver:

1. Start the Ouster ROS driver separately:
```bash
ros2 launch ouster_ros driver.launch.py sensor_hostname:=<YOUR_SENSOR_IP>
```

2. Launch both sensor configuration and point cloud filtering:
```bash
ros2 launch ouster_pointcloud_slice slice_with_config.launch.py sensor_hostname:=169.254.223.207
```

This automatically configures your sensor and starts filtering point cloud data.

### Basic Filtering Only
If you only need point cloud filtering without sensor configuration:

1. Start the Ouster ROS driver separately:
```bash
ros2 launch ouster_ros driver.launch.py sensor_hostname:=<YOUR_SENSOR_IP>
```

2. Launch the point cloud slicer:
```bash
ros2 launch ouster_pointcloud_slice slice.launch.py
```

## Configuration

### Point Cloud Filtering Parameters

Edit `config/slice_params.yaml`:

```yaml
ouster_slice:
  ros__parameters:
    min_range: 0.0          # Minimum distance in meters
    max_range: 5.0          # Maximum distance in meters
    min_angle_deg: -180.0   # Minimum azimuth angle in degrees
    max_angle_deg: 180.0    # Maximum azimuth angle in degrees
    input_topic: /ouster/points
    output_topic: /ouster/points_filtered
```

### Sensor Configuration Parameters

The configurator supports all major Ouster sensor parameters. Key parameters include:

- **lidar_mode**: Resolution and frame rate (512x10, 512x20, 1024x10, 1024x20, 2048x10, 4096x5)
- **azimuth_window_start/end**: Horizontal field of view limits in millidegrees (0-360000)
- **udp_dest**: Destination IP address for sensor data packets
- **udp_profile_lidar**: Data format (single/dual return, different packet profiles)
- **timestamp_mode**: Timestamping method (internal, PTP, sync pulse, ROS time)
- **lidar_port/imu_port**: Network ports for data transmission
- **persist_config**: Whether to save settings permanently to sensor

For detailed parameter descriptions, see [Configuration Guide](docs/CONFIGURATION_GUIDE.md).

## Live Parameter Changes

### Point Cloud Filtering
```bash
# Adjust filtering range
ros2 param set /ouster_slice min_range 2.0
ros2 param set /ouster_slice max_range 10.0

# Set forward-facing 90-degree cone
ros2 param set /ouster_slice min_angle_deg -45.0
ros2 param set /ouster_slice max_angle_deg 45.0
```

### Sensor Configuration
```bash
# Change resolution and frame rate
ros2 param set /ouster_configurator lidar_mode "1024x20"
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Set forward-facing field of view
ros2 param set /ouster_configurator azimuth_window_start 270000
ros2 param set /ouster_configurator azimuth_window_end 90000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```

## Visualization

Launch RViz2 to visualize both original and filtered point clouds:

```bash
rviz2
```

Configure RViz2:
- Set **Fixed Frame** to `os_sensor` or `os_lidar`
- Add **PointCloud2** displays:
  - `/ouster/points` (original data)
  - `/ouster/points_filtered` (filtered data)

## Performance Monitoring

Monitor sensor configuration:
```bash
ros2 service call /ouster_configurator/get_sensor_info std_srvs/srv/Trigger
ros2 topic echo /ouster_configurator/status
```

## ROS2 Interface

### Nodes
- `/ouster_slice`: Point cloud filtering (C++)
- `/ouster_configurator`: Sensor configuration management (Python)

### Topics
- Input: `/ouster/points` (sensor_msgs/PointCloud2)
- Output: `/ouster/points_filtered` (sensor_msgs/PointCloud2)
- Status: `/ouster_configurator/status` (std_msgs/String)
- Config: `/ouster_configurator/current_config` (std_msgs/String)

### Services
- `/ouster_configurator/apply_config`: Apply parameters to sensor
- `/ouster_configurator/get_sensor_info`: Get sensor metadata
- `/ouster_configurator/reset_config`: Reset to factory defaults

## Common Use Cases

### Autonomous Navigation
```bash
# Forward-facing 90-degree FOV, balanced resolution
ros2 param set /ouster_configurator lidar_mode "1024x20"
ros2 param set /ouster_configurator azimuth_window_start 315000
ros2 param set /ouster_configurator azimuth_window_end 45000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```

### High-Detail Mapping
```bash
# Maximum resolution, full 360-degree coverage
ros2 param set /ouster_configurator lidar_mode "2048x10"
ros2 param set /ouster_configurator azimuth_window_start 0
ros2 param set /ouster_configurator azimuth_window_end 360000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```


## Troubleshooting

### Point Cloud Issues
- **No output**: Check if `/ouster/points` is being published and verify parameter ranges
- **High latency**: Check CPU usage, consider reducing sensor resolution or filtering range
- **Parameter updates not working**: Confirm nodes are running with `ros2 node list`

### Sensor Configuration Issues
- **Configuration fails**: Verify sensor connectivity with `ping <sensor_ip>` and check ouster-sdk installation
- **"No valid scans received"**: Ensure `udp_dest` matches your host IP and ports aren't blocked
- **Settings don't persist**: Use `persist_config: true` or configure via ouster-cli directly


## Package Structure

```
ouster_pointcloud_slice/
├── src/pointcloud_slice.cpp       # Point cloud filtering node
├── scripts/ouster_configurator.py # Sensor configuration node
├── launch/
│   ├── slice.launch.py            # Basic filtering
│   └── slice_with_config.launch.py # Complete setup
├── config/
│   ├── slice_params.yaml          # Filtering parameters
│   └── ouster_config.yaml         # Sensor parameters
└── docs/CONFIGURATION_GUIDE.md    # Detailed configuration documentation
```

## Documentation

For detailed information about sensor configuration parameters, performance tuning, and advanced usage, see:

- [Configuration Guide](docs/CONFIGURATION_GUIDE.md) - Comprehensive sensor parameter documentation
- [Enhancement Summary](ENHANCEMENT_SUMMARY.md) - Technical implementation details


## License

MIT License - see LICENSE file for details.
