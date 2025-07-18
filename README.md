# Ouster Point Cloud Slice

A high-performance ROS2 C++ package for real-time filtering of Ouster LiDAR point clouds by range and azimuth angle. Designed for minimal latency processing with live parameter reconfiguration.

## Overview

This package provides efficient point cloud filtering to extract specific sectors from Ouster LiDAR data, reducing computational load for downstream applications like obstacle avoidance, navigation, and sensor fusion.

## Features

- **Real-time Processing**: ~19 Hz publication rate matching LiDAR sensor frequency
- **Low Latency**: ~2-3 ms processing delay
- **Live Reconfiguration**: Change parameters without restarting the node
- **Dual Filtering**: Range (distance) and azimuth (angle) filtering
- **Configurable Topics**: Custom input/output topic names
- **PCL Integration**: Uses Point Cloud Library for efficient processing

## Dependencies

- ROS2 Humble
- PCL (Point Cloud Library)
- pcl_ros
- sensor_msgs
- rclcpp

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/hassannmurtada/ouster_bridge.git
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select ouster_pointcloud_slice
source install/setup.bash
```

## Usage

### Basic Launch

Start the point cloud slicer with default parameters:

```bash
ros2 launch ouster_pointcloud_slice slice.launch.py
```

**Default Parameters:**
- Range: 1.0 - 5.0 meters
- Angle: 0° - 90° (front-right quadrant)
- Input: `/ouster/points`
- Output: `/ouster/points_filtered`

### Configuration

Edit `config/slice_params.yaml` to customize default parameters:

```yaml
ouster_slice:
  ros__parameters:
    min_range:      1.0      # minimum distance (meters)
    max_range:      5.0      # maximum distance (meters)  
    min_angle_deg:  0.0      # minimum azimuth angle (degrees)
    max_angle_deg:  90.0     # maximum azimuth angle (degrees)
    input_topic:    /ouster/points
    output_topic:   /ouster/points_filtered
```

### Live Parameter Updates

Change filtering parameters in real-time:

```bash
# Adjust range filtering
ros2 param set /ouster_slice min_range 2.0
ros2 param set /ouster_slice max_range 8.0

# Adjust angle filtering  
ros2 param set /ouster_slice min_angle_deg -45.0
ros2 param set /ouster_slice max_angle_deg 45.0
```

## Performance Testing

### Check Publication Rate
```bash
ros2 topic hz /ouster/points_filtered
# Expected: ~19 Hz (matching your LiDAR frequency)
```

### Check Latency
```bash
ros2 topic delay /ouster/points_filtered  
# Expected: ~2-3 ms
```

### Monitor Parameters
```bash
ros2 param list /ouster_slice
ros2 param get /ouster_slice max_range
```

## Visualization in RViz2

1. Launch RViz2: `rviz2`
2. Add PointCloud2 displays:
   - **Topic 1**: `/ouster/points` → Full 360° point cloud
   - **Topic 2**: `/ouster/points_filtered` → Filtered sector
3. Set Fixed Frame to your LiDAR frame (usually `os_sensor`)

## Use Cases

- **Obstacle Avoidance**: Filter forward-facing points for navigation
- **Sensor Fusion**: Reduce point cloud size before camera alignment
- **Performance Optimization**: Process only relevant sectors for real-time applications
- **Multi-Robot Systems**: Different robots monitor different sectors

## Technical Details

### Algorithm
1. **Range Filtering**: PCL PassThrough filter on distance
2. **Azimuth Filtering**: Manual filtering using `atan2(y, x)`
3. **Output**: Cleaned point cloud with original timestamps

### Coordinate System
- **0°**: Positive X-axis (sensor front)
- **90°**: Positive Y-axis (sensor left)
- **180°**: Negative X-axis (sensor back)
- **270°**: Negative Y-axis (sensor right)

### Performance Characteristics
- **Input Rate**: 19 Hz (typical Ouster sensor)
- **Output Rate**: 19 Hz (matching input)
- **Processing Latency**: 2-3 ms
- **Memory Usage**: Low (streaming processing)

## Package Structure

```
ouster_pointcloud_slice/
├── src/
│   └── pointcloud_slice.cpp     # Main filtering node
├── launch/
│   └── slice.launch.py          # Launch file with parameters
├── config/
│   └── slice_params.yaml        # Default parameter values
├── CMakeLists.txt               # Build configuration
├── package.xml                  # Package manifest
└── README.md                    # This file
```

## Troubleshooting

### No Output on /ouster/points_filtered
- Check if `/ouster/points` is being published: `ros2 topic hz /ouster/points`
- Verify the slicer node is running: `ros2 node list | grep ouster_slice`
- Check parameter values aren't too restrictive

### High Latency
- Verify PCL is properly installed
- Check system CPU usage
- Consider reducing input point cloud density

### Parameter Updates Not Working
- Confirm node is running: `ros2 node info /ouster_slice`
- Check parameter exists: `ros2 param list /ouster_slice`

## Integration with Ouster Bridge

This package works seamlessly with the main `ouster_bridge` package for localization:

```bash
# Terminal 1: Ouster ROS driver + Point cloud filtering
ros2 launch ouster_ros driver.launch.py sensor_hostname:=169.254.223.207 &
ros2 launch ouster_pointcloud_slice slice.launch.py

# Terminal 2: Localization with visualization  
ros2 launch ouster_bridge localize_with_bridge.launch.py
```

## Contributing

Contributions welcome! Areas for improvement:
- Additional filtering methods (height, intensity)
- GPU acceleration for larger point clouds
- Multi-sector filtering
- Dynamic reconfigure GUI

## License

MIT License - see LICENSE file for details.
