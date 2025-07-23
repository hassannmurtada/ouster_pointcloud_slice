# Ouster LiDAR Configuration Guide

This document provides comprehensive information about configuring Ouster LiDAR sensors using the ouster_pointcloud_slice package.

## Overview

The Ouster configurator allows you to modify sensor parameters in real-time using ROS2 services and parameters. All configuration changes are applied directly to the sensor via the ouster-cli interface.

## Core Configuration Parameters

### Network Settings

#### sensor_hostname
- **Description**: IP address or hostname of the Ouster sensor
- **Type**: String
- **Default**: "169.254.223.207"
- **Example**: "192.168.1.100", "169.254.223.207"

#### udp_dest
- **Description**: Destination IP address where sensor sends data packets
- **Type**: String
- **Default**: "" (uses sensor default)
- **Example**: "192.168.1.100" (unicast), "239.1.2.3" (multicast)

#### lidar_port / imu_port
- **Description**: UDP ports for LiDAR and IMU data
- **Type**: Integer (0-65535)
- **Default**: 0 (auto-assign)
- **Example**: 7502, 7503

#### mtp_dest / mtp_main
- **Description**: Multicast configuration parameters
- **Type**: String / Boolean
- **Usage**: For multicast data distribution to multiple clients

### LiDAR Operation Parameters

#### lidar_mode
- **Description**: Sets resolution and frame rate simultaneously
- **Type**: String
- **Options**: 
  - "512x10" - 512 azimuth columns, 10 Hz
  - "512x20" - 512 azimuth columns, 20 Hz
  - "1024x10" - 1024 azimuth columns, 10 Hz
  - "1024x20" - 1024 azimuth columns, 20 Hz (recommended)
  - "2048x10" - 2048 azimuth columns, 10 Hz
  - "4096x5" - 4096 azimuth columns, 5 Hz
- **Impact**: Higher resolution = more points, lower frame rate = less bandwidth

#### azimuth_window_start / azimuth_window_end
- **Description**: Defines horizontal field of view in millidegrees
- **Type**: Integer (0-360000)
- **Default**: 0 to 360000 (full 360°)
- **Examples**:
  - Forward-facing 90°: start=-45000, end=45000
  - Left hemisphere: start=0, end=180000
  - Custom sector: start=45000, end=135000

### Data Format Parameters

#### udp_profile_lidar
- **Description**: Selects which data channels are transmitted
- **Type**: String
- **Options**:
  - "LEGACY" - Legacy format (not recommended)
  - "RNG19_RFL8_SIG16_NIR16" - Standard single return
  - "RNG19_RFL8_SIG16_NIR16_DUAL" - Dual return (2x bandwidth)
  - "FUSA_RNG15_RFL8_NIR8_DUAL" - Functional safety dual return
- **Usage**: Dual return modes help detect vegetation and thin objects

#### timestamp_mode
- **Description**: Method for timestamping measurements
- **Type**: String
- **Options**:
  - "TIME_FROM_INTERNAL_OSC" - Internal sensor clock
  - "TIME_FROM_SYNC_PULSE_IN" - External sync pulse
  - "TIME_FROM_PTP_1588" - PTP network time
  - "TIME_FROM_ROS_TIME" - ROS system time
- **Usage**: Use PTP or SYNC_IN for multi-sensor synchronization

#### ptp_utc_tai_offset
- **Description**: UTC/TAI offset for PTP timestamp mode
- **Type**: Float
- **Default**: -37.0
- **Usage**: Only relevant when using TIME_FROM_PTP_1588

### Configuration Management

#### persist_config
- **Description**: Whether to save settings to sensor permanent memory
- **Type**: Boolean
- **Default**: false
- **Usage**: true = settings survive power cycle, false = temporary

#### auto_apply_on_startup
- **Description**: Automatically apply ROS parameters when node starts
- **Type**: Boolean
- **Default**: true
- **Usage**: Convenient for consistent sensor configuration

## Performance Considerations

### Resolution vs Frame Rate Trade-offs

| Mode | Points/Scan | Update Rate | Bandwidth | Best For |
|------|-------------|-------------|-----------|----------|
| 4096x5 | ~524,000 | 5 Hz | Very High | Surveying, detailed mapping |
| 2048x10 | ~262,000 | 10 Hz | High | Inspection, high-detail SLAM |
| 1024x20 | ~131,000 | 20 Hz | Medium | General robotics, navigation |
| 1024x10 | ~131,000 | 10 Hz | Medium-Low | Long-range navigation |
| 512x20 | ~65,000 | 20 Hz | Low | Fast autonomy |
| 512x10 | ~65,000 | 10 Hz | Very Low | Maximum range, minimal processing |

### Azimuth Window Optimization

Reducing the field of view can significantly decrease bandwidth and processing requirements:
- 180° FOV: ~50% reduction
- 120° FOV: ~67% reduction  
- 90° FOV: ~75% reduction

### Network Bandwidth Estimation

Approximate bandwidth requirements:
- Single return modes: 10-50 Mbps depending on resolution/rate
- Dual return modes: 20-100 Mbps (approximately double)
- Additional factors: packet overhead, network protocol efficiency

## Configuration Workflows

### Basic Setup
```bash
# Set sensor IP and basic parameters
ros2 param set /ouster_configurator sensor_hostname "169.254.223.207"
ros2 param set /ouster_configurator udp_dest "192.168.1.100"
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```

### Performance Optimization
```bash
# Balanced mode for general robotics
ros2 param set /ouster_configurator lidar_mode "1024x20"
ros2 param set /ouster_configurator azimuth_window_start -90000  # -90°
ros2 param set /ouster_configurator azimuth_window_end 90000     # +90°
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```

### High-Detail Mapping
```bash
# Maximum resolution for detailed scanning
ros2 param set /ouster_configurator lidar_mode "2048x10"
ros2 param set /ouster_configurator azimuth_window_start 0
ros2 param set /ouster_configurator azimuth_window_end 360000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```

### Vegetation Detection
```bash
# Dual return mode for seeing through foliage
ros2 param set /ouster_configurator udp_profile_lidar "RNG19_RFL8_SIG16_NIR16_DUAL"
ros2 param set /ouster_configurator lidar_mode "1024x10"
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```

## Advanced Topics

### Multi-Sensor Configurations
When using multiple sensors:
- Assign unique lidar_port and imu_port to each sensor
- Use different UDP destinations or multicast groups
- Consider network switch capacity and VLAN segmentation

### Synchronization
For multi-sensor fusion:
- Use TIME_FROM_PTP_1588 with a PTP grandmaster clock
- Or use TIME_FROM_SYNC_PULSE_IN with hardware sync distribution
- Ensure all sensors use the same timestamp_mode

### Troubleshooting

#### Configuration Not Applied
- Check sensor connectivity: `ping <sensor_ip>`
- Verify ouster-cli installation: `pip show ouster-sdk`
- Check for conflicting applications using the sensor

#### Poor Performance
- Reduce resolution or frame rate
- Implement azimuth window to limit FOV
- Check network bandwidth and CPU usage
- Consider point cloud filtering downstream

#### Network Issues
- Verify UDP destination matches host IP
- Check firewall settings for UDP ports
- Ensure network interface can handle bandwidth
- Monitor for packet drops with network tools

## Parameter Reference

Complete list of configurable parameters with their ouster-cli equivalents:

| ROS Parameter | ouster-cli Command | Description |
|---------------|-------------------|-------------|
| sensor_hostname | source \<ip\> | Sensor IP address |
| lidar_mode | config lidar_mode \<mode\> | Resolution and frame rate |
| azimuth_window_start | config azimuth_window_start \<value\> | FOV start angle |
| azimuth_window_end | config azimuth_window_end \<value\> | FOV end angle |
| udp_dest | config udp_dest \<ip\> | Data destination IP |
| udp_profile_lidar | config udp_profile_lidar \<profile\> | Packet format |
| timestamp_mode | config timestamp_mode \<mode\> | Timestamping method |
| lidar_port | config lidar_port \<port\> | LiDAR data port |
| imu_port | config imu_port \<port\> | IMU data port |
| persist_config | config persist_config true | Save to permanent memory |

This guide covers the essential aspects of Ouster LiDAR configuration. For sensor-specific details, consult the Ouster sensor manual and firmware documentation.
