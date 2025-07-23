# Ouster Point Cloud Slice & Configuration Manager

A high-performance ROS2 C++ package for real-time filtering of Ouster LiDAR point clouds **with integrated sensor configuration management**. This package combines efficient point cloud slicing with comprehensive Ouster LiDAR parameter configuration via `ouster-cli`, providing a complete solution for Ouster sensor management in ROS2.

## 🚀 New Features

- **🔧 Integrated LiDAR Configuration**: Configure Ouster sensor parameters directly through ROS2 services
- **⚡ Live Parameter Updates**: Change both filtering and sensor parameters without restarting
- **📊 Configuration Monitoring**: Real-time status updates and sensor information
- **🎯 Optimized Performance**: ~19 Hz publication rate with ~2-3 ms latency
- **🔄 Auto-Configuration**: Automatically apply sensor settings on startup

## Overview

This package provides:

1. **Point Cloud Filtering**: Efficient range and azimuth filtering of Ouster LiDAR data
2. **Sensor Configuration**: Complete Ouster LiDAR parameter management via ouster-cli integration  
3. **Live Reconfiguration**: Change parameters without restarting nodes
4. **Performance Monitoring**: Real-time status and configuration feedback

Perfect for applications requiring both optimized point cloud processing and dynamic sensor configuration.

## Features

- **Real-time Processing**: ~19 Hz publication rate matching LiDAR sensor frequency
- **Low Latency**: ~2-3 ms processing delay
- **Live Reconfiguration**: Change parameters without restarting the node
- **Dual Filtering**: Range (distance) and azimuth (angle) filtering
- **Configurable Topics**: Custom input/output topic names
- **PCL Integration**: Uses Point Cloud Library for efficient processing
- **🆕 Sensor Configuration**: Complete Ouster LiDAR parameter management
- **🆕 ouster-cli Integration**: Direct sensor configuration via ROS2 services
- **🆕 Auto-Apply Settings**: Automatically configure sensor on startup
- **🆕 Configuration Monitoring**: Real-time sensor status and parameter feedback

## Dependencies

- ROS2 Humble
- PCL (Point Cloud Library)
- pcl_ros
- sensor_msgs
- rclcpp
- **🆕 ouster-sdk**: `pip install ouster-sdk` (for sensor configuration)
- **🆕 std_srvs**: For configuration services

## Installation

1. **Install ouster-sdk** (required for sensor configuration):
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

## 🔧 Ouster LiDAR Configuration Parameters

This package provides comprehensive control over Ouster LiDAR parameters, focusing on the most impactful settings for resolution, frame rate, and data quality.

### The Critical Parameters — What You Can Actually **Tune**

| Rank ▲ | Setting (ROS param / CLI name) | Why You'd Touch It | Typical Values & Notes |
| --- | --- | --- | --- |
| 1 | **`lidar_mode`** | Sets horizontal resolution **&** frame-rate – the single biggest driver of point-count, bandwidth, range boost and CPU load. | `2048x10` → max detail, 10 Hz<br>`1024x20` → balanced<br>`512x20` → low bandwidth / long range |
| 2 | **Azimuth window** (`azimuth_window_start/end`) | Crops the horizontal field-of-view in firmware so the sensor streams only the angles you care about, saving ∼proportional bandwidth. | Start/end in **millidegrees** (-180,000 – +180,000)<br>Great for forward-facing robots |
| 3 | **UDP destination (`udp_dest`)** | Tells the sensor *where* to send packets. Required for any network topology. | Unicast IP or **multicast-group** (MTP) |
| 4 | **Return profile (`udp_profile_lidar`)** | Choose *which* channels travel: single-return vs dual-return, low-data-rate, FUSA, etc. Trades detail vs bandwidth. | `RNG19_RFL8_SIG16_NIR16` (default single)<br>`RNG19_RFL8_SIG16_NIR16_DUAL` (dual returns, 2× packets) |
| 5 | **Ports (`lidar_port/imu_port`)** | Non-default ports for multiple sensors or firewall tunneling. | Any free UDP port (≥ 1024) |
| 6 | **Time source (`timestamp_mode`)** | Locks packets to GPS-PTP or SYNC-IN. Critical for multi-sensor fusion. | `TIME_FROM_PTP_1588`, `TIME_FROM_SYNC_PULSE_IN`, `TIME_FROM_INTERNAL_OSC` |

### LiDAR Mode Deep Dive — Resolution × Frame Rate

The `lidar_mode` parameter is the **single most important setting** as it determines:

- **Point density**: Higher resolution = more points per scan
- **Update rate**: Higher frame rate = more frequent updates  
- **Bandwidth**: More points/faster rate = higher network load
- **Range performance**: Lower resolution modes can achieve longer range
- **CPU load**: More points = more processing required

| Mode | Resolution | Frame Rate | Points/Scan | Bandwidth | Best For |
|------|------------|------------|-------------|-----------|----------|
| `4096x5` | 4096 × 128 | 5 Hz | ~524k | Highest | **Mapping, surveying** |
| `2048x10` | 2048 × 128 | 10 Hz | ~262k | High | **Detailed mapping, inspection** |
| `1024x20` | 1024 × 128 | 20 Hz | ~131k | Medium | **⭐ Balanced: SLAM, navigation** |
| `1024x10` | 1024 × 128 | 10 Hz | ~131k | Medium-Low | **Long-range navigation** |
| `512x20` | 512 × 128 | 20 Hz | ~65k | Low | **Fast autonomy, low bandwidth** |
| `512x10` | 512 × 128 | 10 Hz | ~65k | Lowest | **Maximum range, minimal data** |

**💡 Recommendation**: Start with `1024x20` for most applications — it provides excellent balance of detail and update rate.

### Azimuth Window — Field of View Optimization

Configure the sensor's horizontal field of view to reduce unnecessary data:

```yaml
azimuth_window_start: -60000    # -60° (millidegrees)
azimuth_window_end: 60000       # +60° (120° total FOV)
```

**Coordinate System**:
- **0°** (0): Positive X-axis (sensor front)
- **90°** (90000): Positive Y-axis (sensor left)  
- **180°** (180000): Negative X-axis (sensor back)
- **-90°** (-90000): Negative Y-axis (sensor right)

**Bandwidth Savings**: A 120° window saves ~67% bandwidth compared to full 360°.

### Network Configuration

Critical for proper data delivery:

```yaml
udp_dest: "192.168.1.100"      # Your host computer IP
lidar_port: 7502               # LiDAR data port
imu_port: 7503                 # IMU data port
```

For **multicast** (multiple receivers):
```yaml
udp_dest: "239.1.2.3"          # Multicast group IP
```

### Data Format Selection

Choose packet format based on your needs:

| Profile | Returns | Use Case | Bandwidth |
|---------|---------|----------|-----------|
| `RNG19_RFL8_SIG16_NIR16` | Single | Standard applications | 1× |
| `RNG19_RFL8_SIG16_NIR16_DUAL` | Dual | Vegetation, thin objects | 2× |
| `FUSA_RNG15_RFL8_NIR8_DUAL` | Dual + Safety | Automotive/safety critical | 2× |
| `LEGACY` | Single | Older software compatibility | 1× |

**Dual returns** are essential for:
- Detecting vegetation and foliage
- Seeing through chain-link fences
- Improving detection of thin objects (wires, poles)

### ⚙️ How to Configure — Two Methods

#### Method 1: ROS2 Parameters (Recommended)

Edit `config/ouster_config.yaml` and restart the configurator:

```yaml
ouster_configurator:
  ros__parameters:
    sensor_hostname: "169.254.223.207"
    lidar_mode: "1024x20"              # Set resolution×framerate
    azimuth_window_start: -90000       # -90° start
    azimuth_window_end: 90000          # +90° end (180° FOV)
    udp_dest: "192.168.1.100"
    udp_profile_lidar: "RNG19_RFL8_SIG16_NIR16"
    timestamp_mode: "TIME_FROM_INTERNAL_OSC"
    auto_apply_on_startup: true
```

#### Method 2: Direct ouster-cli Commands

```bash
# High-detail mode with limited FOV for autonomous navigation
ouster-cli source 169.254.223.207 config \
    lidar_mode 1024x20 \
    azimuth_window_start -90000 \
    azimuth_window_end 90000 \
    udp_dest 192.168.1.100

# Maximum range mode for long-distance scanning  
ouster-cli source 169.254.223.207 config \
    lidar_mode 512x10 \
    azimuth_window_start -180000 \
    azimuth_window_end 180000

# Dual-return mode for vegetation work
ouster-cli source 169.254.223.207 config \
    udp_profile_lidar RNG19_RFL8_SIG16_NIR16_DUAL
```

### Configuration Persistence

- **ouster-cli changes**: Persistent across power cycles
- **ROS parameters**: Applied on node startup, can override persistent settings
- **Best practice**: Use CLI for "factory" settings, ROS params for session-specific changes

### ⚠️ Important Gotchas

1. **Sensor reboot**: Changing `lidar_mode`, `udp_profile`, or `timestamp_mode` triggers a brief sensor reconnection
2. **Millidegrees**: Azimuth angles use millidegrees (360° = 360,000)
3. **Single UDP destination**: Unless using multicast (MTP), sensor can only send to one destination
4. **Dual-return bandwidth**: Doubles packet count — ensure network/disk can handle the load
5. **Parameter precedence**: ROS configurator overrides persistent sensor settings

---

## Usage

### Prerequisites

**IMPORTANT**: This package can work in two modes:

1. **With Live Sensor**: Configure and stream from a live Ouster sensor
2. **Replay Mode**: Process recorded bag files (configuration features disabled)

### Option 1: Complete Setup with Live Sensor Configuration

Launch both the configurator and point cloud slicer:

```bash
# Configure sensor and start filtering (recommended)
ros2 launch ouster_pointcloud_slice slice_with_config.launch.py \
    sensor_hostname:=169.254.223.207
```

This will:
1. **Auto-configure** the sensor with optimal settings
2. **Start filtering** the point cloud data
3. **Provide services** for live configuration changes

### Option 2: Basic Point Cloud Filtering Only

If you just want point cloud filtering without sensor configuration:

1. **First, launch the Ouster ROS driver separately:**
```bash
# For live sensor (replace with your sensor's IP)
ros2 launch ouster_ros driver.launch.py sensor_hostname:=<SENSOR_IP>

# OR for bag file replay
ros2 bag play <your_bag_file.bag>
```

2. **Verify the Ouster driver is publishing:**
```bash
ros2 topic hz /ouster/points
# Should show ~10-20 Hz publication rate
```

3. **Start the point cloud slicer:**
```bash
ros2 launch ouster_pointcloud_slice slice.launch.py
```

### 🔧 Live Sensor Configuration

When using the complete setup, you can configure the sensor in real-time:

#### Using ROS2 Services

```bash
# Apply current configuration to sensor
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Get current sensor information
ros2 service call /ouster_configurator/get_sensor_info std_srvs/srv/Trigger

# Reset sensor to factory defaults
ros2 service call /ouster_configurator/reset_config std_srvs/srv/Trigger
```

#### Using ROS2 Parameters

```bash
# Change to high-resolution mode
ros2 param set /ouster_configurator lidar_mode "2048x10"
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Set forward-facing 90° FOV  
ros2 param set /ouster_configurator azimuth_window_start -45000
ros2 param set /ouster_configurator azimuth_window_end 45000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Switch to dual-return mode for vegetation
ros2 param set /ouster_configurator udp_profile_lidar "RNG19_RFL8_SIG16_NIR16_DUAL"
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```

#### Configuration Monitoring

```bash
# Monitor configuration status
ros2 topic echo /ouster_configurator/status

# View current sensor configuration
ros2 topic echo /ouster_configurator/current_config
```

### 🎯 Point Cloud Filtering Configuration

Edit `config/slice_params.yaml` to customize point cloud filtering:

```yaml
ouster_slice:
  ros__parameters:
    min_range:      0.0      # minimum distance (meters)
    max_range:      5.0      # maximum distance (meters)  
    min_angle_deg:  -180.0   # minimum azimuth angle (degrees)
    max_angle_deg:  180.0    # maximum azimuth angle (degrees)
    input_topic:    /ouster/points
    output_topic:   /ouster/points_filtered
```

**Default Parameters:**
- Range: 0.0 - 5.0 meters
- Angle: -180° - 180° (full 360° view)
- Input: `/ouster/points`
- Output: `/ouster/points_filtered`

### Live Parameter Updates

Change filtering parameters in real-time:

```bash
# Adjust range filtering for close objects
ros2 param set /ouster_slice min_range 2.0
ros2 param set /ouster_slice max_range 8.0

# Create forward-facing 90° cone
ros2 param set /ouster_slice min_angle_deg -45.0
ros2 param set /ouster_slice max_angle_deg 45.0

# Monitor specific sector (e.g., left side)
ros2 param set /ouster_slice min_angle_deg 45.0
ros2 param set /ouster_slice max_angle_deg 135.0
```

## 📊 Visualization with RViz2

To visualize both original and filtered point clouds:

1. **Ensure your setup is running:**
```bash
# Complete setup with configuration
ros2 launch ouster_pointcloud_slice slice_with_config.launch.py sensor_hostname:=169.254.223.207

# OR basic filtering only
ros2 launch ouster_pointcloud_slice slice.launch.py
```

2. **Launch RViz2:**
```bash
rviz2
```

3. **Configure RViz2:**
   - Set **Fixed Frame** to `os_sensor` or `os_lidar`
   - Add **PointCloud2** displays:
     - **Topic 1**: `/ouster/points` (original data) — Color: White/Gray
     - **Topic 2**: `/ouster/points_filtered` (filtered sector) — Color: Red/Green
   - **Optional**: Add **String** displays for configuration monitoring:
     - `/ouster_configurator/status` — Configuration status updates
     - `/ouster_configurator/current_config` — Live sensor configuration

4. **Test live configuration:**
```bash
# Change sensor to forward-facing high-res mode
ros2 param set /ouster_configurator lidar_mode "2048x10"
ros2 param set /ouster_configurator azimuth_window_start -45000  # -45°
ros2 param set /ouster_configurator azimuth_window_end 45000     # +45°
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Adjust point cloud filtering
ros2 param set /ouster_slice min_angle_deg -30.0
ros2 param set /ouster_slice max_angle_deg 30.0
ros2 param set /ouster_slice max_range 10.0
```

## 🔍 Performance Testing & Monitoring

### Check Point Cloud Processing
```bash
# Verify filtering performance
ros2 topic hz /ouster/points_filtered
# Expected: Matches sensor rate (10-20 Hz depending on lidar_mode)

ros2 topic delay /ouster/points_filtered  
# Expected: ~2-3 ms processing latency

# Monitor filtering statistics in logs
ros2 node info /ouster_slice
```

### Check Sensor Configuration
```bash
# Verify sensor status
ros2 service call /ouster_configurator/get_sensor_info std_srvs/srv/Trigger

# Monitor configuration changes
ros2 topic hz /ouster_configurator/status
ros2 topic echo /ouster_configurator/current_config

# Check current sensor parameters
ros2 param list /ouster_configurator
ros2 param get /ouster_configurator lidar_mode
```

### Performance by LiDAR Mode
| Mode | Expected Hz | Point Count | Latency | Best For |
|------|-------------|-------------|---------|----------|
| `512x10` | 10 Hz | ~65k points | <2 ms | Max range, low CPU |
| `512x20` | 20 Hz | ~65k points | <2 ms | Fast autonomy |
| `1024x10` | 10 Hz | ~131k points | ~2 ms | Balanced performance |
| `1024x20` | 20 Hz | ~131k points | ~3 ms | **⭐ Recommended** |
| `2048x10` | 10 Hz | ~262k points | ~5 ms | High detail mapping |

### Monitor System Resources
```bash
# Check CPU usage
htop

# Monitor network bandwidth (if using live sensor)
iftop -i <network_interface>

# Check ROS2 node resource usage
ros2 daemon stop && ros2 daemon start --ros-args --log-level debug
```

## 🎮 Example Use Cases & Configurations

### 🚗 Autonomous Navigation
```bash
# Fast, forward-facing configuration
ros2 param set /ouster_configurator lidar_mode "1024x20"
ros2 param set /ouster_configurator azimuth_window_start -90000  # -90°
ros2 param set /ouster_configurator azimuth_window_end 90000     # +90°
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Filter for close obstacles
ros2 param set /ouster_slice min_range 0.5
ros2 param set /ouster_slice max_range 15.0
ros2 param set /ouster_slice min_angle_deg -45.0
ros2 param set /ouster_slice max_angle_deg 45.0
```

### 🗺️ High-Detail Mapping
```bash
# Maximum resolution for detailed scanning
ros2 param set /ouster_configurator lidar_mode "2048x10"
ros2 param set /ouster_configurator azimuth_window_start -180000  # Full 360°
ros2 param set /ouster_configurator azimuth_window_end 180000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Minimal filtering for complete data
ros2 param set /ouster_slice min_range 0.1
ros2 param set /ouster_slice max_range 50.0
ros2 param set /ouster_slice min_angle_deg -180.0
ros2 param set /ouster_slice max_angle_deg 180.0
```

### 🌿 Vegetation Detection
```bash
# Dual-return mode for seeing through foliage
ros2 param set /ouster_configurator udp_profile_lidar "RNG19_RFL8_SIG16_NIR16_DUAL"
ros2 param set /ouster_configurator lidar_mode "1024x10"
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Process both returns
ros2 param set /ouster_slice min_range 1.0
ros2 param set /ouster_slice max_range 30.0
```

### 🔋 Low-Bandwidth / Long Range
```bash
# Minimal data mode for maximum range
ros2 param set /ouster_configurator lidar_mode "512x10"
ros2 param set /ouster_configurator azimuth_window_start -60000   # 120° FOV
ros2 param set /ouster_configurator azimuth_window_end 60000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Long-range filtering
ros2 param set /ouster_slice min_range 5.0
ros2 param set /ouster_slice max_range 100.0
```

### 🤖 Multi-Robot Systems
```bash
# Configure different sectors for different robots
# Robot 1: Front sector
ros2 param set /ouster_slice min_angle_deg -45.0
ros2 param set /ouster_slice max_angle_deg 45.0

# Robot 2: Left sector  
ros2 param set /ouster_slice min_angle_deg 45.0
ros2 param set /ouster_slice max_angle_deg 135.0

# Robot 3: Right sector
ros2 param set /ouster_slice min_angle_deg -135.0
ros2 param set /ouster_slice max_angle_deg -45.0
```

- **Obstacle Avoidance**: Filter forward-facing points for navigation
- **Sensor Fusion**: Reduce point cloud size before camera alignment  
- **Performance Optimization**: Process only relevant sectors for real-time applications
- **Multi-Robot Systems**: Different robots monitor different sectors
- **Vegetation Mapping**: Use dual-return mode for forest/agriculture applications
- **Long-Range Detection**: Optimize for maximum sensing distance

## Technical Details

### Point Cloud Processing Algorithm
1. **Range Filtering**: PCL PassThrough filter on distance
2. **Azimuth Filtering**: Manual filtering using `atan2(y, x)`
3. **Output**: Cleaned point cloud with original timestamps and frame_id

### Sensor Configuration System
1. **Parameter Management**: ROS2 parameters mapped to ouster-cli commands
2. **Service Interface**: Trigger-based services for applying/querying configuration
3. **Auto-Apply**: Optional automatic configuration on node startup
4. **Status Monitoring**: Real-time feedback via topics

### Coordinate System
- **0°**: Positive X-axis (sensor front)
- **90°**: Positive Y-axis (sensor left)
- **180°**: Negative X-axis (sensor back)
- **270°**: Negative Y-axis (sensor right)

### Performance Characteristics
- **Input Rate**: 5-20 Hz (depending on lidar_mode)
- **Output Rate**: Matches input rate
- **Processing Latency**: 2-5 ms (depends on point count)
- **Memory Usage**: Low (streaming processing)
- **Configuration Time**: 2-5 seconds (sensor reboot for major changes)

## Package Structure

```
ouster_pointcloud_slice/
├── src/
│   └── pointcloud_slice.cpp          # Main filtering node (C++)
├── scripts/
│   └── ouster_configurator.py        # 🆕 Sensor configuration node (Python)
├── launch/
│   ├── slice.launch.py               # Basic point cloud filtering
│   └── slice_with_config.launch.py   # 🆕 Complete setup with configuration
├── config/
│   ├── slice_params.yaml             # Point cloud filtering parameters  
│   └── ouster_config.yaml            # 🆕 Sensor configuration parameters
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package manifest
└── README.md                         # This documentation
```

## 🔧 ROS2 Interface

### Nodes

#### `/ouster_slice` (Point Cloud Filtering)
- **Type**: C++ node
- **Function**: Real-time point cloud filtering
- **Subscriptions**: `/ouster/points` (sensor_msgs/PointCloud2)
- **Publications**: `/ouster/points_filtered` (sensor_msgs/PointCloud2)
- **Parameters**: `min_range`, `max_range`, `min_angle_deg`, `max_angle_deg`, `input_topic`, `output_topic`

#### `/ouster_configurator` (Sensor Configuration) 🆕
- **Type**: Python node  
- **Function**: Ouster LiDAR parameter management
- **Services**: 
  - `~/apply_config` (std_srvs/Trigger) — Apply current parameters to sensor
  - `~/get_sensor_info` (std_srvs/Trigger) — Get sensor metadata and config
  - `~/reset_config` (std_srvs/Trigger) — Reset sensor to factory defaults
- **Publications**:
  - `~/status` (std_msgs/String) — Configuration status updates
  - `~/current_config` (std_msgs/String) — Live sensor configuration
- **Parameters**: `sensor_hostname`, `lidar_mode`, `azimuth_window_start/end`, `udp_dest`, `udp_profile_lidar`, etc.

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/ouster/points` | PointCloud2 | Input | Raw LiDAR data from Ouster driver |
| `/ouster/points_filtered` | PointCloud2 | Output | Filtered point cloud |
| `/ouster_configurator/status` | String | Output | Configuration status messages |
| `/ouster_configurator/current_config` | String | Output | Current sensor configuration JSON |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/ouster_configurator/apply_config` | Trigger | Apply ROS parameters to sensor |
| `/ouster_configurator/get_sensor_info` | Trigger | Retrieve sensor metadata |
| `/ouster_configurator/reset_config` | Trigger | Reset to factory defaults |

## Troubleshooting

### Point Cloud Filtering Issues

#### No Output on `/ouster/points_filtered`
- Check if `/ouster/points` is being published: `ros2 topic hz /ouster/points`
- Verify the slicer node is running: `ros2 node list | grep ouster_slice`  
- Check parameter values aren't too restrictive:
  ```bash
  ros2 param get /ouster_slice min_range
  ros2 param get /ouster_slice max_range
  ros2 param get /ouster_slice min_angle_deg  
  ros2 param get /ouster_slice max_angle_deg
  ```

#### High Processing Latency
- Check system CPU usage: `htop`
- Verify PCL is properly installed
- Consider reducing sensor resolution: `ros2 param set /ouster_configurator lidar_mode "512x10"`
- Reduce point cloud density with azimuth window: 
  ```bash
  ros2 param set /ouster_configurator azimuth_window_start -90000
  ros2 param set /ouster_configurator azimuth_window_end 90000
  ```

#### Parameter Updates Not Working
- Confirm node is running: `ros2 node info /ouster_slice`
- Check parameter exists: `ros2 param list /ouster_slice`

### Sensor Configuration Issues

#### Configuration Commands Fail
- **Check ouster-sdk installation**: `pip show ouster-sdk`
- **Verify sensor connectivity**: `ping 169.254.223.207`
- **Test ouster-cli manually**: `ouster-cli source 169.254.223.207 metadata`
- **Check sensor IP**: Ensure `sensor_hostname` parameter is correct

#### "No valid scans received" Error
- **Network issue**: Verify `udp_dest` matches your host IP: `ip addr show`
- **Port conflicts**: Check ports aren't in use: `ss -ulnp | grep 7502`
- **Firewall**: Ensure UDP ports 7502/7503 are open
- **Multiple clients**: Only one client can receive UDP data unless using multicast

#### Sensor Won't Apply Configuration
- **Firmware compatibility**: Check sensor firmware version: 
  ```bash
  ros2 service call /ouster_configurator/get_sensor_info std_srvs/srv/Trigger
  ```
- **Invalid parameters**: Check logs for specific error messages
- **Sensor busy**: Ensure no other processes are configuring the sensor
- **Network timeout**: Increase timeout if using slow network connection

#### Configuration Doesn't Persist
- **Expected behavior**: ROS parameters override sensor persistent settings on startup
- **To make persistent**: Use ouster-cli directly: `ouster-cli source <IP> config <param> <value>`
- **Check persistence**: Power cycle sensor and check settings

### Network and Performance Issues

#### Low Frame Rate / Dropped Packets
- **Check network bandwidth**: `iftop -i <interface>`
- **Reduce data rate**: Lower resolution mode or smaller azimuth window
- **UDP buffer size**: Increase system UDP buffers:
  ```bash
  sudo sysctl -w net.core.rmem_max=268435456
  sudo sysctl -w net.core.rmem_default=268435456
  ```

#### Multiple Sensors Interference
- **Use different ports**: Configure each sensor with unique `lidar_port`/`imu_port`
- **Use multicast**: Set sensors to different multicast groups
- **Separate networks**: Use different network interfaces/VLANs

### Common Error Messages

| Error | Cause | Solution |
|-------|-------|----------|
| `ouster-cli not found` | Missing ouster-sdk | `pip install ouster-sdk` |
| `Connection timeout` | Network/firewall issue | Check IP, ping sensor, verify UDP ports |
| `Invalid parameter value` | Wrong parameter type/range | Check sensor manual for valid values |
| `Sensor reboot required` | Major config change | Wait 10-15 seconds for sensor to restart |
| `No valid scans received` | UDP destination wrong | Check `udp_dest` matches your host IP |

### Debug Commands

```bash
# Check all running nodes
ros2 node list

# Monitor all topics
ros2 topic list
ros2 topic hz /ouster/points
ros2 topic hz /ouster/points_filtered

# Check parameter values
ros2 param list /ouster_configurator
ros2 param list /ouster_slice

# View detailed node information
ros2 node info /ouster_configurator
ros2 node info /ouster_slice

# Test sensor connectivity
ping <sensor_ip>
ouster-cli source <sensor_ip> metadata

# Check network activity
sudo tcpdump -i any port 7502 or port 7503
```

## Integration Examples

### Integration with Ouster Bridge

This package works seamlessly with the main `ouster_bridge` package for localization:

```bash
# Terminal 1: Complete Ouster setup with configuration
ros2 launch ouster_pointcloud_slice slice_with_config.launch.py sensor_hostname:=169.254.223.207

# Terminal 2: Localization with visualization  
ros2 launch ouster_bridge localize_with_bridge.launch.py
```

### Integration with Navigation Stack

```bash
# Configure for autonomous navigation
ros2 param set /ouster_configurator lidar_mode "1024x20"
ros2 param set /ouster_configurator azimuth_window_start -90000
ros2 param set /ouster_configurator azimuth_window_end 90000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Filter for navigation-relevant data
ros2 param set /ouster_slice min_range 0.5
ros2 param set /ouster_slice max_range 15.0
ros2 param set /ouster_slice min_angle_deg -60.0
ros2 param set /ouster_slice max_angle_deg 60.0

# Launch navigation stack
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
```

### Integration with SLAM

```bash
# High-resolution mapping configuration
ros2 param set /ouster_configurator lidar_mode "2048x10"
ros2 param set /ouster_configurator azimuth_window_start -180000
ros2 param set /ouster_configurator azimuth_window_end 180000
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Minimal filtering for SLAM
ros2 param set /ouster_slice min_range 0.1
ros2 param set /ouster_slice max_range 50.0

# Launch SLAM (example with slam_toolbox)
ros2 launch slam_toolbox online_async_launch.py
```

### Multi-Sensor Setup

```bash
# Sensor 1: Front-facing
ros2 launch ouster_pointcloud_slice slice_with_config.launch.py \
    sensor_hostname:=169.254.223.207 \
    enable_configurator:=true

# Configure for forward sector
ros2 param set /ouster_configurator azimuth_window_start -45000
ros2 param set /ouster_configurator azimuth_window_end 45000
ros2 param set /ouster_configurator lidar_port 7502
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger

# Sensor 2: Rear-facing (different IP and ports)
ros2 launch ouster_pointcloud_slice slice_with_config.launch.py \
    sensor_hostname:=169.254.223.208 \
    enable_configurator:=true

# Configure for rear sector with different ports
ros2 param set /ouster_configurator azimuth_window_start 135000
ros2 param set /ouster_configurator azimuth_window_end -135000
ros2 param set /ouster_configurator lidar_port 7504
ros2 param set /ouster_configurator imu_port 7505
ros2 service call /ouster_configurator/apply_config std_srvs/srv/Trigger
```

## Contributing

Contributions welcome! Areas for improvement:

### Current Roadmap
- **Additional filtering methods**: Height-based filtering, intensity thresholding
- **GPU acceleration**: CUDA-based filtering for larger point clouds  
- **Multi-sector filtering**: Simultaneous filtering of multiple angular regions
- **Dynamic reconfigure GUI**: Web-based or RQT configuration interface
- **Advanced sensor features**: Beam masking, signal multiplier control
- **Configuration profiles**: Save/load common sensor configurations
- **Automatic optimization**: AI-driven parameter tuning based on environment

### How to Contribute
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Commit your changes: `git commit -m 'Add amazing feature'`
4. Push to the branch: `git push origin feature/amazing-feature`
5. Open a Pull Request

### Development Setup
```bash
# Clone for development
git clone https://github.com/hassannmurtada/ouster_pointcloud_slice.git
cd ouster_pointcloud_slice

# Install development dependencies
pip install ouster-sdk pytest black flake8

# Run tests
colcon test --packages-select ouster_pointcloud_slice

# Format code
black scripts/
```

## License

MIT License - see LICENSE file for details.
