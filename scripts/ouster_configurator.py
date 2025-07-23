#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
import subprocess
import json
import threading
import time

class OusterConfigurator(Node):
    """
    ROS2 node for configuring Ouster LiDAR parameters via ouster-cli.
    Provides services and parameters for live sensor configuration.
    """
    
    def __init__(self):
        super().__init__('ouster_configurator')
        
        # Declare parameters
        self.declare_parameter('sensor_hostname', '169.254.223.207')
        self.declare_parameter('lidar_mode', '1024x20')
        self.declare_parameter('azimuth_window_start', -180000)  # millidegrees
        self.declare_parameter('azimuth_window_end', 180000)     # millidegrees
        self.declare_parameter('udp_dest', '169.254.223.100')
        self.declare_parameter('udp_profile_lidar', 'RNG19_RFL8_SIG16_NIR16')
        self.declare_parameter('timestamp_mode', 'TIME_FROM_INTERNAL_OSC')
        self.declare_parameter('lidar_port', 7502)
        self.declare_parameter('imu_port', 7503)
        self.declare_parameter('auto_apply_on_startup', True)
        
        # Get current parameters
        self.sensor_hostname = self.get_parameter('sensor_hostname').value
        
        # Create services
        self.apply_config_srv = self.create_service(
            Trigger, '~/apply_config', self.apply_config_callback)
        self.get_sensor_info_srv = self.create_service(
            Trigger, '~/get_sensor_info', self.get_sensor_info_callback)
        self.reset_config_srv = self.create_service(
            Trigger, '~/reset_config', self.reset_config_callback)
        
        # Create publishers for status updates
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.config_pub = self.create_publisher(String, '~/current_config', 10)
        
        # Parameter callback for live updates
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Auto-apply configuration on startup if enabled
        if self.get_parameter('auto_apply_on_startup').value:
            self.get_logger().info("Auto-applying configuration on startup...")
            threading.Thread(target=self.delayed_auto_apply, daemon=True).start()
        
        self.get_logger().info(f"Ouster Configurator started for sensor: {self.sensor_hostname}")
        self.get_logger().info("Available services:")
        self.get_logger().info("  ~/apply_config - Apply current ROS parameters to sensor")
        self.get_logger().info("  ~/get_sensor_info - Get current sensor configuration")
        self.get_logger().info("  ~/reset_config - Reset sensor to factory defaults")
    
    def delayed_auto_apply(self):
        """Apply configuration after a short delay to ensure node is fully initialized"""
        time.sleep(2.0)
        self.apply_current_config()
    
    def parameter_callback(self, params):
        """Handle parameter changes"""
        result = rclpy.parameter.SetParametersResult()
        result.successful = True
        
        # Update sensor hostname if changed
        for param in params:
            if param.name == 'sensor_hostname':
                self.sensor_hostname = param.value
                self.get_logger().info(f"Sensor hostname updated to: {self.sensor_hostname}")
        
        return result
    
    def run_ouster_cli(self, command_args):
        """Execute ouster-cli command and return result"""
        try:
            cmd = ['ouster-cli', 'source', self.sensor_hostname] + command_args
            self.get_logger().info(f"Executing: {' '.join(cmd)}")
            
            result = subprocess.run(
                cmd, 
                capture_output=True, 
                text=True, 
                timeout=30
            )
            
            if result.returncode == 0:
                return True, result.stdout
            else:
                return False, result.stderr
                
        except subprocess.TimeoutExpired:
            return False, "Command timed out after 30 seconds"
        except FileNotFoundError:
            return False, "ouster-cli not found. Please install ouster-sdk: pip install ouster-sdk"
        except Exception as e:
            return False, f"Error executing command: {str(e)}"
    
    def apply_current_config(self):
        """Apply current ROS parameters to the sensor"""
        self.get_logger().info("Applying configuration to sensor...")
        
        # Get current parameter values
        lidar_mode = self.get_parameter('lidar_mode').value
        azimuth_start = self.get_parameter('azimuth_window_start').value
        azimuth_end = self.get_parameter('azimuth_window_end').value
        udp_dest = self.get_parameter('udp_dest').value
        udp_profile = self.get_parameter('udp_profile_lidar').value
        timestamp_mode = self.get_parameter('timestamp_mode').value
        lidar_port = self.get_parameter('lidar_port').value
        imu_port = self.get_parameter('imu_port').value
        
        # Build configuration command
        config_args = ['config']
        config_args.extend(['lidar_mode', lidar_mode])
        config_args.extend(['azimuth_window_start', str(azimuth_start)])
        config_args.extend(['azimuth_window_end', str(azimuth_end)])
        config_args.extend(['udp_dest', udp_dest])
        config_args.extend(['udp_profile_lidar', udp_profile])
        config_args.extend(['timestamp_mode', timestamp_mode])
        config_args.extend(['lidar_port', str(lidar_port)])
        config_args.extend(['imu_port', str(imu_port)])
        
        success, output = self.run_ouster_cli(config_args)
        
        if success:
            self.get_logger().info("Configuration applied successfully!")
            self.publish_status("Configuration applied successfully")
            return True, "Configuration applied successfully"
        else:
            self.get_logger().error(f"Failed to apply configuration: {output}")
            self.publish_status(f"Configuration failed: {output}")
            return False, output
    
    def apply_config_callback(self, request, response):
        """Service callback for applying configuration"""
        success, message = self.apply_current_config()
        response.success = success
        response.message = message
        return response
    
    def get_sensor_info_callback(self, request, response):
        """Service callback for getting sensor information"""
        self.get_logger().info("Getting sensor information...")
        
        # Get sensor metadata
        success, output = self.run_ouster_cli(['metadata'])
        
        if success:
            try:
                # Parse metadata and extract key information
                metadata = json.loads(output)
                info = {
                    'firmware_version': metadata.get('firmware_version', 'Unknown'),
                    'prod_line': metadata.get('prod_line', 'Unknown'),
                    'prod_pn': metadata.get('prod_pn', 'Unknown'),
                    'prod_sn': metadata.get('prod_sn', 'Unknown'),
                    'lidar_mode': metadata.get('lidar_mode', 'Unknown'),
                    'azimuth_window': metadata.get('azimuth_window', 'Unknown'),
                    'udp_dest': metadata.get('udp_dest', 'Unknown'),
                    'udp_profile_lidar': metadata.get('udp_profile_lidar', 'Unknown'),
                    'timestamp_mode': metadata.get('timestamp_mode', 'Unknown'),
                    'lidar_port': metadata.get('lidar_port', 'Unknown'),
                    'imu_port': metadata.get('imu_port', 'Unknown')
                }
                
                info_str = json.dumps(info, indent=2)
                self.get_logger().info(f"Sensor info retrieved:\n{info_str}")
                self.publish_config(info_str)
                
                response.success = True
                response.message = info_str
            except json.JSONDecodeError:
                response.success = False
                response.message = f"Failed to parse metadata: {output}"
        else:
            response.success = False
            response.message = f"Failed to get sensor info: {output}"
        
        return response
    
    def reset_config_callback(self, request, response):
        """Service callback for resetting sensor configuration"""
        self.get_logger().info("Resetting sensor to factory defaults...")
        
        # Reset to factory defaults
        success, output = self.run_ouster_cli(['config', 'reset'])
        
        if success:
            self.get_logger().info("Sensor reset to factory defaults")
            self.publish_status("Sensor reset to factory defaults")
            response.success = True
            response.message = "Sensor reset to factory defaults"
        else:
            self.get_logger().error(f"Failed to reset sensor: {output}")
            self.publish_status(f"Reset failed: {output}")
            response.success = False
            response.message = output
        
        return response
    
    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[{self.get_clock().now().to_msg()}] {message}"
        self.status_pub.publish(msg)
    
    def publish_config(self, config_str):
        """Publish current configuration"""
        msg = String()
        msg.data = config_str
        self.config_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OusterConfigurator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Ouster Configurator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
