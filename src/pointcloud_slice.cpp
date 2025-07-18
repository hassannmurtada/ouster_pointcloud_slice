#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <cmath>

class CloudSlicer : public rclcpp::Node
{
public:
  CloudSlicer() : Node("ouster_slice")
  {
    // Declare parameters with defaults - Full 360° view by default
    declare_parameter<double>("min_range", 0.0);
    declare_parameter<double>("max_range", 5.0);
    declare_parameter<double>("min_angle_deg", -180.0);  // Full 360° coverage
    declare_parameter<double>("max_angle_deg", 180.0);   // Full 360° coverage
    declare_parameter<std::string>("input_topic", "/ouster/points");
    declare_parameter<std::string>("output_topic", "/ouster/points_filtered");

    get_parameters();

    // Subscription uses SensorDataQoS to match Ouster driver (BEST_EFFORT)
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      std::bind(&CloudSlicer::callback, this, std::placeholders::_1)
    );

    // Publisher uses RELIABLE QoS for RViz2 compatibility
    auto reliable_qos = rclcpp::QoS(10);
    reliable_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      params_.output_topic, reliable_qos
    );

    RCLCPP_INFO(get_logger(),
      "Slicer: [%s] → [%s], range=%.1f–%.1f m, ang=%.1f–%.1f°",
      params_.input_topic.c_str(), params_.output_topic.c_str(),
      params_.min_range, params_.max_range,
      params_.min_angle_deg, params_.max_angle_deg
    );

    // Allow live reconfigure
    add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &changes) -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        bool params_changed = false;
        for (auto &c : changes) {
          if (c.get_name() == "min_range") {
            params_.min_range = c.as_double();
            params_changed = true;
          }
          if (c.get_name() == "max_range") {
            params_.max_range = c.as_double();
            params_changed = true;
          }
          if (c.get_name() == "min_angle_deg") {
            params_.min_angle_deg = c.as_double();
            params_changed = true;
          }
          if (c.get_name() == "max_angle_deg") {
            params_.max_angle_deg = c.as_double();
            params_changed = true;
          }
        }
        
        if (params_changed) {
          RCLCPP_INFO(get_logger(),
            "Parameters updated: range=%.1f–%.1f m, ang=%.1f–%.1f°",
            params_.min_range, params_.max_range,
            params_.min_angle_deg, params_.max_angle_deg
          );
        }
        
        return result;
      }
    );
  }

private:
  struct {
    double min_range, max_range;
    double min_angle_deg, max_angle_deg;
    std::string input_topic, output_topic;
  } params_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;

  void get_parameters()
  {
    get_parameter("min_range",     params_.min_range);
    get_parameter("max_range",     params_.max_range);
    get_parameter("min_angle_deg", params_.min_angle_deg);
    get_parameter("max_angle_deg", params_.max_angle_deg);
    get_parameter("input_topic",   params_.input_topic);
    get_parameter("output_topic",  params_.output_topic);
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Get current parameters (this ensures we always use latest values)
    get_parameters();
    
    // Convert ROS → PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // Combined range and azimuth filter in one pass for efficiency
    pcl::PointCloud<pcl::PointXYZI>::Ptr kept(new pcl::PointCloud<pcl::PointXYZI>);
    kept->reserve(cloud->size());
    
    double min_a = params_.min_angle_deg * M_PI / 180.0;
    double max_a = params_.max_angle_deg * M_PI / 180.0;

    size_t input_count = cloud->points.size();
    size_t range_filtered = 0;
    size_t angle_filtered = 0;

    for (const auto &p : cloud->points) {
      // Skip invalid points
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
        
      // Calculate horizontal distance (2D range, excluding Z)
      double range = std::sqrt(p.x * p.x + p.y * p.y);
      
      // Range filter
      if (range < params_.min_range || range > params_.max_range) {
        range_filtered++;
        continue;
      }
        
      // Azimuth filter  
      double az = std::atan2(p.y, p.x);
      if (az < min_a || az > max_a) {
        angle_filtered++;
        continue;
      }
      
      kept->points.push_back(p);
    }

    // Publish filtered cloud
    kept->width = kept->points.size();
    kept->height = 1;
    kept->is_dense = false;

    // Debug output every 50 messages with current parameters
    static int msg_count = 0;
    if (++msg_count % 50 == 0) {
      RCLCPP_INFO(get_logger(),
        "Filtering: %zu→%zu points (filtered %zu by range, %zu by angle) | Using range=%.1f-%.1f m, angle=%.1f-%.1f°",
        input_count, kept->points.size(), range_filtered, angle_filtered,
        params_.min_range, params_.max_range, params_.min_angle_deg, params_.max_angle_deg
      );
    }

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*kept, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudSlicer>());
  rclcpp::shutdown();
  return 0;
}
