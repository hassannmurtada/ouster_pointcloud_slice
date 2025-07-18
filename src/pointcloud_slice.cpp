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
    // Declare parameters with defaults
    declare_parameter<double>("min_range", 1.0);
    declare_parameter<double>("max_range", 5.0);
    declare_parameter<double>("min_angle_deg", 0.0);
    declare_parameter<double>("max_angle_deg", 90.0);
    declare_parameter<std::string>("input_topic", "/ouster/points");
    declare_parameter<std::string>("output_topic", "/ouster/points_filtered");

    get_parameters();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.input_topic, rclcpp::SensorDataQoS(),
      std::bind(&CloudSlicer::callback, this, std::placeholders::_1)
    );

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      params_.output_topic, rclcpp::SensorDataQoS()
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
        for (auto &c : changes) {
          if (c.get_name() == "min_range")      params_.min_range     = c.as_double();
          if (c.get_name() == "max_range")      params_.max_range     = c.as_double();
          if (c.get_name() == "min_angle_deg")  params_.min_angle_deg = c.as_double();
          if (c.get_name() == "max_angle_deg")  params_.max_angle_deg = c.as_double();
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
    // Convert ROS → PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // 1) Range filter
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");  // approximate radial by X; for full radial, skip this
    pass.setFilterLimits(params_.min_range, params_.max_range);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ranged(new pcl::PointCloud<pcl::PointXYZI>);
    pass.filter(*cloud_ranged);

    // 2) Azimuth filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr kept(new pcl::PointCloud<pcl::PointXYZI>);
    kept->reserve(cloud_ranged->size());
    double min_a = params_.min_angle_deg * M_PI / 180.0;
    double max_a = params_.max_angle_deg * M_PI / 180.0;

    for (auto &p : cloud_ranged->points) {
      double az = std::atan2(p.y, p.x);
      if (az >= min_a && az <= max_a)
        kept->points.push_back(p);
    }

    // 3) Publish back
    kept->width = kept->points.size();
    kept->height = 1;

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
