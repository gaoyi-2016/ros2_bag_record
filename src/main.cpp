#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include <yaml-cpp/yaml.h>

class Ros2BagRecordNode : public rclcpp::Node
{
public:
  Ros2BagRecordNode(std::string config_path) : Node("ros2_bag_record_node")
  {
    RCLCPP_INFO(this->get_logger(), "配置文件: %s", config_path.c_str());
    YAML::Node config = YAML::LoadFile(config_path);

    // rosbag
    std::string rosbag_path = config["rosbag_path"].as<std::string>();
    RCLCPP_INFO(this->get_logger(), "rosbag_path: %s", rosbag_path.c_str());

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y_%m_%d_%H_%M_%S");

    rosbag_name_ = rosbag_path + "/my_ros2bag_" + ss.str();
    RCLCPP_INFO(this->get_logger(), "rosbag_name: %s", rosbag_name_.c_str());

    // lidar
    lidar_en_ = config["lidar_en"].as<int>();
    lidar_topic_ = config["lidar_topic"].as<std::vector<std::string>>();

    RCLCPP_INFO(this->get_logger(), "lidar_en: %d", lidar_en_);
    for(int i = 0; i < lidar_en_; i++)
    {
      RCLCPP_INFO(this->get_logger(), "lidar_topic_%d: %s", i, lidar_topic_[i].c_str());
    }

    // imu
    imu_en_ = config["imu_en"].as<int>();
    imu_topic_ = config["imu_topic"].as<std::vector<std::string>>();

    RCLCPP_INFO(this->get_logger(), "imu_en: %d", imu_en_);
    for(int i = 0; i < imu_en_; i++)
    {
      RCLCPP_INFO(this->get_logger(), "imu_topic_%d: %s", i, imu_topic_[i].c_str());
    }

    // pose
    pose_en_ = config["pose_en"].as<int>();
    pose_topic_ = config["pose_topic"].as<std::vector<std::string>>();

    RCLCPP_INFO(this->get_logger(), "pose_en: %d", pose_en_);
    for(int i = 0; i < pose_en_; i++)
    {
      RCLCPP_INFO(this->get_logger(), "pose_topic_%d: %s", i, pose_topic_[i].c_str());
    }

    // rosbag_writer
    rosbag_writer_ = std::make_unique<rosbag2_cpp::Writer>(); 
    rosbag_writer_->open(rosbag_name_); 

    // sub_lidar
    sub_lidar_.resize(lidar_en_);
    for(int i = 0; i < lidar_en_; i++)
    {
      sub_lidar_[i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_[i],
        rclcpp::SensorDataQoS().reliable(),
        [this, i](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {this->subscribe_lidar(msg, i);}
      );
    }

    // sub_imu
    sub_imu_.resize(imu_en_);
    for(int i = 0; i < imu_en_; i++)
    {
      sub_imu_[i] = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_[i],
        rclcpp::SensorDataQoS(),
        [this, i](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {this->subscribe_imu(msg, i);}
      );
    }

    // sub_pose
    sub_pose_.resize(pose_en_);
    for(int i = 0; i < pose_en_; i++)
    {
      sub_pose_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_[i],
        rclcpp::SensorDataQoS().reliable(),
        [this, i](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {this->subscribe_pose(msg, i);}
      );
    }

    RCLCPP_INFO(this->get_logger(), "录制开始, 打开rosbag");
  }

  ~Ros2BagRecordNode()
  {
    RCLCPP_INFO(this->get_logger(), "录制结束, 关闭rosbag");
  }

private:
  void subscribe_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, int num)
  {
    if (msg->data.empty()) 
    {
      RCLCPP_WARN(this->get_logger(), "收到空点云，跳过写入");
      return;
    }

    builtin_interfaces::msg::Time stamp_time = msg->header.stamp;
    double timestamp = stamp_time.sec + stamp_time.nanosec * 1e-9;

    RCLCPP_INFO(this->get_logger(), "lidar_topic[%d] timestamp[%f] 总点数[%d]", num, timestamp, msg->width * msg->height);

    rosbag_writer_->write<sensor_msgs::msg::PointCloud2>(*msg, lidar_topic_[num], msg->header.stamp);
  }

  void subscribe_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg, int num)
  {
    // builtin_interfaces::msg::Time stamp_time = msg->header.stamp;
    // double timestamp = stamp_time.sec + stamp_time.nanosec * 1e-9;

    // RCLCPP_INFO(this->get_logger(), "imu_topic[%d] timestamp[%f]", num, timestamp);

    rosbag_writer_->write<sensor_msgs::msg::Imu>(*msg, imu_topic_[num], msg->header.stamp);
  }

  void subscribe_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg, int num)
  {
    // builtin_interfaces::msg::Time stamp_time = msg->header.stamp;
    // double timestamp = stamp_time.sec + stamp_time.nanosec * 1e-9;

    // RCLCPP_INFO(this->get_logger(), "imu_topic[%d] timestamp[%f]", num, timestamp);

    rosbag_writer_->write<geometry_msgs::msg::PoseStamped>(*msg, "/pose_" + std::to_string(num), msg->header.stamp);
  }

private:
  std::string rosbag_name_;
  std::unique_ptr<rosbag2_cpp::Writer> rosbag_writer_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_lidar_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> sub_imu_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> sub_pose_;
  int lidar_en_, imu_en_, pose_en_;
  std::vector<std::string> lidar_topic_, imu_topic_, pose_topic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  

  std::string config_path = argv[1];

  auto record_rosbag_node = std::make_shared<Ros2BagRecordNode>(config_path); 

  rclcpp::spin(record_rosbag_node); 

  rclcpp::shutdown();  

  return 0;
}