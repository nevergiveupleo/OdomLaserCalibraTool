#include <io.h>
#include <utils.h>

messageIO::messageIO(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // ROS_INFO("Initing IO...");
}

void messageIO::readDataFromBag(const std::string &bag_name, const std::string &laser_topic_name, const std::string &odom_topic_name,
          std::vector<odometerData> &odom_data, std::vector<laserScanData> &laser_data)
{
  rosbag2_cpp::readers::SequentialReader bag;
  RCLCPP_INFO(node_->get_logger(), "%s", colouredString("Opening bag...", YELLOW, REGULAR).c_str());
  rosbag2_cpp::StorageOptions storage_options{.uri = bag_name, .storage_id = "sqlite3"};
  rosbag2_cpp::ConverterOptions converter_options{.input_serialization_format = "cdr", .output_serialization_format = "cdr"};

  try{
    bag.open(storage_options, converter_options);
  }
  catch (std::exception& e) {
    RCLCPP_INFO(node_->get_logger(), "%s", colouredString("ERROR!", RED, BOLD).c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "%s", colouredString("DONE!", GREEN, REGULAR).c_str());
  RCLCPP_INFO(node_->get_logger(), "%s", colouredString("Quering topics bag...", YELLOW, REGULAR).c_str());

  std::vector<std::string> topics;
  topics.push_back(laser_topic_name);
  topics.push_back(odom_topic_name);

  // Prepare serializers
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serializer;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serializer;

  // Read all messages into a vector, then iterate (similar to rosbag::View)
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  while (bag.has_next()) {
    messages.push_back(bag.read_next());
  }

  RCLCPP_INFO(node_->get_logger(), "%s", colouredString("Reading bag data...", YELLOW, REGULAR).c_str());

  for (const auto & serialized_msg : messages) {
    const auto & topic = serialized_msg->topic_name;
    if (topic == laser_topic_name) {
      rclcpp::SerializedMessage rmsg(*serialized_msg->serialized_data);
      sensor_msgs::msg::LaserScan scan;
      scan_serializer.deserialize_message(&rmsg, &scan);

      laserScanData tmp;
      tmp.ranges = scan.ranges;
      tmp.scan_time = scan.scan_time;
      tmp.time_increment = scan.time_increment;
      tmp.timestamp = scan.header.stamp;
      tmp.angle_increment = scan.angle_increment;
      tmp.max_angle = scan.angle_max;
      tmp.min_angle = scan.angle_min;
      tmp.max_range = scan.range_max;
      tmp.min_range = scan.range_min;
      laser_data.push_back(tmp);
    }  else if (topic == odom_topic_name) {
      rclcpp::SerializedMessage rmsg(*serialized_msg->serialized_data);
      nav_msgs::msg::Odometry odom_msg;
      odom_serializer.deserialize_message(&rmsg, &odom_msg);

      odometerData tmp;
      tmp.timestamp = odom_msg.header.stamp;
      // tmp.x = odom_msg.pose.pose.position.x;
      // tmp.y = odom_msg.pose.pose.position.y;
      // tmp.theta = tf2::getYaw(odom_msg.pose.pose.orientation);
      // tmp.linear_velocity << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z;
      // tmp.angular_velocity << odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z;
      double r_l = 0.4;
      double r_r = 0.4;
      double b = 0.7;
      double v = odom_msg.twist.twist.linear.x;
      double omega = odom_msg.twist.twist.angular.z;
      tmp.v_l = (v / r_l) - ((omega * b) / (2 * r_l));
      tmp.v_r = (v / r_r) + ((omega * b) / (2 * r_r));
      odom_data.push_back(tmp);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unknown topic: %s", topic.c_str());
      continue;
    }
  }
//    nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
//    if (odom != NULL) {
//      odometerData tmp;
//      tmp.timestamp = odom->header.stamp;
//      double r_l = 0.4;
//      double r_r = 0.4;
//      double b = 0.7;
//      double v = odom->twist.twist.linear.x;
//      double omega = odom->twist.twist.angular.z;
//      tmp.v_l = (v / r_l) - ((omega * b) / (2 * r_l));
//      tmp.v_r = (v / r_r) + ((omega * b) / (2 * r_r));
//      odom_data.push_back(tmp);
//    }

//    sensor_msgs::JointStateConstPtr odom = m.instantiate<sensor_msgs::JointState>();
//    if (odom != NULL) {
//      odometerData tmp;
//      tmp.timestamp = odom->header.stamp;
//      tmp.v_l = odom->velocity[0];
//      tmp.v_r = odom->velocity[1];
//      odom_data.push_back(tmp);
//    }
//

  // std::cout << "laser size: " << laser_data.size() << '\n' << "odom size: " << odom_data.size().c_str());

  RCLCPP_INFO(node_->get_logger(), "%s", colouredString("Data reading finished!", GREEN, REGULAR).c_str());

  return;

}
