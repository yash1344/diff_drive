#ifndef POSE_REGISTRY_HPP
#define POSE_REGISTRY_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pose_registry_interfaces/srv/store_pose.hpp>
#include <pose_registry_interfaces/srv/retrieve_pose.hpp>
#include <map>
#include <string>
#include <nlohmann/json.hpp>
#include <fstream>

namespace pose_registry
{
  using json = nlohmann::json;

  class PoseRegistry : public rclcpp::Node
  {
  public:
    PoseRegistry();
    ~PoseRegistry();

  private:
    // Data structure to store poses
    std::map<std::string, geometry_msgs::msg::PoseStamped> poses_;

    // File path for persistence
    std::string poses_file_path_;

    // Load poses from JSON file
    void load_poses_from_file(const std::string &poses_file_path_);

    // append pose to JSON file
    void append_pose_to_file(const std::string &pose_name, const geometry_msgs::msg::PoseStamped &pose_stamped);

    // Service callbacks
    void store_pose_callback(
        const std::shared_ptr<pose_registry_interfaces::srv::StorePose::Request> request,
        std::shared_ptr<pose_registry_interfaces::srv::StorePose::Response> response);

    void retrieve_pose_callback(
        const std::shared_ptr<pose_registry_interfaces::srv::RetrievePose::Request> request,
        std::shared_ptr<pose_registry_interfaces::srv::RetrievePose::Response> response);

    // Service servers
    rclcpp::Service<pose_registry_interfaces::srv::StorePose>::SharedPtr store_service_;
    rclcpp::Service<pose_registry_interfaces::srv::RetrievePose>::SharedPtr retrieve_service_;
  };
}
#endif // POSE_REGISTRY_HPP
