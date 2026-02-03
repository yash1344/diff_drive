#ifndef STRETCH_FLEX_HPP
#define STRETCH_FLEX_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pose_registry_interfaces/srv/store_pose.hpp>
#include <pose_registry_interfaces/srv/retrieve_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <stretch_flex_interfaces/msg/save_pose_interface.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <string>

namespace stretch_flex
{
  class StretchFlex : public rclcpp::Node
  {
  public:
    StretchFlex();
    ~StretchFlex();

  private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp::Subscription<stretch_flex_interfaces::msg::SavePoseInterface>::SharedPtr subscription_save_current_pose_; // Subscription for save_current_pose service
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_navigate_to_pose_goal_;                       // Subscription for navigate_to_pose action

    void save_current_pose_callback(const stretch_flex_interfaces::msg::SavePoseInterface::SharedPtr msg);
    void navigate_to_pose_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Client<pose_registry_interfaces::srv::StorePose>::SharedPtr store_pose_client_;       // Client to call pose_registry store_pose service
    rclcpp::Client<pose_registry_interfaces::srv::RetrievePose>::SharedPtr retrieve_pose_client_; // Client to call pose_registry retrieve_pose service

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_action_client_; // Action client for navigate_to_pose action

    // Action callbacks
    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr &goal_handle);
    void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result);
  };
}
#endif // STRETCH_FLEX_HPP
