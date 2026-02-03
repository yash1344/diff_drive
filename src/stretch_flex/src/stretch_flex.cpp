#include "stretch_flex/stretch_flex.hpp"

using namespace stretch_flex;

StretchFlex::StretchFlex() : rclcpp::Node("stretch_flex")
{
    // Create service server for save_current_pose
    subscription_save_current_pose_ = this->create_subscription<stretch_flex_interfaces::msg::SavePoseInterface>(
        "stretch_flex/save_current_pose",
        10,
        std::bind(
            &StretchFlex::save_current_pose_callback,
            this,
            std::placeholders::_1));

    // Create subscription for navigate_to_pose action
    subscription_navigate_to_pose_goal_ = this->create_subscription<std_msgs::msg::String>(
        "stretch_flex/navigate_to_pose",
        10,
        std::bind(
            &StretchFlex::navigate_to_pose_callback,
            this,
            std::placeholders::_1));

    // Create client to call pose_registry store_pose service
    store_pose_client_ = this->create_client<pose_registry_interfaces::srv::StorePose>(
        "/pose_registry/store_pose");

    // Create client to call pose_registry retrieve_pose service
    retrieve_pose_client_ = this->create_client<pose_registry_interfaces::srv::RetrievePose>(
        "/pose_registry/retrieve_pose");

    // Create action client for navigate_to_pose
    navigate_action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this,
        "/navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "StretchFlex node initialized");
}

StretchFlex::~StretchFlex() {}

void StretchFlex::save_current_pose_callback(const stretch_flex_interfaces::msg::SavePoseInterface::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received request to save current pose as '%s'", msg->pose_name.c_str());

    try
    {
        // Check if pose label is provided
        if (msg->pose_name.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Pose name cannot be empty");
            return;
        }

        // Wait for the store_pose service to be available
        if (!store_pose_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "pose_registry/store_pose service not available");
            return;
        }

        // Create request for pose_registry store service
        auto request = std::make_shared<pose_registry_interfaces::srv::StorePose::Request>();
        request->pose_name = msg->pose_name;

        // Get current pose from tf2
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            auto tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            // Wait for the transform to become available
            if (!tf_buffer->canTransform("map", msg->frame_id, tf2::TimePointZero, tf2::durationFromSec(15.0)))
            {
                RCLCPP_ERROR(this->get_logger(), "Transform from map to %s not available after 15 seconds", msg->frame_id.c_str());
                return;
            }

            transform = tf_buffer->lookupTransform("map", msg->frame_id, tf2::TimePointZero);
            request->pose.header = transform.header;
            request->pose.pose.position.x = transform.transform.translation.x;
            request->pose.pose.position.y = transform.transform.translation.y;
            request->pose.pose.position.z = transform.transform.translation.z;
            request->pose.pose.orientation = transform.transform.rotation;
            request->pose.header.frame_id = transform.header.frame_id;

            RCLCPP_INFO(this->get_logger(), "Current pose obtained from tf2 for frame_id: %s", msg->frame_id.c_str());
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        // Call the pose_registry store service
        store_pose_client_->async_send_request(
            request);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error saving pose: %s", e.what());
    }
}

void StretchFlex::navigate_to_pose_callback(const std_msgs::msg::String::SharedPtr msg)
{
    try
    {
        // Check if pose label is provided
        if (msg->data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Pose name cannot be empty");
            return;
        }

        // Wait for the retrieve_pose service to be available
        if (!retrieve_pose_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "pose_registry/retrieve_pose service not available");
            return;
        }

        // Create request for pose_registry retrieve service
        auto request = std::make_shared<pose_registry_interfaces::srv::RetrievePose::Request>();
        request->pose_name = msg->data;

        // Call the pose_registry retrieve service
        auto future = retrieve_pose_client_->async_send_request(
            request,
            [this](rclcpp::Client<pose_registry_interfaces::srv::RetrievePose>::SharedFuture future)
            {
                auto result = future.get();
                if (!result->success)
                {
                    RCLCPP_WARN(this->get_logger(), "Pose not found in registry");
                    return;
                }

                if (!navigate_action_client_->wait_for_action_server(std::chrono::seconds(5)))
                {
                    RCLCPP_ERROR(this->get_logger(), "navigate_to_pose action server not available");
                    return;
                }

                auto goal_msg = NavigateToPose::Goal();
                goal_msg.pose = result->pose;

                auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
                send_goal_options.goal_response_callback =
                    std::bind(&StretchFlex::goal_response_callback, this, std::placeholders::_1);
                send_goal_options.feedback_callback =
                    std::bind(&StretchFlex::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
                send_goal_options.result_callback =
                    std::bind(&StretchFlex::result_callback, this, std::placeholders::_1);

                navigate_action_client_->async_send_goal(goal_msg, send_goal_options);
            });
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error navigating to pose: %s", e.what());
    }
}

void StretchFlex::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void StretchFlex::feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_DEBUG(this->get_logger(), "Remaining distance: %f m", feedback->distance_remaining);
}

void StretchFlex::result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StretchFlex>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
