#include "pose_registry/pose_registry.hpp"
#include <fstream>
#include <iostream>

using namespace pose_registry;

PoseRegistry::PoseRegistry() : rclcpp::Node("pose_registry")
{

    this->declare_parameter<std::string>("pose_file", "/workspaces/diff_drive_robot/src/pose_registry/config/poses.json");
    poses_file_path_ = this->get_parameter("pose_file").as_string();

    // Load existing poses from .json file
    load_poses_from_file(poses_file_path_);

    // Create service servers
    store_service_ = this->create_service<pose_registry_interfaces::srv::StorePose>(
        "pose_registry/store_pose",
        std::bind(
            &PoseRegistry::store_pose_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    retrieve_service_ = this->create_service<pose_registry_interfaces::srv::RetrievePose>(
        "pose_registry/retrieve_pose",
        std::bind(
            &PoseRegistry::retrieve_pose_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "PoseRegistry node initialized");
    RCLCPP_INFO(this->get_logger(), "Loaded %zu poses from file",
                poses_.size());
}

PoseRegistry::~PoseRegistry() {}

void PoseRegistry::store_pose_callback(
    const std::shared_ptr<pose_registry_interfaces::srv::StorePose::Request> request,
    std::shared_ptr<pose_registry_interfaces::srv::StorePose::Response> response)
{
    try
    {
        if (request->pose_name.empty())
        {
            response->success = false;
            response->message = "Pose name cannot be empty";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Store pose in memory
        poses_[request->pose_name] = request->pose;

        // Save to file
        append_pose_to_file(request->pose_name, request->pose);

        response->success = true;
        response->message = "Pose '" + request->pose_name + "' stored successfully";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->message = std::string("Error storing pose: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void PoseRegistry::retrieve_pose_callback(
    const std::shared_ptr<pose_registry_interfaces::srv::RetrievePose::Request>
        request,
    std::shared_ptr<pose_registry_interfaces::srv::RetrievePose::Response>
        response)
{
    try
    {
        auto it = poses_.find(request->pose_name);
        if (it != poses_.end())
        {
            response->pose = it->second;
            response->success = true;
            response->message = "Pose '" + request->pose_name + "' retrieved successfully";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }
        else
        {
            response->success = false;
            response->message = "Pose '" + request->pose_name + "' not found";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        }
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->message = std::string("Error retrieving pose: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}
// Private method to load poses from JSON file
void PoseRegistry::load_poses_from_file(const std::string &poses_file_path_)
{
    try
    {
        std::ifstream file(poses_file_path_);
        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "Could not open poses file: %s", poses_file_path_.c_str());
            return;
        }

        json poses_json;
        file >> poses_json;
        file.close();

        for (auto &[pose_name, pose_data] : poses_json.items())
        {
            try
            {
                geometry_msgs::msg::PoseStamped pose_stamped;

                // Extract frame_id
                if (pose_data.contains("frame_id"))
                {
                    pose_stamped.header.frame_id = pose_data["frame_id"];
                }
                pose_stamped.header.stamp = this->get_clock()->now();

                // Extract position [x, y, z]
                if (pose_data.contains("position") && pose_data["position"].size() == 3)
                {
                    pose_stamped.pose.position.x = pose_data["position"][0];
                    pose_stamped.pose.position.y = pose_data["position"][1];
                    pose_stamped.pose.position.z = pose_data["position"][2];
                }

                // Extract orientation [x, y, z, w] (quaternion)
                if (pose_data.contains("orientation") && pose_data["orientation"].size() == 4)
                {
                    pose_stamped.pose.orientation.x = pose_data["orientation"][0];
                    pose_stamped.pose.orientation.y = pose_data["orientation"][1];
                    pose_stamped.pose.orientation.z = pose_data["orientation"][2];
                    pose_stamped.pose.orientation.w = pose_data["orientation"][3];
                }

                poses_[pose_name] = pose_stamped;
                RCLCPP_DEBUG(this->get_logger(), "Loaded pose: %s", pose_name.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Error parsing pose '%s': %s", pose_name.c_str(), e.what());
            }
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading poses file: %s", e.what());
    }
}

// Append pose to JSON file
void PoseRegistry::append_pose_to_file(const std::string &pose_name, const geometry_msgs::msg::PoseStamped &pose_stamped)
{
    try
    {
        json poses_json;
        std::ifstream file_in(poses_file_path_);
        if (file_in.is_open())
        {
            file_in >> poses_json;
            file_in.close();
        }
        poses_json[pose_name] = {
            {"frame_id", pose_stamped.header.frame_id},
            {"position", {pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z}},
            {"orientation", {pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w}}};

        std::ofstream file_out(poses_file_path_);
        file_out << poses_json.dump(4);
        file_out.close();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error appending pose to file: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseRegistry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}