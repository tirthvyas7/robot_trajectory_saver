#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "amr_msgs/srv/trajsave.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>


class TrajectorySaver : public rclcpp::Node
{
public:
TrajectorySaver() : Node("trajectory_saver")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectorySaver::odom_callback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);
        service_ = this->create_service<amr_msgs::srv::Trajsave>(
            "save_trajectory", 
            std::bind(&TrajectorySaver::service_server, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    
    void service_server(
        const std::shared_ptr<amr_msgs::srv::Trajsave::Request> request,
        std::shared_ptr<amr_msgs::srv::Trajsave::Response> response_)
    {
        duration_ = request->duration;
        filename_ = request-> filename;
        marker_array_.markers.clear();
        start_time_ = this->get_clock()->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Adjust timer frequency if needed
            std::bind(&TrajectorySaver::timer_callback, this));
        
        response_->result=true;
    }

    void timer_callback()
    {
        // Check elapsed time
        if ((this->get_clock()->now() - start_time_).nanoseconds()/1e9 >= duration_)
        {
            timer_->cancel();  // Stop the timer
            save_to_yaml(filename_);
            RCLCPP_INFO(this->get_logger(), "Saved trajectory after %ld seconds", duration_);
        }
    }

    bool save_to_yaml(const std::string &filename){
        try
            {
                YAML::Emitter out;
                out << YAML::BeginSeq;

                for (const auto &marker : marker_array_.markers)
                {
                    out << YAML::BeginMap;
                    out << YAML::Key << "id" << YAML::Value << marker.id;
                    out << YAML::Key << "position" << YAML::Flow << YAML::BeginMap;
                    out << YAML::Key << "x" << YAML::Value << marker.pose.position.x;
                    out << YAML::Key << "y" << YAML::Value << marker.pose.position.y;
                    out << YAML::Key << "z" << YAML::Value << marker.pose.position.z;
                    out << YAML::EndMap; // position

                    out << YAML::Key << "orientation" << YAML::Flow << YAML::BeginMap;
                    out << YAML::Key << "qx" << YAML::Value << marker.pose.orientation.x;
                    out << YAML::Key << "qy" << YAML::Value << marker.pose.orientation.y;
                    out << YAML::Key << "qz" << YAML::Value << marker.pose.orientation.z;
                    out << YAML::Key << "qw" << YAML::Value << marker.pose.orientation.w;
                    out << YAML::EndMap; // orientation

                    out << YAML::EndMap; // marker
                }

                out << YAML::EndSeq;
                // out << YAML::EndMap;

                // Open file safely
                std::ofstream file(filename);
                if (!file.is_open())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
                    return false;
                }

                file << out.c_str();
                file.close();

                RCLCPP_INFO(this->get_logger(), "Successfully saved markers to: %s", filename.c_str());
                return true;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception while saving YAML: %s", e.what());
                return false;
            }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "trajectory";
        marker.id = marker_id_++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = msg->pose.pose.position.x;
        marker.pose.position.y = msg->pose.pose.position.y;
        marker.pose.position.z = msg->pose.pose.position.z;
        marker.pose.orientation = msg->pose.pose.orientation;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker_array_.markers.push_back(marker);
        marker_pub_->publish(marker_array_);
        

    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Service<amr_msgs::srv::Trajsave>::SharedPtr service_;
    visualization_msgs::msg::MarkerArray marker_array_;
    int marker_id_ = 0;
    rclcpp::Time start_time_;
    int64_t duration_;
    std::string filename_;
    std::shared_ptr<amr_msgs::srv::Trajsave::Response> response_;
    const std::shared_ptr<amr_msgs::srv::Trajsave::Request> request;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectorySaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
