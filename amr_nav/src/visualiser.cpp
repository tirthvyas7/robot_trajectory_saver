#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>

class TrajectoryVisualizer : public rclcpp::Node
{
public:
    TrajectoryVisualizer() : Node("trajectory_visualizer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);
        // timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&TrajectoryVisualizer::load_and_publish, this));
        this->load_and_publish();
    }

private:
    void load_and_publish()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Load YAML file
        // std::string file_path = "example.yaml";
        YAML::Node config = YAML::LoadFile("trajectory.yaml");

        geometry_msgs::msg::TransformStamped transform;

        for (size_t i = 0; i < config.size(); ++i)
        {
            YAML::Node marker_node = config[i]; // Explicitly access each node

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = this->now();
                marker.ns = "trajectory";
                marker.id = marker_node["id"].as<int>();
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;

                // Extract position
                geometry_msgs::msg::Pose pose;
                pose.position.x = marker_node["position"]["x"].as<double>();
                pose.position.y = marker_node["position"]["y"].as<double>();
                pose.position.z = marker_node["position"]["z"].as<double>();
                pose.orientation.x = marker_node["orientation"]["qx"].as<double>();
                pose.orientation.y = marker_node["orientation"]["qy"].as<double>();
                pose.orientation.z = marker_node["orientation"]["qz"].as<double>();
                pose.orientation.w = marker_node["orientation"]["qw"].as<double>();

                // Transform to odom frame
                tf2::doTransform(pose, pose, transform);
                marker.pose = pose;
                marker_array.markers.push_back(marker);
            
        }

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %lu transformed markers", marker_array.markers.size());
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryVisualizer>());
    rclcpp::shutdown();
    return 0;
}
