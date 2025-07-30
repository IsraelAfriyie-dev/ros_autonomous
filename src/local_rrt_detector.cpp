#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class LocalRRTDetector : public rclcpp::Node
{
public:
    LocalRRTDetector() : Node("local_rrt_frontier_detector")
    {
        // Initialize random number generators
        unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
        irand = std::make_unique<MTRand_int32>(init, length);
        drand = std::make_unique<MTRand>();
        
        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Declare parameters
        this->declare_parameter<double>("eta", 0.5);
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<std::string>("robot_frame", "/base_link");
        
        // Get parameters
        eta_ = this->get_parameter("eta").as_double();
        map_topic_ = this->get_parameter("map_topic").as_string();
        base_frame_topic_ = this->get_parameter("robot_frame").as_string();
        
        // Create subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, 100, std::bind(&LocalRRTDetector::mapCallBack, this, std::placeholders::_1));
        
        rviz_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 100, std::bind(&LocalRRTDetector::rvizCallBack, this, std::placeholders::_1));
        
        // Create publishers
        targets_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_points", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            std::string(this->get_name()) + "_shapes", 10);
        
        // Initialize visualization markers
        initializeMarkers();
        
        // Create timer for main loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&LocalRRTDetector::mainLoop, this));
        
        map_received_ = false;
        points_collected_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Local RRT Detector initialized");
    }

private:
    // ROS2 components
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr targets_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Parameters
    double eta_;
    std::string map_topic_;
    std::string base_frame_topic_;
    
    // Global variables
    nav_msgs::msg::OccupancyGrid mapData_;
    geometry_msgs::msg::PointStamped clickedpoint_;
    geometry_msgs::msg::PointStamped exploration_goal_;
    visualization_msgs::msg::Marker points_, line_;
    float xdim_, ydim_, resolution_, Xstartx_, Xstarty_, init_map_x_, init_map_y_;
    
    // Random number generators
    std::unique_ptr<MTRand_int32> irand;
    std::unique_ptr<MTRand> drand;
    
    // State variables
    bool map_received_;
    bool points_collected_;
    std::vector<std::vector<float>> V_;
    
    void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        mapData_ = *msg;
        map_received_ = true;
    }
    
    void rvizCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        geometry_msgs::msg::Point p;
        p.x = msg->point.x;
        p.y = msg->point.y;
        p.z = msg->point.z;
        
        points_.points.push_back(p);
    }
    
    void initializeMarkers()
    {
        points_.header.frame_id = "map";  // Will be updated when map is received
        line_.header.frame_id = "map";
        points_.header.stamp = this->get_clock()->now();
        line_.header.stamp = this->get_clock()->now();
        
        points_.ns = line_.ns = "markers";
        points_.id = 0;
        line_.id = 1;
        
        points_.type = visualization_msgs::msg::Marker::POINTS;
        line_.type = visualization_msgs::msg::Marker::LINE_LIST;
        
        points_.action = visualization_msgs::msg::Marker::ADD;
        line_.action = visualization_msgs::msg::Marker::ADD;
        
        points_.pose.orientation.w = 1.0;
        line_.pose.orientation.w = 1.0;
        
        line_.scale.x = 0.03;
        line_.scale.y = 0.03;
        points_.scale.x = 0.3;
        points_.scale.y = 0.3;
        
        // Red color for local RRT
        line_.color.r = 255.0/255.0;
        line_.color.g = 0.0/255.0;
        line_.color.b = 0.0/255.0;
        line_.color.a = 1.0;
        
        points_.color.r = 255.0/255.0;
        points_.color.g = 0.0/255.0;
        points_.color.b = 0.0/255.0;
        points_.color.a = 0.3;
        
        points_.lifetime = rclcpp::Duration::from_nanoseconds(0);
        line_.lifetime = rclcpp::Duration::from_nanoseconds(0);
    }
    
    void mainLoop()
    {
        // Wait until map is received
        if (!map_received_ || mapData_.data.size() < 1) {
            return;
        }
        
        // Update marker frame IDs
        if (points_.header.frame_id != mapData_.header.frame_id) {
            points_.header.frame_id = mapData_.header.frame_id;
            line_.header.frame_id = mapData_.header.frame_id;
        }
        
        // Wait for 5 clicked points
        if (!points_collected_ && points_.points.size() < 5) {
            marker_pub_->publish(points_);
            return;
        }
        
        // Initialize map dimensions and starting point (one time)
        if (!points_collected_ && points_.points.size() >= 5) {
            initializeMapDimensions();
            points_collected_ = true;
            return;
        }
        
        // Main RRT algorithm
        runRRTIteration();
    }
    
    void initializeMapDimensions()
    {
        std::vector<float> temp1, temp2;
        
        // Calculate init_map_x
        temp1.push_back(points_.points[0].x);
        temp1.push_back(points_.points[0].y);
        temp2.push_back(points_.points[2].x);
        temp2.push_back(points_.points[0].y);
        init_map_x_ = Norm(temp1, temp2);
        
        // Calculate init_map_y
        temp1.clear(); temp2.clear();
        temp1.push_back(points_.points[0].x);
        temp1.push_back(points_.points[0].y);
        temp2.push_back(points_.points[0].x);
        temp2.push_back(points_.points[2].y);
        init_map_y_ = Norm(temp1, temp2);
        
        // Calculate starting point
        Xstartx_ = (points_.points[0].x + points_.points[2].x) * 0.5;
        Xstarty_ = (points_.points[0].y + points_.points[2].y) * 0.5;
        
        // Initialize V with the starting point
        geometry_msgs::msg::Point trans = points_.points[4];
        std::vector<float> xnew;
        xnew.push_back(trans.x);
        xnew.push_back(trans.y);
        V_.push_back(xnew);
        
        // Clear points for visualization
        points_.points.clear();
        marker_pub_->publish(points_);
        
        RCLCPP_INFO(this->get_logger(), "Map dimensions initialized: %f x %f", init_map_x_, init_map_y_);
    }
    
    void runRRTIteration()
    {
        // Sample free
        std::vector<float> x_rand, x_nearest, x_new;
        
        float xr = ((*drand)() * init_map_x_) - (init_map_x_ * 0.5) + Xstartx_;
        float yr = ((*drand)() * init_map_y_) - (init_map_y_ * 0.5) + Xstarty_;
        
        x_rand.push_back(xr);
        x_rand.push_back(yr);
        
        // Nearest
        x_nearest = Nearest(V_, x_rand);
        
        // Steer
        x_new = Steer(x_nearest, x_rand, eta_);
        
        // ObstacleFree: 1=free, -1=unknown (frontier region), 0=obstacle
        char checking = ObstacleFree(x_nearest, x_new, mapData_);
        
        if (checking == -1) {
            // Found frontier
            exploration_goal_.header.stamp = this->get_clock()->now();
            exploration_goal_.header.frame_id = mapData_.header.frame_id;
            exploration_goal_.point.x = x_new[0];
            exploration_goal_.point.y = x_new[1];
            exploration_goal_.point.z = 0.0;
            
            geometry_msgs::msg::Point p;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            points_.points.push_back(p);
            
            marker_pub_->publish(points_);
            targets_pub_->publish(exploration_goal_);
            points_.points.clear();
            
            // Clear V and reinitialize with robot position
            V_.clear();
            
            // Get robot transform
            std::vector<float> robot_pos = getRobotPosition();
            if (!robot_pos.empty()) {
                V_.push_back(robot_pos);
                line_.points.clear();
            }
        }
        else if (checking == 1) {
            // Free space - add to tree
            V_.push_back(x_new);
            
            geometry_msgs::msg::Point p;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            line_.points.push_back(p);
            
            p.x = x_nearest[0];
            p.y = x_nearest[1];
            p.z = 0.0;
            line_.points.push_back(p);
        }
        
        // Update timestamps and publish
        line_.header.stamp = this->get_clock()->now();
        marker_pub_->publish(line_);
    }
    
    std::vector<float> getRobotPosition()
    {
        std::vector<float> robot_pos;
        
        try {
            // Look up transform from map to robot base frame
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform(mapData_.header.frame_id, base_frame_topic_, 
                                          tf2::TimePointZero);
            
            robot_pos.push_back(transform_stamped.transform.translation.x);
            robot_pos.push_back(transform_stamped.transform.translation.y);
            
            RCLCPP_DEBUG(this->get_logger(), "Robot position: [%f, %f]", 
                        robot_pos[0], robot_pos[1]);
            
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                       base_frame_topic_.c_str(), mapData_.header.frame_id.c_str(), ex.what());
            
            // Retry after a short delay
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            return robot_pos;  // Return empty vector
        }
        
        return robot_pos;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalRRTDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
