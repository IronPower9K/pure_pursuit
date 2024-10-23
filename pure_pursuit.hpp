/*
Pure Pursuit Implementation in C++. Includes features such as dynamic lookahead. 
Does not have waypoint interpolation yet.
*/
#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // Lidar 센서 관련 헤더 추가

#define _USE_MATH_DEFINES
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node {
   public:
    PurePursuit();

    // LiDAR 콜백 함수 (장애물 회피)
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void get_waypoint();
    void transformandinterp_waypoint();
  
    void publish_message(double steering_angle);

    // 추가: Lidar 데이터 전처리 함수
    void preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // 추가: 속도 계산 함수 선언
    double get_velocity(double steering_angle);
    double p_controller();                     // 추가: 동적 K_p를 적용한 P 제어 함수
    double get_dynamic_Kp(double velocity);    // 추가: 속도에 따른 K_p 계산 함수

   private:
    struct csvFileData {
        std::vector<double> X;
        std::vector<double> Y;
        std::vector<double> V;

        int index;
        int velocity_index;

        Eigen::Vector3d lookahead_point_world;  
        Eigen::Vector3d lookahead_point_car;    
        Eigen::Vector3d current_point_world;    
    };

    Eigen::Matrix3d rotation_m;

    double x_car_world;
    double y_car_world;

    std::string odom_topic;
    std::string car_refFrame;
    std::string drive_topic;
    std::string global_refFrame;
    std::string rviz_current_waypoint_topic;
    std::string rviz_lookahead_waypoint_topic;
    std::string waypoints_path;
    double K_p;
    double min_lookahead;
    double max_lookahead;
    double lookahead_ratio;
    double steering_limit;
    double velocity_percentage;
    double curr_velocity = 0.0;

    bool emergency_braking = false;
    std::string lane_number = "left"; 

    std::fstream csvFile_waypoints;

    csvFileData waypoints;
    int num_waypoints;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_lidar;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server;

    double to_radians(double degrees);
    double to_degrees(double radians);
    double p2pdist(double &x1, double &x2, double &y1, double &y2);

    void load_waypoints();
    void interpolate_waypoints(); 
    void visualize_lookahead_point(Eigen::Vector3d &point);
    void visualize_current_point(Eigen::Vector3d &point);
    void quat_to_rot(double q0, double q1, double q2, double q3);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj);
    void timer_callback();
    void create_interactive_markers();
    void process_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    void save_waypoints_to_csv();

    // 장애물 감지 관련 함수
    bool obstacle_in_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);  
    std::pair<int, int> find_max_gap();  
    int find_best_point(int starting_i, int gap_distance);  

    std::vector<double> processed_lidar;
    // 회피 동작 관련 멤버 변수 추가
    bool avoidance_in_progress;  // 회피 동작 여부를 저장하는 플래그
    rclcpp::Time last_avoidance_time;  // 회피 동작이 시작된 시간

};
