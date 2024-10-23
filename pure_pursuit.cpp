#include "pure_pursuit.hpp"

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
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // Lidar 추가

PurePursuit::PurePursuit() : Node("pure_pursuit_node") {
    // Initialise parameters
    this->declare_parameter("waypoints_path", "/home/hee/f1tenth_ws/src/pure_pursuit/racelines/e7_floor5.csv");
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("car_refFrame", "ego_racecar/base_link");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
    this->declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint");
    this->declare_parameter("global_refFrame", "map");
    this->declare_parameter("min_lookahead", 0.5);
    this->declare_parameter("max_lookahead", 1.0);
    this->declare_parameter("lookahead_ratio", 8.0);
    this->declare_parameter("K_p", 0.5);
    this->declare_parameter("steering_limit", 25.0);
    this->declare_parameter("velocity_percentage", 0.8);

    // Default Values
    waypoints_path = this->get_parameter("waypoints_path").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    car_refFrame = this->get_parameter("car_refFrame").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    rviz_current_waypoint_topic = this->get_parameter("rviz_current_waypoint_topic").as_string();
    rviz_lookahead_waypoint_topic = this->get_parameter("rviz_lookahead_waypoint_topic").as_string();
    global_refFrame = this->get_parameter("global_refFrame").as_string();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    K_p = this->get_parameter("K_p").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));
    subscription_lidar = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PurePursuit::lidar_callback, this, _1)); // Lidar 추가

    timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));

    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
    vis_current_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_current_waypoint_topic, 10);
    vis_lookahead_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_lookahead_waypoint_topic, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 멤버 변수로 추가
    bool avoidance_in_progress = false;  // 회피 동작 여부를 저장하는 플래그
    rclcpp::Time last_avoidance_time;  // 회피 동작이 시작된 시간
    
    // Interactive Marker Server 초기화
    marker_server = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "pure_pursuit_markers",
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_topics_interface(),
        this->get_node_services_interface(),
        rclcpp::QoS(10), rclcpp::QoS(10)
    );

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node has been launched");

    load_waypoints(); // Load waypoints from CSV
    create_interactive_markers(); // Create interactive markers for waypoints
}





// Convert degrees to radians
double PurePursuit::to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Convert radians to degrees
double PurePursuit::to_degrees(double radians) {
    return radians * 180.0 / M_PI;
}

// Point to point distance calculation
double PurePursuit::p2pdist(double &x1, double &x2, double &y1, double &y2) {
    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

// Load waypoints from CSV file
void PurePursuit::load_waypoints() {
    csvFile_waypoints.open(waypoints_path, std::ios::in);

    if (!csvFile_waypoints.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path.c_str());
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "CSV File Opened: %s", waypoints_path.c_str());
    }

    std::string line, word;
    while (std::getline(csvFile_waypoints, line)) {
        std::stringstream s(line);
        std::vector<double> waypoint;
        int col = 0;
        while (std::getline(s, word, ',')) {
            if (!word.empty()) {
                try {
                    double val = std::stod(word);
                    if (col == 0) {
                        waypoints.X.push_back(val);
                    } else if (col == 1) {
                        waypoints.Y.push_back(val);
                    } else if (col == 2) {
                        waypoints.V.push_back(val);
                    }
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "Error converting CSV value to double: %s", e.what());
                    continue;
                }
            }
            col++;
        }
    }

    csvFile_waypoints.close();
    num_waypoints = waypoints.X.size();

    if (num_waypoints == 0) {
        RCLCPP_ERROR(this->get_logger(), "No waypoints were loaded from %s", waypoints_path.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path.c_str());
}

// Create interactive markers for waypoints
void PurePursuit::create_interactive_markers() {
    for (size_t i = 0; i < waypoints.X.size(); i++) {
        visualization_msgs::msg::InteractiveMarker marker;
        marker.header.frame_id = "map";
        marker.pose.position.x = waypoints.X[i];
        marker.pose.position.y = waypoints.Y[i];
        marker.pose.orientation.w = 1.0;
        marker.name = std::to_string(i);  // Marker 이름에 인덱스를 포함

        visualization_msgs::msg::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        control.always_visible = true;

        control.orientation.z = 0;
        marker.controls.push_back(control);

        marker_server->insert(marker, std::bind(&PurePursuit::process_feedback, this, std::placeholders::_1));
    }
    marker_server->applyChanges();  // Apply the changes to the marker server
}

// Process feedback from the interactive marker and update the waypoints
void PurePursuit::process_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
    size_t index = std::stoi(feedback->marker_name);  // Extract index from marker name
    waypoints.X[index] = feedback->pose.position.x;
    waypoints.Y[index] = feedback->pose.position.y;

    save_waypoints_to_csv();  // Save updated waypoints to CSV
}

// Save the updated waypoints to a CSV file
void PurePursuit::save_waypoints_to_csv() {
    std::ofstream csv_file("updated_waypoints.csv");
    for (size_t i = 0; i < waypoints.X.size(); i++) {
        csv_file << waypoints.X[i] << "," << waypoints.Y[i] << "," << waypoints.V[i] << std::endl;
    }
    csv_file.close();
}

// Visualize the lookahead point in RViz
void PurePursuit::visualize_lookahead_point(Eigen::Vector3d &point) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.id = 1;
    vis_lookahead_point_pub->publish(marker);
}

// Visualize the current point in RViz
void PurePursuit::visualize_current_point(Eigen::Vector3d &point) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.b = 1.0;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.id = 1;
    vis_current_point_pub->publish(marker);
}

// Get the next waypoint for pure pursuit
void PurePursuit::get_waypoint() {
    double longest_distance = 0;
    int final_i = -1;
    int start = waypoints.index;
    int end = (waypoints.index + 500) % num_waypoints;

    double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);

    if (end < start) {
        for (int i = start; i < num_waypoints; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead &&
                p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
        for (int i = 0; i < end; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead &&
                p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    } else {
        for (int i = start; i < end; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead &&
                p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    }

    if (final_i == -1) {
        final_i = 0;
        for (int i = 0; i < num_waypoints; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead &&
                p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    }

    double shortest_distance = p2pdist(waypoints.X[0], x_car_world, waypoints.Y[0], y_car_world);
    int velocity_i = 0;
    for (int i = 0; i < num_waypoints; i++) {
        if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= shortest_distance) {
            shortest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
            velocity_i = i;
        }
    }

    waypoints.index = final_i;
    waypoints.velocity_index = velocity_i;
}

// Quaternion to rotation matrix conversion
void PurePursuit::quat_to_rot(double q0, double q1, double q2, double q3) {
    double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
    double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
    double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));

    double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
    double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
    double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));

    double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
    double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
    double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

    rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22;
}

// Transform waypoints to car frame and interpolate
void PurePursuit::transformandinterp_waypoint() {
    waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;
    waypoints.current_point_world << waypoints.X[waypoints.velocity_index], waypoints.Y[waypoints.velocity_index], 0.0;

    visualize_lookahead_point(waypoints.lookahead_point_world);
    visualize_current_point(waypoints.current_point_world);

    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_->lookupTransform(car_refFrame, global_refFrame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
    }

    Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    waypoints.lookahead_point_car = (rotation_m * waypoints.lookahead_point_world) + translation_v;
}

// Get dynamic K_p based on current speed
double PurePursuit::get_dynamic_Kp(double velocity) {
    double min_K_p = 0.5; // 최소 속도에서의 최대 K_p 값
    double max_K_p = 0.1; // 최대 속도에서의 최소 K_p 값
    double max_speed = 6.0; // 차량의 최고 속도 (설정 가능)

    std::cout << "velocity" << velocity << std::endl;

    if(velocity < max_speed){
        // 속도가 높을수록 K_p 값이 작아지고, 속도가 낮을수록 K_p 값이 커짐
        double dynamic_K_p = max_K_p + (min_K_p - max_K_p) * ((max_speed - velocity) / max_speed);
        RCLCPP_INFO(this->get_logger(), "velocity: %f, k_p: %f", velocity, dynamic_K_p);  // 시작 인덱스와 간격 출력
        return dynamic_K_p;
    }
    else{
        return max_K_p;
    }
    
    
    
   
}

// P-controller for steering calculation with dynamic K_p
double PurePursuit::p_controller() {
    double r = waypoints.lookahead_point_car.norm();
    double y = waypoints.lookahead_point_car(1);

    // 현재 속도에 따라 동적으로 K_p 값을 계산
    double dynamic_K_p = get_dynamic_Kp(curr_velocity);

    // P-controller 계산에 dynamic K_p 적용
    double angle = dynamic_K_p * 2 * y / pow(r, 2);

    return angle;
}

// Get velocity based on steering angle
double PurePursuit::get_velocity(double steering_angle) {
    double velocity = 0;

    if (waypoints.V[waypoints.velocity_index]) {
        velocity = waypoints.V[waypoints.velocity_index] * velocity_percentage;
    } else {
        if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            velocity = 6.0 * velocity_percentage;
        } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
            velocity = 2.5 * velocity_percentage;
        } else {
            velocity = 2.0 * velocity_percentage;
        }
    }

    return velocity;
}

// Publish drive messages
void PurePursuit::publish_message(double steering_angle) {
    auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
    
    // 조향 각도 제한
    if (steering_angle < 0.0) {
        drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit));
    } else {
        drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit));
    }

    // 현재 속도 계산 (갱신)
    curr_velocity = get_velocity(drive_msgObj.drive.steering_angle);
    
    // 속도 설정
    drive_msgObj.drive.speed = curr_velocity;

    publisher_drive->publish(drive_msgObj);
}

// Odometry callback
void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    x_car_world = odom_submsgObj->pose.pose.position.x;
    y_car_world = odom_submsgObj->pose.pose.position.y;

    get_waypoint();
    transformandinterp_waypoint();

    double steering_angle = p_controller();
    publish_message(steering_angle);
}

// Timer callback for parameter updates
void PurePursuit::timer_callback() {
    K_p = this->get_parameter("K_p").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
}

// 장애물 감지 범위 확인
bool PurePursuit::obstacle_in_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    int left_limit = 520;
    int right_limit = 560;

    // 디버깅 로그 추가 (this 제거)
    /*
    RCLCPP_INFO(this->get_logger(), "Lidar Scan Data: angle_increment: %f, left_limit: %d, right_limit: %d", 
                scan_msg->angle_increment, left_limit, right_limit);
    */
    for (int i = left_limit; i <= right_limit; i++) {
        double range = scan_msg->ranges[i];

        // 유효하지 않은 값(너무 큰 값) 제외
        if (range > 3.0 || range < scan_msg->range_min || range > scan_msg->range_max) {
            continue;
        }

        // 장애물 감지 로그 추가
        //RCLCPP_INFO(this->get_logger(), "Checking range: %f at index %d", range, i);

        if (range < 1.0) {  // 1미터 이내 장애물 감지
            return true;
        }
    }
    return false;
}

// Lidar 데이터 전처리 함수
void PurePursuit::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    this->processed_lidar.clear();
    for (size_t i = 0; i < scan_msg->ranges.size(); i++) {  // signed-unsigned mismatch 수정
        if (scan_msg->ranges[i] > 3.0) {
            this->processed_lidar.push_back(0);
        } else {
            this->processed_lidar.push_back(scan_msg->ranges[i]);
        }
    }
}

std::pair<int, int> PurePursuit::find_max_gap() {
    int largest_starting_i = 0;
    int longest_gap = 0;
    int curr_gap_start = -1;
    int curr_gap = 0;

    RCLCPP_INFO(this->get_logger(), "Processed Lidar Size: %lu", this->processed_lidar.size());  // Lidar 데이터 크기 출력

    // Lidar 데이터를 순차적으로 확인
    for (size_t i = 0; i < this->processed_lidar.size(); i++) {
        if (this->processed_lidar[i] < 1.5) {
            // 현재 구간이 끝났을 때, 가장 긴 구간을 업데이트
            if (curr_gap > longest_gap) {
                longest_gap = curr_gap;
                largest_starting_i = curr_gap_start;
            }
            curr_gap = 0;  // 간격을 다시 초기화
        } else {
            // 첫 시작점을 저장
            if (curr_gap == 0) {
                curr_gap_start = i;
            }
            curr_gap++;
        }
    }

    // 마지막 구간이 가장 길었을 경우 처리
    if (curr_gap > longest_gap) {
        longest_gap = curr_gap;
        largest_starting_i = curr_gap_start;
    }

    RCLCPP_INFO(this->get_logger(), "Max Gap Found: Start Index: %d, Gap Length: %d", largest_starting_i, longest_gap);  // Max gap 로그 출력

    return std::make_pair(largest_starting_i, longest_gap);
}
int PurePursuit::find_best_point(int starting_i, int gap_distance) {
    RCLCPP_INFO(this->get_logger(), "Finding Best Point: Start Index: %d, Gap Distance: %d", starting_i, gap_distance);  // 시작 인덱스와 간격 출력

    int farthest_i = starting_i;
    double farthest_distance = 0;
    for (int i = starting_i; i < starting_i + gap_distance; i++) {
        if (this->processed_lidar[i] > farthest_distance) {
            farthest_i = i;
            farthest_distance = this->processed_lidar[i];
        }
    }

    RCLCPP_INFO(this->get_logger(), "Best Point Found: Index: %d, Distance: %f", farthest_i, farthest_distance);  // 최적 포인트 출력

    return farthest_i;
}

void PurePursuit::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    rclcpp::Time now = this->now();  // 현재 시간 가져오기

    preprocess_lidar(scan_msg);  // LiDAR 데이터를 전처리

    
    
    /*
    // 회피 동작이 진행 중이며, 3초가 지나지 않았다면 계속 gap follow
    if (avoidance_in_progress && (now - last_avoidance_time).seconds() < 2.0) {
        RCLCPP_INFO(this->get_logger(), "Continuing avoidance maneuver...");

        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();

        // Gap follow logic: 최대 gap 방향으로 설정
        double best_angle_i = find_best_point(find_max_gap().first, find_max_gap().second);
        
        RCLCPP_INFO(this->get_logger(), "index: %f",best_angle_i);
        
        drive_msgObj.drive.steering_angle = best_angle_i*1.0;  // 장애물 회피를 위한 회전 각도 설정
        drive_msgObj.drive.speed = 1.0;  // 장애물 회피 중 속도 줄임

        publisher_drive->publish(drive_msgObj);  // 드라이브 메시지 발행
        return;  // 3초 동안 회피 동작 유지, 다른 동작은 하지 않음
    }
    */

    // 장애물이 감지된 경우
    if (obstacle_in_range(scan_msg)) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected! Executing avoidance maneuver.");

        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
        
        // 최대 gap 방향으로 설정
        double best_angle_i = find_best_point(find_max_gap().first, find_max_gap().second);
        
        drive_msgObj.drive.steering_angle = best_angle_i*2.0;  // 장애물 회피를 위한 회전 각도 설정
        drive_msgObj.drive.speed = 1.5;  // 장애물 회피 중 속도 줄임

        publisher_drive->publish(drive_msgObj);  // 드라이브 메시지 발행
        
        // 회피 동작 상태를 활성화하고 시간을 기록
        avoidance_in_progress = true;
        last_avoidance_time = now;
    } else {
        // 장애물이 없으면 PurePursuit 방식으로 계속 진행
        avoidance_in_progress = false;  // 회피 동작이 종료됨
        
        get_waypoint();  // 함수 호출
        transformandinterp_waypoint();  // 함수 호출

        double steering_angle = p_controller();  // 제어 함수 호출
        std::cout << steering_angle << std::endl;
        publish_message(steering_angle);  // 메시지 발행 함수 호출
    }
}

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
