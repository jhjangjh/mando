#ifndef __MISSION_GENERATOR_HPP__
#define __MISSION_GENERATOR_HPP__
#pragma once

// STD header
#include <string>
#include <stdlib.h>
#include <fstream>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// Utility header
#include <util/transForm.hpp>
#include <util/ini_parser.hpp>

// Message header
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

// Visualize header
#include <visualization_msgs/MarkerArray.h>

// Config header
#include <mission_generator_config.hpp>

// Mission Define
#define NORMAL_DRIVE 0
#define STATIC_OBSTACLE_1 1
#define TRAFFIC_LIGHT 2
#define ROTARY 3
#define DYNAMIC_OBSTACLE 4
#define PARKING 5
#define TUNNEL 6
#define STATIC_OBSTACLE_3 7

class MissionGenerator {

public:
    MissionGenerator(ros::NodeHandle &nh_);
    ~MissionGenerator();

    void Init();
    void ProcessINI();
    void Run();
    void Publish();
    void ReadCSVFile();
    void MakeGlobalRoute();
    void UpdateState();
    void GenerateMission();
    geometry_msgs::Point FindClosestPoint();
    void UpdateRviz();

    void OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg);

private:
    // Publisher
    ros::Publisher p_global_route_pub;
    ros::Publisher p_rviz_lane_pub;
    ros::Publisher p_mission_pub;

    // Subscriber
    ros::Subscriber s_odom_sub;

    // Mutex
    std::mutex mutex_odom;

    // Messages
    geometry_msgs::PoseArray global_route_msg;
    visualization_msgs::MarkerArray lane_marker_array_msg;
    std_msgs::Int8 mission_msg;

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    MissionGeneratorParameters mission_generator_params_;

    // Variables
    std::vector<geometry_msgs::Point> m_waypoint_vec;
    nav_msgs::Odometry m_odom;

    geometry_msgs::Point m_closest_point;
    int m_closest_id;

    double m_ego_x;
    double m_ego_y;
    double m_yaw;
    double m_velocity;

    int m_mission = NORMAL_DRIVE;
    int m_print_count = 0;
};

#endif // __MISSION_GENERATOR_HPP__