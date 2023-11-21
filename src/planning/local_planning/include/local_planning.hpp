#ifndef __LOCAL_PLANNING_HPP__
#define __LOCAL_PLANNING_HPP__
#pragma once

// STD header
#include <string>
#include <stdlib.h>
#include <fstream>
#include <mutex>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// Utility header
#include <util/ini_parser.hpp>

// Message header
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <kucudas_msgs/Trajectory.h>
#include <kucudas_msgs/VehicleInformation.h>
#include <std_msgs/Int8.h>

// Config header
#include <local_planning_config.hpp>

// Mission Define
#define NORMAL_DRIVE 0
#define STATIC_OBSTACLE_1 1
#define TRAFFIC_LIGHT 2
#define ROTARY 3
#define DYNAMIC_OBSTACLE 4
#define PARKING 5
#define TUNNEL 6
#define STATIC_OBSTACLE_3 7

class LocalPlanning {

public:
    LocalPlanning(ros::NodeHandle &nh_);
    ~LocalPlanning();

    void Init();
    void RouteCallback(const geometry_msgs::PoseArrayConstPtr &in_route1_msg);
    void OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg);
    void AheadVehicleCallback(const kucudas_msgs::VehicleInformationConstPtr &in_ahead_vehicle_info_msg);
    void MissionCallback(const std_msgs::Int8ConstPtr &in_mission_msg);
    void ProcessINI();
    void Run();
    void Publish();
    void UpdateState();
    void MakeTrajectory();
    geometry_msgs::Point FindClosestPoint();
    int FindLastIndex(geometry_msgs::Point closest_point, double trajectory_length);
    double CalculateYaw(int index);
    double CalculateCurvature(int index);
    double SpeedProfiling(double curvature);
    void UpdateRvizTrajectory(const kucudas_msgs::Trajectory& trajectory);

private:
    // Publisher
    ros::Publisher p_trajectory_pub;
    ros::Publisher p_rviz_trajectory_pub;

    // Subscriber
    ros::Subscriber s_global_route_sub;
    ros::Subscriber s_odom_sub;
    ros::Subscriber s_ahead_vehicle_sub;
    ros::Subscriber s_mission_sub;

    // Mutex
    std::mutex mutex_route;
    std::mutex mutex_odom;
    std::mutex mutex_ahead_vehicle_info;

    // Messages

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    LocalPlanningParameters local_planning_params_;

    // Variables
    std::vector<geometry_msgs::Point> m_waypoint_vec;

    nav_msgs::Odometry m_odom;
    kucudas_msgs::VehicleInformation m_ahead_vehicle_info;

    double m_ego_x;
    double m_ego_y;
    double m_yaw;
    double m_velocity;

    geometry_msgs::Point m_closest_point;
    int m_closest_id;
    int m_last_id;

    kucudas_msgs::Trajectory m_trajectory;
    visualization_msgs::MarkerArray m_trajectory_marker_array;

    bool get_global_route = false;

    int m_print_count = 0;

    int m_mission = NORMAL_DRIVE;
};

#endif // __LOCAL_PLANNING_HPP__