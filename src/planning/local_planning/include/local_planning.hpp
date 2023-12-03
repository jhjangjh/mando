#ifndef __LOCAL_PLANNING_HPP__
#define __LOCAL_PLANNING_HPP__
#pragma once

// STD header
#include <string>
#include <stdlib.h>
#include <fstream>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// Utility header
#include <util/ini_parser.hpp>

// Message header
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <kucudas_msgs/Trajectory.h>
#include <kucudas_msgs/VehicleInformation.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

// Config header
#include <local_planning_config.hpp>

// Lanelet
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Lanelet.h>

// tf
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// carla
#include <carla_msgs/CarlaEgoVehicleStatus.h>

// Mission Define
#define NORMAL_DRIVE_01 11
#define TRAFFIC_LIGHT_1 1
#define NORMAL_DRIVE_12 12
#define TRAFFIC_LIGHT_2 2
#define NORMAL_DRIVE_23 13
#define TRAFFIC_LIGHT_3 3
#define NORMAL_DRIVE_34 14
#define TRAFFIC_LIGHT_4 4
#define NORMAL_DRIVE_45 15
#define TRAFFIC_LIGHT_5 5
#define NORMAL_DRIVE_56 16
#define TRAFFIC_LIGHT_6 6
#define NORMAL_DRIVE_67 17
#define TRAFFIC_LIGHT_7 7
#define NORMAL_DRIVE_78 18
#define TRAFFIC_LIGHT_8 8
#define LOOP 9
#define TUNNEL 10 
#define NORMAL_DRIVE_910 0
#define STOP 99
#define FAIL 77

#define RED 0
#define GREEN 2
#define GREEN_ 4
#define YELLOW 1

#define NO_BLOCK 0
#define _1_BLOCK 1
#define _2_BLOCK 2
#define _1_2_BLOCK 3

// Namespace
using namespace lanelet;

class LocalPlanning {

public:
    LocalPlanning(ros::NodeHandle &nh_);
    ~LocalPlanning();

    void Init();
    void Route1Callback(const geometry_msgs::PoseArrayConstPtr &in_route1_msg);
    void Route2Callback(const geometry_msgs::PoseArrayConstPtr &in_route2_msg);
    void Route3Callback(const geometry_msgs::PoseArrayConstPtr &in_route3_msg);
    void OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg);
    void GnssCallback(const sensor_msgs::NavSatFixConstPtr &in_gnss_msg);
    void TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg);
    void VsCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_vs_msg);
    void AheadVehicleCallback(const std_msgs::Int8ConstPtr &in_ahead_vehicle_info_msg);
    void MissionCallback(const std_msgs::Int8ConstPtr &in_mission_msg);
    void TrafficLightCallback(const std_msgs::Int8ConstPtr &traffic_light_msg);
    void ProcessINI();
    void Run();
    void Publish();
    void SelectWaypoint();
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
    ros::Subscriber s_global_route1_sub;
    ros::Subscriber s_global_route2_sub;
    ros::Subscriber s_loop_route_sub;
    ros::Subscriber s_odom_sub;
    ros::Subscriber s_gnss_sub;
    ros::Subscriber s_tf_sub;
    ros::Subscriber s_vs_sub;
    ros::Subscriber s_object_block_sub;
    ros::Subscriber s_mission_sub;
    ros::Subscriber s_traffic_light_sub;    // subscribe traffic light signal

    // Messages
    int m_block = NO_BLOCK;

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    LocalPlanningParameters local_planning_params_;

    // Variables

    std::vector<geometry_msgs::Point> m_lane_1_vec;
    std::vector<geometry_msgs::Point> m_lane_loop_vec;
    std::vector<geometry_msgs::Point> m_lane_2_vec;

    std::vector<geometry_msgs::Point> m_waypoint_vec;

    nav_msgs::Odometry m_odom;
    sensor_msgs::NavSatFix m_gnss;
    lanelet::BasicPoint3d m_location_xyz;
    float vs_velocity;
    kucudas_msgs::VehicleInformation m_ahead_vehicle_info;

    // tf Variables
    tf::TransformListener listener;
    tf::StampedTransform m_ego_vehicle_transform;

    // Lanelet
    Origin lanelet_origin{{0.0, 0.0}}; //Town03 Map Center latlon = 0.0 , 0.0
    projection::UtmProjector lanelet_utm_projector;
    LaneletMapPtr lanelet_map;

    double m_ego_x;
    double m_ego_y;
    double m_yaw;
    double m_velocity;

    geometry_msgs::Point m_closest_point;
    int m_closest_id;
    int m_last_id;

    kucudas_msgs::Trajectory m_trajectory;
    visualization_msgs::MarkerArray m_trajectory_marker_array;

    bool get_global_route1 = false;
    bool get_global_route2 = false;
    bool get_loop_route = false;

    int m_print_count = 0;

    int m_mission = NORMAL_DRIVE_01;

    int traffic_signal = GREEN;      // subscribing traffic light signal     
};

#endif // __LOCAL_PLANNING_HPP__