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
#include <util/ini_parser.hpp>

// Message header
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <jsk_rviz_plugins/OverlayText.h>

// Visualize header
#include <visualization_msgs/MarkerArray.h>

// Config header
#include <mission_generator_config.hpp>

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

#define LEFT_BOUNDARY_ID 4292
#define RIGHT_BOUNDARY_ID 4530
#define LANE_LEFT_ID -1370
#define LANE_CENTER_ID 4531
#define LANE_RIGHT_ID -1320

// Mission Define
#define NORMAL_DRIVE 0
#define STATIC_OBSTACLE_1 1
#define TRAFFIC_LIGHT 2
#define ROTARY 3
#define DYNAMIC_OBSTACLE 4
#define PARKING 5
#define TUNNEL 6
#define STATIC_OBSTACLE_3 7

// Namespace
using namespace lanelet;

class MissionGenerator {

public:
    MissionGenerator(ros::NodeHandle &nh_);
    ~MissionGenerator();

    void Init();
    void ProcessINI();
    void Run();
    void Publish();
    void ReadOSMFile();
    void MakeGlobalRoute();
    void UpdateState();
    void GenerateMission();
    geometry_msgs::Point FindClosestPoint();
    void UpdateRviz();

    void OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg);
    void GnssCallback(const sensor_msgs::NavSatFixConstPtr &in_gnss_msg);
    void TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg);
    void VsCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_vs_msg);
    
    

private:
    // Publisher
    ros::Publisher p_global_route1_pub;
    ros::Publisher p_global_route2_pub;
    ros::Publisher p_global_route3_pub;
    ros::Publisher p_rviz_lane_pub;
    ros::Publisher p_mission_pub;
    ros::Publisher p_rviz_mission_pub;

    // Subscriber
    ros::Subscriber s_odom_sub;
    ros::Subscriber s_gnss_sub;
    ros::Subscriber s_tf_sub;
    ros::Subscriber s_vs_sub;

    // Messages
    geometry_msgs::PoseArray global_route1_msg;
    geometry_msgs::PoseArray global_route2_msg;
    geometry_msgs::PoseArray global_route3_msg;
    visualization_msgs::MarkerArray lane_marker_array_msg;
    std_msgs::Int8 mission_msg;
    jsk_rviz_plugins::OverlayText rviz_mission_msg;

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    MissionGeneratorParameters mission_generator_params_;

    // Lanelet
    Origin lanelet_origin{{0.0, 0.0}}; //Town03 Map Center latlon = 0.0 , 0.0
    projection::UtmProjector lanelet_utm_projector;
    LaneletMapPtr lanelet_map;

    // Variables
    std::vector<geometry_msgs::Point> m_waypoint_vec;
    nav_msgs::Odometry m_odom;
    sensor_msgs::NavSatFix m_gnss;
    lanelet::BasicPoint3d m_location_xyz;
    float vs_velocity;

    // tf Variables
    tf::TransformListener listener;
    tf::StampedTransform m_ego_vehicle_transform;


    std::vector<geometry_msgs::Point> m_left_boundary_vec;
    std::vector<geometry_msgs::Point> m_right_boundary_vec;
    std::vector<geometry_msgs::Point> m_lane_left_vec;
    std::vector<geometry_msgs::Point> m_lane_center_vec;
    std::vector<geometry_msgs::Point> m_lane_right_vec;

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