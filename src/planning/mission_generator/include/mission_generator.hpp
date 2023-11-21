#ifndef __MISSION_GENERATOR_HPP__
#define __MISSION_GENERATOR_HPP__
#pragma once

// STD header
#include <string>
#include <stdlib.h>
#include <fstream>

// ROS header
#include <ros/ros.h>

// Utility header
#include <util/transForm.hpp>
#include <util/ini_parser.hpp>

// Message header
#include <geometry_msgs/PoseArray.h>

// Visualize header
#include <visualization_msgs/MarkerArray.h>

// Config header
#include <mission_generator_config.hpp>

// Namespace

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
    void UpdateRvizLane();

private:
    // Publisher
    ros::Publisher p_global_route_pub;
    ros::Publisher p_rviz_lane_pub;

    // Messages
    geometry_msgs::PoseArray global_route_msg;
    visualization_msgs::MarkerArray lane_marker_array_msg;

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    MissionGeneratorParameters mission_generator_params_;

    // Variables
    std::vector<geometry_msgs::Point> m_waypoint_vec;
};

#endif // __MISSION_GENERATOR_HPP__