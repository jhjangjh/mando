#ifndef __GLOBAL_PLANNING_HPP__
#define __GLOBAL_PLANNING_HPP__
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

// Lanelet
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Lanelet.h>
#define LANE1_CENTER_ID -1206
#define LANE2_CENTER_ID -1279
#define LANE1_ID 41665
#define LANE2_ID 41666
#define LANE3_ID 41896

// Visualize header
#include <visualization_msgs/MarkerArray.h>

// Config header
#include <global_planning_config.hpp>

// Namespace
using namespace lanelet;

class GlobalPlanning {

public:
    GlobalPlanning(ros::NodeHandle &nh_);
    ~GlobalPlanning();

    void Init();
    void ProcessINI();
    void Run();
    void Publish();
    void ReadOsmFile();
    void ReadCSVFile();
    void MakeGlobalRoute();
    void UpdateRvizLane();

private:
    // Publisher
    ros::Publisher p_global_route1_pub;
    ros::Publisher p_global_route2_pub;
    ros::Publisher p_rviz_lane_pub;

    // Param
    std::string map_name;

    // Messages
    geometry_msgs::PoseArray global_route1_msg;
    geometry_msgs::PoseArray global_route2_msg;
    visualization_msgs::MarkerArray lane_marker_array_msg;

    // Lanelet
    Origin lanelet_origin{{0.0, 0.0}}; //Town04 Map Center latlon = 0.0 , 0.0
    projection::UtmProjector lanelet_utm_projector;
    LaneletMapPtr lanelet_map;

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    GlobalPlanningParameters global_planning_params_;

    // Variables
    std::vector<geometry_msgs::Point> m_waypoint_vec;

    std::vector<geometry_msgs::Point> m_lanelet_lane1_vec;
    std::vector<geometry_msgs::Point> m_lanelet_lane2_vec;
    std::vector<geometry_msgs::Point> m_lanelet_lane3_vec;

    std::vector<geometry_msgs::Point> m_lane1_vec;
    std::vector<geometry_msgs::Point> m_lane2_vec;
};

#endif // __GLOBAL_PLANNING_HPP__