#ifndef _VEHICLE_CONTROL_HPP__
#define _VEHICLE_CONTROL_HPP__
#pragma once

// STD header
#include <string>
#include <stdlib.h>
#include <fstream>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Utility header
#include <util/ini_parser.hpp>

// Message header
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <kucudas_msgs/Trajectory.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

// Carla header
#include <carla_msgs/CarlaEgoVehicleControl.h>

// Config header
#include <longitudinal_control_config.hpp>
#include <lateral_control_config.hpp>

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

// Namespace
using namespace lanelet;

class VehicleControl {

public:
    VehicleControl(ros::NodeHandle &nh_);
    ~VehicleControl();

    void Init();
    void OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg);
    void GnssCallback(const sensor_msgs::NavSatFixConstPtr &in_gnss_msg);
    void TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg);
    void VsCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_vs_msg);
    void TrajectoryCallback(const kucudas_msgs::TrajectoryConstPtr &in_trajectory_msg);
    void MissionCallback(const std_msgs::Int8ConstPtr &in_mission_msg);
    void TunnelPointCallback(const geometry_msgs::PointConstPtr &in_point_msg);
    void ProcessINI();
    void Run();
    void Publish();
    void UpdateState();
    double PIDControl(double desired_velocity);
    double PurePursuit(kucudas_msgs::TrajectoryPoint m_target_point);
    double SetLookAheadDistance();
    kucudas_msgs::TrajectoryPoint FindClosestPoint();
    kucudas_msgs::TrajectoryPoint FindTargetPoint(kucudas_msgs::TrajectoryPoint closest_point, double lookahead_distance);
    double GetCrossTrackError(kucudas_msgs::TrajectoryPoint target_point);
    double GetSteeringAngle(kucudas_msgs::TrajectoryPoint target_point);
    void SetControlCmd(double pid_speed_error, double steering_angle);
    void UpdateControlState();

private:
    // Publisher
    ros::Publisher p_control_cmd_pub;
    ros::Publisher p_current_speed_pub;
    ros::Publisher p_target_speed_pub;
    ros::Publisher p_speed_error_pub;
    ros::Publisher p_cross_track_error_pub;
    ros::Publisher p_yaw_error_pub;


    // Subscriber
    ros::Subscriber s_trajectory_sub;
    ros::Subscriber s_odom_sub;
    ros::Subscriber s_gnss_sub;
    ros::Subscriber s_tf_sub;
    ros::Subscriber s_vs_sub;
    ros::Subscriber s_mission_sub;
    ros::Subscriber s_tunnel_point_sub;

    // Messages
    carla_msgs::CarlaEgoVehicleControl m_control_msg;
    std_msgs::Float32 m_current_speed_msg;
    std_msgs::Float32 m_target_speed_msg;
    std_msgs::Float32 m_speed_error_msg;
    std_msgs::Float32 m_cross_track_error_msg;
    std_msgs::Float32 m_yaw_error_msg;

    // Environments
    IniParser v_ini_parser_;

    // tf
    tf::TransformListener tfListenr;

    // Configuration parameters
    LongitudinalControlParameters longitudinal_control_params_;
    LateralControlParameters lateral_control_params_;

    // Variables
    nav_msgs::Odometry m_odom;
    sensor_msgs::NavSatFix m_gnss;
    lanelet::BasicPoint3d m_location_xyz;
    float vs_velocity;

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
    double m_target_lateral_error;

    kucudas_msgs::TrajectoryPoint m_closest_point;
    kucudas_msgs::TrajectoryPoint m_target_point;
    int m_closest_id;
    int m_last_id;

    kucudas_msgs::Trajectory m_trajectory;

    std::vector<geometry_msgs::Point> m_parking_waypoint_vec;

    double pid_dt = (1.0)/100;

    double pid_Kp = 0.;
    double pid_Ki = 0.;
    double pid_Kd = 0.;

    double pid_Pout = 0;
    double pid_Iout = 0;
    double pid_Dout = 0;
    double pid_integral = 0.;
    double pid_derivative = 0.;
    double pid_pre_error = 0.;

    int m_print_count = 0;
    int m_mission = NORMAL_DRIVE_01;

    geometry_msgs::Point m_tunnel_point;
};

#endif // __VEHICLE_CONTROL_HPP__