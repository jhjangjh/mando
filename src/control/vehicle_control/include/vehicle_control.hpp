#ifndef _VEHICLE_CONTROL_HPP__
#define _VEHICLE_CONTROL_HPP__
#pragma once

// STD header
#include <string>
#include <stdlib.h>
#include <fstream>
#include <mutex>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Utility header
#include <util/ini_parser.hpp>

// Message header
#include <nav_msgs/Odometry.h>
#include <kucudas_msgs/Trajectory.h>
#include <std_msgs/Float32.h>

// Carla header
#include <carla_msgs/CarlaEgoVehicleControl.h>

// Config header
#include <longitudinal_control_config.hpp>
#include <lateral_control_config.hpp>

class VehicleControl {

public:
    VehicleControl(ros::NodeHandle &nh_);
    ~VehicleControl();

    void Init();
    void OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg);
    void TrajectoryCallback(const kucudas_msgs::TrajectoryConstPtr &in_trajectory_msg);
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

    // Mutex
    std::mutex mutex_odom;
    std::mutex mutex_trajectory;

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

};

#endif // __VEHICLE_CONTROL_HPP__