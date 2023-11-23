#ifndef __TUNNEL_LIDAR_HPP__
#define __TUNNEL_LIDAR_HPP__
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
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>

// Visualize header
#include <visualization_msgs/MarkerArray.h>

// Config header
#include <tunnel_lidar_config.hpp>

// PCL header
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// Mission Define
#define NORMAL_DRIVE 0
#define STATIC_OBSTACLE_1 1
#define TRAFFIC_LIGHT 2
#define ROTARY 3
#define DYNAMIC_OBSTACLE 4
#define PARKING 5
#define TUNNEL 6
#define STATIC_OBSTACLE_3 7

class TunnelLidar {

public:
    TunnelLidar(ros::NodeHandle &nh_);
    ~TunnelLidar();

    void Init();
    void ProcessINI();
    void Run();
    void Publish();
    void UpdateRviz();

    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg);
    void MissionCallback(const std_msgs::Int8ConstPtr &in_mission_msg);

    double GRTheta(double x, double y);
    void VoxelizeData();
    void SetROI();
    void MakeTargetPoint();

private:
    // Publisher
    ros::Publisher p_roi_lidar_pub;
    ros::Publisher p_target_point_pub;
    ros::Publisher p_target_point_rviz_pub;

    // Subscriber
    ros::Subscriber s_lidar_sub;
    ros::Subscriber s_mission_sub;

    // Mutex
    std::mutex mutex_lidar;

    // Messages

    // Environments
    IniParser v_ini_parser_;

    // Configuration parameters
    TunnelLidarParameters tunnel_lidar_params_;

    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_raw_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_voxelized_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filtered_ptr;

    sensor_msgs::PointCloud2 m_output_voxel;
    pcl::PCLPointCloud2 m_cloud_p;
    sensor_msgs::PointCloud2 m_output_gr;
    pcl::PointCloud<pcl::PointXYZ> m_laser_cloud_in;
    sensor_msgs::PointCloud2 m_output_roi;

    geometry_msgs::Point m_target_point;
    visualization_msgs::Marker m_target_point_marker;


    int m_mission = NORMAL_DRIVE;
    int m_print_count = 0;
};

#endif // __TUNNEL_LIDAR_HPP__