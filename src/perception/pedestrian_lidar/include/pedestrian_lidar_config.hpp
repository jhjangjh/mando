#ifndef __PEDESTRIAN_LIDAR_CONFIG_HPP__
#define __PEDESTRIAN_LIDAR_CONFIG_HPP__
#pragma once

typedef struct {
    double voxelsize;
    double roi_distance;
    double offset;
    double front_distance;
    double max_x_roi = 10;
    double max_y_roi = 2;
    double min_y_roi = -2;
    double max_z_roi = 2;
    double min_z_roi = 2;
} PedestrianLidarParameters;

#endif  // __PEDESTRIAN_LIDAR_CONFIG_HPP__