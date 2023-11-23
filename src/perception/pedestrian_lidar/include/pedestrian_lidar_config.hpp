#ifndef __PEDESTRIAN_LIDAR_CONFIG_HPP__
#define __PEDESTRIAN_LIDAR_CONFIG_HPP__
#pragma once

typedef struct {
    double voxelsize;
    double roi_distance;
    double offset;
    double front_distance;
} PedestrianLidarParameters;

#endif  // __PEDESTRIAN_LIDAR_CONFIG_HPP__