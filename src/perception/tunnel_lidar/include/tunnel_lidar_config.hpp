#ifndef __TUNNEL_LIDAR_CONFIG_HPP__
#define __TUNNEL_LIDAR_CONFIG_HPP__
#pragma once

typedef struct {
    double voxelsize;
    double roi_distance;
    double offset;
    double front_distance;
} TunnelLidarParameters;

#endif  // __TUNNEL_LIDAR_CONFIG_HPP__