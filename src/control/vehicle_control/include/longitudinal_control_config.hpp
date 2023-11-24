#ifndef __LONGITUDINAL_CONTROL_CONFIG_HPP__
#define __LONGITUDINAL_CONTROL_CONFIG_HPP__
#pragma once

typedef struct {
    double P_gain;
    double I_gain;
    double D_gain;
    bool use_manual_desired_velocity;
    double manual_desired_velocity;
    double tunnel_velocity;
    bool use_tunnel_lidar;
} LongitudinalControlParameters;

#endif  // __LONGITUDINAL_CONTROL_CONFIG_HPP__