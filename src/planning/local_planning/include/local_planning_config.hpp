#ifndef __LOCAL_PLANNING_CONFIG_HPP__
#define __LOCAL_PLANNING_CONFIG_HPP__
#pragma once

typedef struct {
    double trajectory_length;
    double max_speed;
    double max_lateral_accelation;
    bool use_acc_mode;
    int acc_active_distance;
} LocalPlanningParameters;

#endif  // __LOCAL_PLANNING_CONFIG_HPP__