
// STD header
#include <string>
#include <stdlib.h>
#include <fstream>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// Message header
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <kucudas_msgs/Trajectory.h>
#include <kucudas_msgs/VehicleInformation.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <carla_msgs/CarlaTrafficLightInfo.h>
#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_msgs/CarlaTrafficLightStatus.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>

#define RED 0
#define GREEN 2
#define GREEN_ 4
#define YELLOW 1

#define NO_BLOCK 0
#define _1_BLOCK 1
#define _2_BLOCK 2
#define _1_2_BLOCK 3

#define NORMAL_DRIVE 0
#define TRAFFIC_LIGHT_1 1
#define TRAFFIC_LIGHT_2 2
#define TRAFFIC_LIGHT_3 3
#define TRAFFIC_LIGHT_4 4
#define TRAFFIC_LIGHT_5 5
#define TRAFFIC_LIGHT_6 6
#define TRAFFIC_LIGHT_7 7
#define TRAFFIC_LIGHT_8 8
#define LOOP 9
#define TUNNEL 10

// Namespace
using namespace lanelet;

class Detection {

public:
    Detection(ros::NodeHandle &nh_);
    ~Detection();

    void Init();
    void TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg);
    void AheadVehicleCallback(const visualization_msgs::MarkerArrayConstPtr &in_ahead_vehicle_info_msg);
    void TrafficLightCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &traffic_light_list_msg);
    void GlobalRoute_1_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_1);
    void GlobalRoute_2_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_2);
    void GlobalRoute_3_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_3);
    void MissionState_Callback(const std_msgs::Int8ConstPtr &in_mission_state);

    bool is_Route_1_block(double threshold);
    bool is_Route_2_block(double threshold);
    bool is_Route_3_block(double threshold);
    void Run();
    void Publish(int bolck_msg, int traffic_msg);

private:
    // Publisher
    ros::Publisher p_traffic_info_pub;
    ros::Publisher p_object_block_pub;

    // Subscriber
    ros::Subscriber s_tf_sub;
    ros::Subscriber s_ahead_vehicle_sub;
    ros::Subscriber s_traffic_light_sub;    // subscribe traffic light signal
    ros::Subscriber s_global_route_1;
    ros::Subscriber s_global_route_2;
    ros::Subscriber s_global_route_3;
    ros::Subscriber s_mission_state;

    // Variales
    nav_msgs::Odometry m_odom;
    geometry_msgs::PoseArray m_vehicle_posearray;
    geometry_msgs::PoseArray m_global_route_1;
    geometry_msgs::PoseArray m_global_route_2;
    geometry_msgs::PoseArray m_global_route_3;
    int m_mission_state = NORMAL_DRIVE;
    int m_traffic_light = GREEN;
        
    // tf Variables
    tf::TransformListener listener;
    tf::StampedTransform m_ego_vehicle_transform;

};

Detection::Detection(ros::NodeHandle &nh_){
    s_tf_sub = nh_.subscribe("/tf",10,&Detection::TfCallback,this);
    s_ahead_vehicle_sub = nh_.subscribe("/carla/markers", 10, &Detection::AheadVehicleCallback, this);
    s_traffic_light_sub = nh_.subscribe("/carla/traffic_lights/status", 10, &Detection::TrafficLightCallback, this);
    s_global_route_1 = nh_.subscribe("/adas/planning/global_route1", 1, &Detection::GlobalRoute_1_Callback, this);
    s_global_route_2 = nh_.subscribe("/adas/planning/global_route2", 1, &Detection::GlobalRoute_2_Callback, this);
    s_global_route_3 = nh_.subscribe("/adas/planning/global_route3", 1, &Detection::GlobalRoute_3_Callback, this);
    s_mission_state = nh_.subscribe("/adas/planning/mission", 1, &Detection::MissionState_Callback, this);

    p_object_block_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/block", 10);
    p_traffic_info_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/traffic", 10);
    Init();
}

Detection::~Detection(){}


void Detection::Init(){
    m_global_route_1.poses.clear();
    m_global_route_2.poses.clear();
    m_global_route_3.poses.clear();
}

void Detection::TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg){
    listener.lookupTransform("map","ego_vehicle",ros::Time(0),m_ego_vehicle_transform);
    m_odom.pose.pose.position.x = m_ego_vehicle_transform.getOrigin().x();
    m_odom.pose.pose.position.y = m_ego_vehicle_transform.getOrigin().y();
    m_odom.pose.pose.position.z = m_ego_vehicle_transform.getOrigin().z();
}

void Detection::AheadVehicleCallback(const visualization_msgs::MarkerArrayConstPtr &in_ahead_vehicle_info_msg){
    m_vehicle_posearray.poses.clear();
    double distance = 0.0;
    for(auto point : in_ahead_vehicle_info_msg->markers){
        
        distance = sqrt(pow((point.pose.position.x - m_odom.pose.pose.position.x),2) + pow((point.pose.position.y - m_odom.pose.pose.position.y),2));
        if(distance<10 && distance>1.0){
            ROS_WARN_STREAM("distance : "<<distance);
            m_vehicle_posearray.poses.push_back(point.pose);
        }
    }
}

void Detection::TrafficLightCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &traffic_light_list_msg){
    int container_local_carla_diff = 37;
    m_traffic_light=GREEN;

    for(auto light : traffic_light_list_msg->traffic_lights){

        if((light.id - container_local_carla_diff)==31 && m_mission_state==TRAFFIC_LIGHT_1){
            m_traffic_light=light.state;
        }

        if((light.id - container_local_carla_diff)==29 && m_mission_state==TRAFFIC_LIGHT_2){
            m_traffic_light=light.state;
        }

        if((light.id - container_local_carla_diff)==27 && m_mission_state==TRAFFIC_LIGHT_3){
            m_traffic_light=light.state;
        }

        if((light.id - container_local_carla_diff)==51 && m_mission_state==TRAFFIC_LIGHT_4){
            m_traffic_light=light.state;
        }

        if((light.id - container_local_carla_diff)==47 && m_mission_state==TRAFFIC_LIGHT_5){
            m_traffic_light=light.state;
        }

        if((light.id - container_local_carla_diff)==41 && m_mission_state==TRAFFIC_LIGHT_6){
            m_traffic_light=light.state;
        }

        if((light.id - container_local_carla_diff)==50 && m_mission_state==TRAFFIC_LIGHT_7){
            m_traffic_light=light.state;
        }

        if((light.id - container_local_carla_diff)==26 && m_mission_state==TRAFFIC_LIGHT_8){
            m_traffic_light=light.state;
        }
    }
}

void Detection::GlobalRoute_1_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_1){
    if(m_global_route_1.poses.size()==0){
        for(auto waypoint : in_global_route_1->poses){
            m_global_route_1.poses.push_back(waypoint);
        }
    }
}

void Detection::GlobalRoute_2_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_2){
    if(m_global_route_2.poses.size()==0){
        for(auto waypoint : in_global_route_2->poses){
            m_global_route_2.poses.push_back(waypoint);
        }
    }
}

void Detection::GlobalRoute_3_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_3){
    if(m_global_route_3.poses.size()==0){
        for(auto waypoint : in_global_route_3->poses){
            m_global_route_3.poses.push_back(waypoint);
        }
    }
}

void Detection::MissionState_Callback(const std_msgs::Int8ConstPtr &in_mission_state){
    m_mission_state = in_mission_state->data;
}

bool Detection::is_Route_1_block(double threshold){
    if (m_vehicle_posearray.poses.empty() || m_global_route_1.poses.empty()) {
        return false;
    }

    double closest_distance = DBL_MAX;  // Use DBL_MAX for positive infinity
    geometry_msgs::Pose closest_vehicle_pose;

    for (const auto& vehicle_pose : m_vehicle_posearray.poses) {
        for (const auto& waypoint : m_global_route_1.poses) {
            double distance = sqrt(pow((vehicle_pose.position.x - waypoint.position.x), 2) +
                                   pow((vehicle_pose.position.y - waypoint.position.y), 2));

            if (distance < closest_distance) {
                closest_distance = distance;
                closest_vehicle_pose = vehicle_pose;
            }
        }
    }

    ROS_INFO_STREAM("route 1 : "<<closest_distance);

    return closest_distance<threshold;
}

bool Detection::is_Route_2_block(double threshold){
    if (m_vehicle_posearray.poses.empty() || m_global_route_2.poses.empty()) {
        return false;
    }

    double closest_distance = DBL_MAX;  // Use DBL_MAX for positive infinity
    geometry_msgs::Pose closest_vehicle_pose;

    for (const auto& vehicle_pose : m_vehicle_posearray.poses) {
        for (const auto& waypoint : m_global_route_2.poses) {
            double distance = sqrt(pow((vehicle_pose.position.x - waypoint.position.x), 2) +
                                   pow((vehicle_pose.position.y - waypoint.position.y), 2));

            if (distance < closest_distance) {
                closest_distance = distance;
                closest_vehicle_pose = vehicle_pose;
            }
        }
    }
    ROS_INFO_STREAM("route 2 : "<<closest_distance);

    return closest_distance<threshold;
}

bool Detection::is_Route_3_block(double threshold){
    if (m_vehicle_posearray.poses.empty() || m_global_route_3.poses.empty()) {
        return false;
    }

    double closest_distance = DBL_MAX;  // Use DBL_MAX for positive infinity
    geometry_msgs::Pose closest_vehicle_pose;

    for (const auto& vehicle_pose : m_vehicle_posearray.poses) {
        for (const auto& waypoint : m_global_route_3.poses) {
            double distance = sqrt(pow((vehicle_pose.position.x - waypoint.position.x), 2) +
                                   pow((vehicle_pose.position.y - waypoint.position.y), 2));

            if (distance < closest_distance) {
                closest_distance = distance;
                closest_vehicle_pose = vehicle_pose;
            }
        }
    }

    ROS_INFO_STREAM("route 3 : "<<closest_distance);

    return closest_distance<threshold;
}

void Detection::Run(){
    ROS_INFO_STREAM("\n\n===========================================");
    ROS_INFO_STREAM(m_vehicle_posearray);
    ROS_INFO_STREAM(m_odom.pose.pose);
    bool global_route_1_block=false;
    bool global_route_2_block=false;
    bool global_route_3_block=false;

    global_route_1_block = is_Route_1_block(1.0);
    global_route_2_block = is_Route_2_block(1.0);
    // global_route_3_block = is_Route_3_block(1.0);

    if(global_route_1_block){
        ROS_INFO_STREAM(" 1 block ");
    }
    if(global_route_2_block){
        ROS_INFO_STREAM(" 2 block ");
    }
    if(global_route_3_block){
        ROS_INFO_STREAM(" 3 block ");
    }

    int block_int = NO_BLOCK;
    if(global_route_1_block==true && global_route_2_block==true){       // true, true
        block_int = _1_2_BLOCK;
    }
    else if(global_route_1_block==true && global_route_2_block==false){
        block_int = _1_BLOCK;
    }
    else if(global_route_1_block==false && global_route_2_block==true){
        block_int = _2_BLOCK;
    }
    else if(global_route_1_block==false && global_route_2_block==false){
        block_int = NO_BLOCK;
    }

    int traffic_int = m_traffic_light;

    Publish(block_int, traffic_int);
}

void Detection::Publish(int block_msg, int traffic_msg){
    std_msgs::Int8 o_block_msg;
    o_block_msg.data = block_msg;
    p_object_block_pub.publish(o_block_msg);

    std_msgs::Int8 o_traffic_msg;
    o_traffic_msg.data = traffic_msg;
    p_traffic_info_pub.publish(o_traffic_msg);
}


int main(int argc, char** argv)
{
    std::string node_name = "detection";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    Detection detection(nh);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {   
        detection.Run();
        // detection.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
