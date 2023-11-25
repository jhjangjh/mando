// // STD header
// #include <string>
// #include <stdlib.h>
// #include <fstream>

// // ROS header
// #include <ros/ros.h>
// #include <tf/tf.h>

// // Message header
// #include <geometry_msgs/PoseArray.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <kucudas_msgs/Trajectory.h>
// #include <kucudas_msgs/VehicleInformation.h>
// #include <std_msgs/Int8.h>
// #include <std_msgs/Bool.h>
// #include <visualization_msgs/MarkerArray.h>

// // Lanelet
// #include <lanelet2_io/Io.h>
// #include <lanelet2_projection/UTM.h>
// #include <lanelet2_core/primitives/Lanelet.h>

// // tf
// #include <tf/tfMessage.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>

// // carla
// #include <carla_msgs/CarlaEgoVehicleStatus.h>
// #include <carla_msgs/CarlaTrafficLightInfo.h>
// #include <carla_msgs/CarlaTrafficLightInfoList.h>
// #include <carla_msgs/CarlaTrafficLightStatus.h>
// #include <carla_msgs/CarlaTrafficLightStatusList.h>

// #define RED 0
// #define GREEN 2
// #define GREEN_ 4
// #define YELLOW 1

// #define NO_BLOCK 0
// #define _1_BLOCK 1
// #define _2_BLOCK 2
// #define _1_2_BLOCK 3

// // Mission Define
// #define NORMAL_DRIVE_01 11
// #define TRAFFIC_LIGHT_1 1
// #define NORMAL_DRIVE_12 12
// #define TRAFFIC_LIGHT_2 2
// #define NORMAL_DRIVE_23 13
// #define TRAFFIC_LIGHT_3 3
// #define NORMAL_DRIVE_34 14
// #define TRAFFIC_LIGHT_4 4
// #define NORMAL_DRIVE_45 15
// #define TRAFFIC_LIGHT_5 5
// #define NORMAL_DRIVE_56 16
// #define TRAFFIC_LIGHT_6 6
// #define NORMAL_DRIVE_67 17
// #define TRAFFIC_LIGHT_7 7
// #define NORMAL_DRIVE_78 18
// #define TRAFFIC_LIGHT_8 8
// #define LOOP 9
// #define TUNNEL 10 
// #define NORMAL_DRIVE_910 0

// // Namespace
// using namespace lanelet;

// class Detection {

// public:
//     Detection(ros::NodeHandle &nh_);
//     ~Detection();

//     void Init();
//     void TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg);
//     void AheadVehicleCallback(const visualization_msgs::MarkerArrayConstPtr &in_ahead_vehicle_info_msg);
//     void TrafficLightCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &traffic_light_list_msg);
//     void GlobalRoute_1_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_1);
//     void GlobalRoute_2_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_2);
//     void GlobalRoute_3_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_3);
//     void MissionState_Callback(const std_msgs::Int8ConstPtr &in_mission_state);
//     void IDCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &in_id_msg);

//     bool is_Route_1_block(double threshold);
//     bool is_Route_2_block(double threshold);
//     bool is_Route_3_block(double threshold);
//     void Run();
//     void Publish(int bolck_msg, int traffic_msg);

// private:
//     // Publisher
//     ros::Publisher p_traffic_info_pub;
//     ros::Publisher p_object_block_pub;

//     // Subscriber
//     ros::Subscriber s_tf_sub;
//     ros::Subscriber s_ahead_vehicle_sub;
//     ros::Subscriber s_traffic_light_sub;    // subscribe traffic light signal
//     ros::Subscriber s_global_route_1;
//     ros::Subscriber s_global_route_2;
//     ros::Subscriber s_global_route_3;
//     ros::Subscriber s_mission_state;
//     ros::Subscriber s_traffic_id;

//     // Variales
//     nav_msgs::Odometry m_odom;
//     geometry_msgs::PoseArray m_vehicle_posearray;
//     geometry_msgs::PoseArray m_global_route_1;
//     geometry_msgs::PoseArray m_global_route_2;
//     geometry_msgs::PoseArray m_global_route_3;
//     int m_mission_state = NORMAL_DRIVE_01;
//     int m_traffic_light = GREEN;

//     //Traffic light xyz
//     geometry_msgs::Vector3 traffic_1_;
//     geometry_msgs::Vector3 traffic_2_;
//     geometry_msgs::Vector3 traffic_3_;
//     geometry_msgs::Vector3 traffic_4_;
//     geometry_msgs::Vector3 traffic_5_;
//     geometry_msgs::Vector3 traffic_6_;
//     geometry_msgs::Vector3 traffic_7_;
//     geometry_msgs::Vector3 traffic_8_;

//     int traffic_1_id_;
//     int traffic_2_id_;
//     int traffic_3_id_;
//     int traffic_4_id_;
//     int traffic_5_id_;
//     int traffic_6_id_;
//     int traffic_7_id_;
//     int traffic_8_id_;

//     bool traffic_passed_1 = false;
//     bool traffic_passed_2 = false;
//     bool traffic_passed_3 = false;
//     bool traffic_passed_4 = false;
//     bool traffic_passed_5 = false;
//     bool traffic_passed_6 = false;
//     bool traffic_passed_7 = false;
//     bool traffic_passed_8 = false;

        
//     // tf Variables
//     tf::TransformListener listener;
//     tf::StampedTransform m_ego_vehicle_transform;

// };

// Detection::Detection(ros::NodeHandle &nh_){
//     s_tf_sub = nh_.subscribe("/tf",10,&Detection::TfCallback,this);
//     s_ahead_vehicle_sub = nh_.subscribe("/carla/markers", 10, &Detection::AheadVehicleCallback, this);
//     s_traffic_light_sub = nh_.subscribe("/carla/traffic_lights/status", 10, &Detection::TrafficLightCallback, this);
//     s_global_route_1 = nh_.subscribe("/adas/planning/global_route1", 1, &Detection::GlobalRoute_1_Callback, this);
//     s_global_route_2 = nh_.subscribe("/adas/planning/global_route2", 1, &Detection::GlobalRoute_2_Callback, this);
//     s_global_route_3 = nh_.subscribe("/adas/planning/global_route3", 1, &Detection::GlobalRoute_3_Callback, this);
//     s_mission_state = nh_.subscribe("/adas/planning/mission", 1, &Detection::MissionState_Callback, this);
//     s_traffic_id = nh_.subscribe("/carla/traffic_lights/info", 10, &Detection::IDCallback, this);

//     p_object_block_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/block", 10);
//     p_traffic_info_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/traffic", 10);
//     Init();
// }

// Detection::~Detection(){}


// void Detection::Init(){
//     m_global_route_1.poses.clear();
//     m_global_route_2.poses.clear();
//     m_global_route_3.poses.clear();

//     traffic_1_.x = -67.82;
//     traffic_1_.y = -121.06;

//     traffic_2_.x = -68.34;
//     traffic_2_.y = 12.37;

//     traffic_3_.x = 20.46;
//     traffic_3_.y = 187.89;

//     traffic_4_.x = 94.99;
//     traffic_4_.y = 184.36;

//     traffic_5_.x = 73.22;
//     traffic_5_.y = 123.29;

//     traffic_6_.x = 163.13;
//     traffic_6_.y = 123.77;

//     traffic_7_.x = 163.67;
//     traffic_7_.y = 216.69;

//     traffic_8_.x = 224.50;
//     traffic_8_.y = -80.86;

// }

// void Detection::TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg){
//     listener.lookupTransform("map","ego_vehicle",ros::Time(0),m_ego_vehicle_transform);
//     m_odom.pose.pose.position.x = m_ego_vehicle_transform.getOrigin().x();
//     m_odom.pose.pose.position.y = m_ego_vehicle_transform.getOrigin().y();
//     m_odom.pose.pose.position.z = m_ego_vehicle_transform.getOrigin().z();
// }

// void Detection::AheadVehicleCallback(const visualization_msgs::MarkerArrayConstPtr &in_ahead_vehicle_info_msg){
//     m_vehicle_posearray.poses.clear();
//     double distance = 0.0;
//     for(auto point : in_ahead_vehicle_info_msg->markers){
        
//         distance = sqrt(pow((point.pose.position.x - m_odom.pose.pose.position.x),2) + pow((point.pose.position.y - m_odom.pose.pose.position.y),2));
//         if(distance<15 && distance>1.0){
//             // ROS_INFO_STREAM("distance : "<<distance);
//             m_vehicle_posearray.poses.push_back(point.pose);
//         }
//     }
// }

// void Detection::IDCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &in_id_msg){
//     double min_dist = DBL_MAX;
    
//     //find traffic 1 id
//     for(auto light : in_id_msg->traffic_lights){
//         // To Do
//         double distance = std::sqrt(std::pow((light.transform.position.x - traffic_1_.x), 2) +
//                                     std::pow((light.transform.position.y - traffic_1_.y), 2));

//         if (distance < min_dist) {
//             min_dist = distance;
//             traffic_1_id_ = light.id;
//         }
//     }
//     min_dist = DBL_MAX;

//     for(auto light : in_id_msg->traffic_lights){
//         // To Do
//         double distance = std::sqrt(std::pow((light.transform.position.x - traffic_2_.x), 2) +
//                                     std::pow((light.transform.position.y - traffic_2_.y), 2));

//         if (distance < min_dist) {
//             min_dist = distance;
//             traffic_2_id_ = light.id;
//         }
//     }
//     min_dist = DBL_MAX;

//     for(auto light : in_id_msg->traffic_lights){
//         // To Do
//         double distance = std::sqrt(std::pow((light.transform.position.x - traffic_3_.x), 2) +
//                                     std::pow((light.transform.position.y - traffic_3_.y), 2));

//         if (distance < min_dist) {
//             min_dist = distance;
//             traffic_3_id_ = light.id;
//         }
//     }
//     min_dist = DBL_MAX;

//     for(auto light : in_id_msg->traffic_lights){
//         // To Do
//         double distance = std::sqrt(std::pow((light.transform.position.x - traffic_4_.x), 2) +
//                                     std::pow((light.transform.position.y - traffic_4_.y), 2));

//         if (distance < min_dist) {
//             min_dist = distance;
//             traffic_4_id_ = light.id;
//         }
//     }
//     min_dist = DBL_MAX;

//     for(auto light : in_id_msg->traffic_lights){
//         // To Do
//         double distance = std::sqrt(std::pow((light.transform.position.x - traffic_5_.x), 2) +
//                                     std::pow((light.transform.position.y - traffic_5_.y), 2));

//         if (distance < min_dist) {
//             min_dist = distance;
//             traffic_5_id_ = light.id;
//         }
//     }
//     min_dist = DBL_MAX;

//     for(auto light : in_id_msg->traffic_lights){
//         // To Do
//         double distance = std::sqrt(std::pow((light.transform.position.x - traffic_6_.x), 2) +
//                                     std::pow((light.transform.position.y - traffic_6_.y), 2));

//         if (distance < min_dist) {
//             min_dist = distance;
//             traffic_6_id_ = light.id;
//         }
//     }
//     min_dist = DBL_MAX;

//     for(auto light : in_id_msg->traffic_lights){
//         // To Do
//         double distance = std::sqrt(std::pow((light.transform.position.x - traffic_7_.x), 2) +
//                                     std::pow((light.transform.position.y - traffic_7_.y), 2));

//         if (distance < min_dist) {
//             min_dist = distance;
//             traffic_7_id_ = light.id;
//         }
//     }
//     min_dist = DBL_MAX;

//     for(auto light : in_id_msg->traffic_lights){
//         // To Do
//         double distance = std::sqrt(std::pow((light.transform.position.x - traffic_8_.x), 2) +
//                                     std::pow((light.transform.position.y - traffic_8_.y), 2));

//         if (distance < min_dist) {
//             min_dist = distance;
//             traffic_8_id_ = light.id;
//         }
//     }

//     ROS_INFO_STREAM("1 id : "<<traffic_1_id_);
//     ROS_INFO_STREAM("2 id : "<<traffic_2_id_);
//     ROS_INFO_STREAM("3 id : "<<traffic_3_id_);
//     ROS_INFO_STREAM("4 id : "<<traffic_4_id_);
//     ROS_INFO_STREAM("5 id : "<<traffic_5_id_);
//     ROS_INFO_STREAM("6 id : "<<traffic_6_id_);
//     ROS_INFO_STREAM("7 id : "<<traffic_7_id_);
//     ROS_INFO_STREAM("8 id : "<<traffic_8_id_);
// }


// void Detection::TrafficLightCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &traffic_light_list_msg){
//     int container_local_carla_diff = 0;
//     m_traffic_light=GREEN;

//     for(auto light : traffic_light_list_msg->traffic_lights){
//         if((light.id==traffic_1_id_) && (traffic_passed_1==false) &&(m_mission_state!=TRAFFIC_LIGHT_1)){
//             ROS_INFO_STREAM("0~1");
//             m_traffic_light=light.state;
//         }

//         if((light.id==traffic_1_id_) && (m_mission_state==TRAFFIC_LIGHT_1)){
//             ROS_INFO_STREAM("1");
//             m_traffic_light=light.state;
//             traffic_passed_1=true;
//         }

//         if((light.id==traffic_2_id_) && (traffic_passed_1==true) && (traffic_passed_2==false) &&(m_mission_state!=TRAFFIC_LIGHT_1)){
//             ROS_INFO_STREAM("1~2");
//             m_traffic_light=light.state;
//         }

//         if((light.id==traffic_2_id_) && (m_mission_state==TRAFFIC_LIGHT_2)){
//             ROS_INFO_STREAM("2");
//             m_traffic_light=light.state;
//             traffic_passed_2=true;
//         }

//         if((light.id==traffic_3_id_) && (traffic_passed_2==true) && (traffic_passed_3==false) &&(m_mission_state!=TRAFFIC_LIGHT_2)){
//             ROS_INFO_STREAM("2~3");
//             m_traffic_light=light.state;
//         }


//         if((light.id==traffic_3_id_) && (m_mission_state==TRAFFIC_LIGHT_3)){
//             ROS_INFO_STREAM("3");
//             m_traffic_light=light.state;
//             traffic_passed_3=true;
//         }

//         if((light.id==traffic_4_id_) && (traffic_passed_3==true) && (traffic_passed_4==false) &&(m_mission_state!=TRAFFIC_LIGHT_3)){
//             ROS_INFO_STREAM("3~4");
//             m_traffic_light=light.state;
//         }

//         if(light.id==traffic_4_id_ && m_mission_state==TRAFFIC_LIGHT_4){
//             ROS_INFO_STREAM("4");
//             m_traffic_light=light.state;
//             traffic_passed_4=true;
//         }

//         if((light.id==traffic_5_id_) && (traffic_passed_4==true) && (traffic_passed_5==false) &&(m_mission_state!=TRAFFIC_LIGHT_4)){
//             ROS_INFO_STREAM("4~5");
//             m_traffic_light=light.state;
//         }

//         if(light.id==traffic_5_id_ && m_mission_state==TRAFFIC_LIGHT_5){
//             ROS_INFO_STREAM("5");
//             m_traffic_light=light.state;
//             traffic_passed_5=true;
//         }

//         if((light.id==traffic_6_id_) && (traffic_passed_5==true) && (traffic_passed_6==false) &&(m_mission_state!=TRAFFIC_LIGHT_6)){
//             ROS_INFO_STREAM("5~6");
//             m_traffic_light=light.state;
//         }

//         if(light.id==traffic_6_id_ && m_mission_state==TRAFFIC_LIGHT_6){
//             ROS_INFO_STREAM("6");
//             m_traffic_light=light.state;
//             traffic_passed_6=true;
//         }

//         if((light.id==traffic_7_id_) && (traffic_passed_6==true) && (traffic_passed_7==false) &&(m_mission_state!=TRAFFIC_LIGHT_7)){
//             ROS_INFO_STREAM("6~7");
//             m_traffic_light=light.state;
//         }

//         if(light.id==traffic_7_id_ && m_mission_state==TRAFFIC_LIGHT_7){
//             ROS_INFO_STREAM("7");
//             m_traffic_light=light.state;
//             traffic_passed_7=true;
//         }

//         if((light.id==traffic_8_id_) && (traffic_passed_7==true) && (traffic_passed_8==false) &&(m_mission_state!=TRAFFIC_LIGHT_8)){
//             ROS_INFO_STREAM("7~8");
//             m_traffic_light=light.state;
//         }

//         if(light.id==traffic_8_id_ && m_mission_state==TRAFFIC_LIGHT_8){
//             ROS_INFO_STREAM("8");
//             m_traffic_light=light.state;
//             traffic_passed_8=true;
//         }

//     }
//         if(m_traffic_light==RED){
//             ROS_INFO_STREAM("RED");
//         }
//         else if(m_traffic_light==GREEN){
//             ROS_INFO_STREAM("Green");
//         }
//         else if(m_traffic_light==GREEN){
//             ROS_INFO_STREAM("Green");
//         }
// }

// void Detection::GlobalRoute_1_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_1){
//     if(m_global_route_1.poses.size()==0){
//         for(auto waypoint : in_global_route_1->poses){
//             m_global_route_1.poses.push_back(waypoint);
//         }
//     }
// }

// void Detection::GlobalRoute_2_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_2){
//     if(m_global_route_2.poses.size()==0){
//         for(auto waypoint : in_global_route_2->poses){
//             m_global_route_2.poses.push_back(waypoint);
//         }
//     }
// }

// void Detection::GlobalRoute_3_Callback(const geometry_msgs::PoseArrayConstPtr &in_global_route_3){
//     if(m_global_route_3.poses.size()==0){
//         for(auto waypoint : in_global_route_3->poses){
//             m_global_route_3.poses.push_back(waypoint);
//         }
//     }
// }

// void Detection::MissionState_Callback(const std_msgs::Int8ConstPtr &in_mission_state){
//     m_mission_state = in_mission_state->data;
// }

// bool Detection::is_Route_1_block(double threshold){
//     if (m_vehicle_posearray.poses.empty() || m_global_route_1.poses.empty()) {
//         return false;
//     }

//     double closest_distance = DBL_MAX;  // Use DBL_MAX for positive infinity
//     geometry_msgs::Pose closest_vehicle_pose;

//     for (const auto& vehicle_pose : m_vehicle_posearray.poses) {
//         for (const auto& waypoint : m_global_route_1.poses) {
//             double distance = sqrt(pow((vehicle_pose.position.x - waypoint.position.x), 2) +
//                                    pow((vehicle_pose.position.y - waypoint.position.y), 2));

//             if (distance < closest_distance) {
//                 closest_distance = distance;
//                 closest_vehicle_pose = vehicle_pose;
//             }
//         }
//     }

//     ROS_INFO_STREAM("route 1 : "<<closest_distance);

//     return closest_distance<threshold;
// }

// bool Detection::is_Route_2_block(double threshold){
//     if (m_vehicle_posearray.poses.empty() || m_global_route_2.poses.empty()) {
//         return false;
//     }

//     double closest_distance = DBL_MAX;  // Use DBL_MAX for positive infinity
//     geometry_msgs::Pose closest_vehicle_pose;

//     for (const auto& vehicle_pose : m_vehicle_posearray.poses) {
//         for (const auto& waypoint : m_global_route_2.poses) {
//             double distance = sqrt(pow((vehicle_pose.position.x - waypoint.position.x), 2) +
//                                    pow((vehicle_pose.position.y - waypoint.position.y), 2));

//             if (distance < closest_distance) {
//                 closest_distance = distance;
//                 closest_vehicle_pose = vehicle_pose;
//             }
//         }
//     }
//     ROS_INFO_STREAM("route 2 : "<<closest_distance);

//     return closest_distance<threshold;
// }

// bool Detection::is_Route_3_block(double threshold){
//     if (m_vehicle_posearray.poses.empty() || m_global_route_3.poses.empty()) {
//         return false;
//     }

//     double closest_distance = DBL_MAX;  // Use DBL_MAX for positive infinity
//     geometry_msgs::Pose closest_vehicle_pose;

//     for (const auto& vehicle_pose : m_vehicle_posearray.poses) {
//         for (const auto& waypoint : m_global_route_3.poses) {
//             double distance = sqrt(pow((vehicle_pose.position.x - waypoint.position.x), 2) +
//                                    pow((vehicle_pose.position.y - waypoint.position.y), 2));

//             if (distance < closest_distance) {
//                 closest_distance = distance;
//                 closest_vehicle_pose = vehicle_pose;
//             }
//         }
//     }

//     ROS_INFO_STREAM("route 3 : "<<closest_distance);

//     return closest_distance<threshold;
// }

// void Detection::Run(){
//     ROS_INFO_STREAM("\n\n===========================================");
//     ROS_INFO_STREAM(m_vehicle_posearray);
//     ROS_INFO_STREAM(m_odom.pose.pose);
//     bool global_route_1_block=false;
//     bool global_route_2_block=false;
//     bool global_route_3_block=false;

//     global_route_1_block = is_Route_1_block(1.0);
//     global_route_2_block = is_Route_2_block(1.0);
//     // global_route_3_block = is_Route_3_block(1.0);

//     if(global_route_1_block){
//         ROS_INFO_STREAM(" 1 block ");
//     }
//     if(global_route_2_block){
//         ROS_INFO_STREAM(" 2 block ");
//     }
//     if(global_route_3_block){
//         ROS_INFO_STREAM(" 3 block ");
//     }

//     int block_int = NO_BLOCK;
//     if(global_route_1_block==true && global_route_2_block==true){       // true, true
//         block_int = _1_2_BLOCK;
//     }
//     else if(global_route_1_block==true && global_route_2_block==false){
//         block_int = _1_BLOCK;
//     }
//     else if(global_route_1_block==false && global_route_2_block==true){
//         block_int = _2_BLOCK;
//     }
//     else if(global_route_1_block==false && global_route_2_block==false){
//         block_int = NO_BLOCK;
//     }

//     int traffic_int = m_traffic_light;

//     Publish(block_int, traffic_int);
// }

// void Detection::Publish(int block_msg, int traffic_msg){
//     std_msgs::Int8 o_block_msg;
//     o_block_msg.data = block_msg;
//     p_object_block_pub.publish(o_block_msg);

//     std_msgs::Int8 o_traffic_msg;
//     o_traffic_msg.data = traffic_msg;
//     p_traffic_info_pub.publish(o_traffic_msg);
// }


// int main(int argc, char** argv)
// {
//     std::string node_name = "detection";
//     ros::init(argc, argv, node_name);
//     ros::NodeHandle nh("~");

//     Detection detection(nh);
//     ros::Rate loop_rate(30);

//     while(ros::ok())
//     {   
//         detection.Run();
//         // detection.Publish();

//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }


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
    void IDCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &in_id_msg);

    bool is_Route_1_block(double threshold);
    bool is_Route_2_block(double threshold);
    bool is_Route_3_block(double threshold);
    void Run();
    void Publish(int bolck_msg, int traffic_msg);
    void get_signal();

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
    ros::Subscriber s_traffic_id;

    // Variales
    nav_msgs::Odometry m_odom;
    geometry_msgs::PoseArray m_vehicle_posearray;
    geometry_msgs::PoseArray m_global_route_1;
    geometry_msgs::PoseArray m_global_route_2;
    geometry_msgs::PoseArray m_global_route_3;
    int m_mission_state = NORMAL_DRIVE_01;
    int m_traffic_light = GREEN;



    //Traffic light xyz
    geometry_msgs::Vector3 traffic_1_;
    geometry_msgs::Vector3 traffic_2_;
    geometry_msgs::Vector3 traffic_3_;
    geometry_msgs::Vector3 traffic_4_;
    geometry_msgs::Vector3 traffic_5_;
    geometry_msgs::Vector3 traffic_6_;
    geometry_msgs::Vector3 traffic_7_;
    geometry_msgs::Vector3 traffic_8_;

    int traffic_1_id_;
    int traffic_2_id_;
    int traffic_3_id_;
    int traffic_4_id_;
    int traffic_5_id_;
    int traffic_6_id_;
    int traffic_7_id_;
    int traffic_8_id_;

    bool traffic_passed_1 = false;
    bool traffic_passed_2 = false;
    bool traffic_passed_3 = false;
    bool traffic_passed_4 = false;
    bool traffic_passed_5 = false;
    bool traffic_passed_6 = false;
    bool traffic_passed_7 = false;
    bool traffic_passed_8 = false;

    carla_msgs::CarlaTrafficLightStatusList traffic_list;
        
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
    s_traffic_id = nh_.subscribe("/carla/traffic_lights/info", 10, &Detection::IDCallback, this);

    p_object_block_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/block", 10);
    p_traffic_info_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/traffic", 10);
    Init();
}

Detection::~Detection(){}


void Detection::Init(){
    m_global_route_1.poses.clear();
    m_global_route_2.poses.clear();
    m_global_route_3.poses.clear();

    traffic_1_.x = -67.82;
    traffic_1_.y = -121.06;

    traffic_2_.x = -68.34;
    traffic_2_.y = 12.37;

    traffic_3_.x = 20.46;
    traffic_3_.y = 187.89;

    traffic_4_.x = 94.99;
    traffic_4_.y = 184.36;

    traffic_5_.x = 73.22;
    traffic_5_.y = 123.29;

    traffic_6_.x = 163.13;
    traffic_6_.y = 123.77;

    traffic_7_.x = 163.67;
    traffic_7_.y = 216.69;

    traffic_8_.x = 224.50;
    traffic_8_.y = -80.86;

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
        if(distance<15 && distance>1.0){
            // ROS_INFO_STREAM("distance : "<<distance);
            m_vehicle_posearray.poses.push_back(point.pose);
        }
    }
}

void Detection::IDCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &in_id_msg){
    double min_dist = DBL_MAX;
    
    //find traffic 1 id
    for(auto light : in_id_msg->traffic_lights){
        // To Do
        double distance = std::sqrt(std::pow((light.transform.position.x - traffic_1_.x), 2) +
                                    std::pow((light.transform.position.y - traffic_1_.y), 2));

        if (distance < min_dist) {
            min_dist = distance;
            traffic_1_id_ = light.id;
        }
    }
    min_dist = DBL_MAX;

    for(auto light : in_id_msg->traffic_lights){
        // To Do
        double distance = std::sqrt(std::pow((light.transform.position.x - traffic_2_.x), 2) +
                                    std::pow((light.transform.position.y - traffic_2_.y), 2));

        if (distance < min_dist) {
            min_dist = distance;
            traffic_2_id_ = light.id;
        }
    }
    min_dist = DBL_MAX;

    for(auto light : in_id_msg->traffic_lights){
        // To Do
        double distance = std::sqrt(std::pow((light.transform.position.x - traffic_3_.x), 2) +
                                    std::pow((light.transform.position.y - traffic_3_.y), 2));

        if (distance < min_dist) {
            min_dist = distance;
            traffic_3_id_ = light.id;
        }
    }
    min_dist = DBL_MAX;

    for(auto light : in_id_msg->traffic_lights){
        // To Do
        double distance = std::sqrt(std::pow((light.transform.position.x - traffic_4_.x), 2) +
                                    std::pow((light.transform.position.y - traffic_4_.y), 2));

        if (distance < min_dist) {
            min_dist = distance;
            traffic_4_id_ = light.id;
        }
    }
    min_dist = DBL_MAX;

    for(auto light : in_id_msg->traffic_lights){
        // To Do
        double distance = std::sqrt(std::pow((light.transform.position.x - traffic_5_.x), 2) +
                                    std::pow((light.transform.position.y - traffic_5_.y), 2));

        if (distance < min_dist) {
            min_dist = distance;
            traffic_5_id_ = light.id;
        }
    }
    min_dist = DBL_MAX;

    for(auto light : in_id_msg->traffic_lights){
        // To Do
        double distance = std::sqrt(std::pow((light.transform.position.x - traffic_6_.x), 2) +
                                    std::pow((light.transform.position.y - traffic_6_.y), 2));

        if (distance < min_dist) {
            min_dist = distance;
            traffic_6_id_ = light.id;
        }
    }
    min_dist = DBL_MAX;

    for(auto light : in_id_msg->traffic_lights){
        // To Do
        double distance = std::sqrt(std::pow((light.transform.position.x - traffic_7_.x), 2) +
                                    std::pow((light.transform.position.y - traffic_7_.y), 2));

        if (distance < min_dist) {
            min_dist = distance;
            traffic_7_id_ = light.id;
        }
    }
    min_dist = DBL_MAX;

    for(auto light : in_id_msg->traffic_lights){
        // To Do
        double distance = std::sqrt(std::pow((light.transform.position.x - traffic_8_.x), 2) +
                                    std::pow((light.transform.position.y - traffic_8_.y), 2));

        if (distance < min_dist) {
            min_dist = distance;
            traffic_8_id_ = light.id;
        }
    }

    ROS_INFO_STREAM("1 id : "<<traffic_1_id_);
    ROS_INFO_STREAM("2 id : "<<traffic_2_id_);
    ROS_INFO_STREAM("3 id : "<<traffic_3_id_);
    ROS_INFO_STREAM("4 id : "<<traffic_4_id_);
    ROS_INFO_STREAM("5 id : "<<traffic_5_id_);
    ROS_INFO_STREAM("6 id : "<<traffic_6_id_);
    ROS_INFO_STREAM("7 id : "<<traffic_7_id_);
    ROS_INFO_STREAM("8 id : "<<traffic_8_id_);
}


void Detection::TrafficLightCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &traffic_light_list_msg){
    traffic_list = *traffic_light_list_msg;
}

void Detection::get_signal(){

    for(auto light : traffic_list.traffic_lights){
        if((light.id==traffic_1_id_) && (m_mission_state==NORMAL_DRIVE_01)){
            ROS_INFO_STREAM("0~1");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_1_id_) && (m_mission_state==TRAFFIC_LIGHT_1)){
            ROS_INFO_STREAM("1");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_2_id_) && (m_mission_state==NORMAL_DRIVE_12)){
            ROS_INFO_STREAM("1~2");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_2_id_) && (m_mission_state==TRAFFIC_LIGHT_2)){
            ROS_INFO_STREAM("2");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_3_id_) && (m_mission_state==NORMAL_DRIVE_23)){
            ROS_INFO_STREAM("2~3");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_3_id_) && (m_mission_state==TRAFFIC_LIGHT_3)){
            ROS_INFO_STREAM("3");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_4_id_) &&(m_mission_state==NORMAL_DRIVE_34)){
            ROS_INFO_STREAM("3~4");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_4_id_) && (m_mission_state==TRAFFIC_LIGHT_4)){
            ROS_INFO_STREAM("4");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_5_id_) && (m_mission_state==NORMAL_DRIVE_45)){
            ROS_INFO_STREAM("4~5");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_5_id_) && (m_mission_state==TRAFFIC_LIGHT_5)){
            ROS_INFO_STREAM("5");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_6_id_) && (m_mission_state==NORMAL_DRIVE_56)){
            ROS_INFO_STREAM("5~6");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_6_id_) && (m_mission_state==TRAFFIC_LIGHT_6)){
            ROS_INFO_STREAM("6");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_7_id_) && (m_mission_state==NORMAL_DRIVE_67)){
            ROS_INFO_STREAM("6~7");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_7_id_) && (m_mission_state==TRAFFIC_LIGHT_7)){
            ROS_INFO_STREAM("7");
            m_traffic_light=light.state;
        }

        if((light.id==traffic_8_id_) &&(m_mission_state==NORMAL_DRIVE_78)){
            ROS_INFO_STREAM("7~8");
            m_traffic_light=light.state;
        }

        if(light.id==traffic_8_id_ && m_mission_state==TRAFFIC_LIGHT_8){
            ROS_INFO_STREAM("8");
            m_traffic_light=light.state;
        }


    }

    if(m_traffic_light==RED){
        ROS_INFO_STREAM("RED");
    }
    else if(m_traffic_light==GREEN){
        ROS_INFO_STREAM("Green");
    }
    else if(m_traffic_light==GREEN_){
        ROS_INFO_STREAM("Green");
    }
    else if(m_traffic_light==YELLOW){
        ROS_INFO_STREAM("Yellow");
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
    
    get_signal();

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