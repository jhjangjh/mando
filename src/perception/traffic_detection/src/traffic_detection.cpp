
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


// Namespace
using namespace lanelet;

class Detection {

public:
    Detection(ros::NodeHandle &nh_);
    ~Detection();

    void Init();
    void TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg);
    void AheadVehicleCallback(const visualization_msgs::MarkerArrayConstPtr &in_ahead_vehicle_info_msg);
    void TrafficLightCallback(const std_msgs::Bool &traffic_light_msg);

    void Run();
    void Publish();

private:
    // Publisher
    ros::Publisher p_traffic_info_pub;
    ros::Publisher p_object_info_pub;

    // Subscriber
    ros::Subscriber s_tf_sub;
    ros::Subscriber s_ahead_vehicle_sub;
    ros::Subscriber s_traffic_light_sub;    // subscribe traffic light signal

    // Variales
    nav_msgs::Odometry m_odom;

};

Detection::Detection(ros::NodeHandle &nh_){
    
    s_tf_sub = nh_.subscribe("/tf",10,&Detection::TfCallback,this);
    s_ahead_vehicle_sub = nh_.subscribe("/carla/markers", 10, &Detection::AheadVehicleCallback, this);
    s_traffic_light_sub = nh_.subscribe("/carla/traffic_lights/status", 10, &Detection::TrafficLightCallback, this);

    Init();
}

Detection::~Detection(){}


void Detection::Init(){

}

void Detection::TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg){
    // listener.lookupTransform("map","ego_vehicle",ros::Time(0),m_ego_vehicle_transform);
    // m_odom.pose.pose.position.x = m_ego_vehicle_transform.getOrigin().x();
    // m_odom.pose.pose.position.y = m_ego_vehicle_transform.getOrigin().y();
    // m_odom.pose.pose.position.z = m_ego_vehicle_transform.getOrigin().z();
}

void Detection::AheadVehicleCallback(const visualization_msgs::MarkerArrayConstPtr &in_ahead_vehicle_info_msg){
}

void Detection::TrafficLightCallback(const std_msgs::Bool &traffic_light_msg){

}


void Detection::Run(){

}

void Detection::Publish(){

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
        detection.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
