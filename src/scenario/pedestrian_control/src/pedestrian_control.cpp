#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <carla_msgs/CarlaWalkerControl.h>
#include <nav_msgs/Odometry.h>

class PedesControl {
public:
    PedesControl();
    ~PedesControl();

    // for pedestrian1
    void PedSpeedCB(const std_msgs::Float32ConstPtr msg);
    void PedOdomCB(const nav_msgs::OdometryConstPtr msg);
    void setPath();
    void PedControl();


    // // for pedestrian2
    // void PedSpeedCB2(const std_msgs::Float32ConstPtr msg);
    // void PedOdomCB2(const nav_msgs::OdometryConstPtr msg);
    // void PedControl_2();
    void VehicleOdomCB(const nav_msgs::OdometryConstPtr msg);

    
    void Run();

private:
    ros::NodeHandle nh;

    // for pedestrian1
    ros::Publisher pub_WalkerControl;
    ros::Subscriber sub_pedes_speed;
    ros::Subscriber sub_pedes_odom;

    nav_msgs::Path pedes_path_msg;
    carla_msgs::CarlaWalkerControl pedes_cmd_msg;
    geometry_msgs::Pose current_pose;

    float set_pedes_speed = 2.2;
    float cur_pedes_speed;
    int waypoint_idx = 0;

    // // for pedestrian2
    // int jump_once = 0;
    // ros::Publisher pub_WalkerControl2;
    // ros::Subscriber sub_pedes_speed2;
    // ros::Subscriber sub_pedes_odom2;
    ros::Subscriber sub_vehicle_odom;

    // nav_msgs::Path pedes_path_msg2;
    // carla_msgs::CarlaWalkerControl pedes_cmd_msg2;
    // geometry_msgs::Pose current_pose2;
    geometry_msgs::Pose vehicle_pose;

    // float cur_pedes_speed2;

    double MIN_DISTANCE = 0.2;

    

};

PedesControl::PedesControl() {
    // for pedestrian1
    pub_WalkerControl = nh.advertise<carla_msgs::CarlaWalkerControl>("/carla/pedestrian/walker_control_cmd", 10);
    sub_pedes_speed = nh.subscribe("/carla/pedestrian/speedometer", 10, &PedesControl::PedSpeedCB, this);
    sub_pedes_odom = nh.subscribe("/carla/pedestrian/odometry", 10, &PedesControl::PedOdomCB, this);
    
    // // for pedestrian2
    // pub_WalkerControl2 = nh.advertise<carla_msgs::CarlaWalkerControl>("/carla/jaywalking_pedestrian2/walker_control_cmd", 10);
    // sub_pedes_speed2 = nh.subscribe("/carla/jaywalking_pedestrian2/speedometer", 10, &PedesControl::PedSpeedCB2, this);
    // sub_pedes_odom2 = nh.subscribe("/carla/jaywalking_pedestrian2/odometry", 10, &PedesControl::PedOdomCB2, this);
    sub_vehicle_odom = nh.subscribe("/carla/ego_vehicle/odometry", 10, &PedesControl::VehicleOdomCB, this);
    
    // set pedestrian path
    // setPath();
}

PedesControl::~PedesControl() {}

void PedesControl::VehicleOdomCB(const nav_msgs::OdometryConstPtr msg) {
    vehicle_pose.position.x = msg->pose.pose.position.x;
    vehicle_pose.position.y = msg->pose.pose.position.y;
}

void PedesControl::PedSpeedCB(const std_msgs::Float32ConstPtr msg) {
    cur_pedes_speed = msg->data;
}

void PedesControl::PedOdomCB(const nav_msgs::OdometryConstPtr msg) {
    current_pose = msg->pose.pose;
}

// // Pedestrian 2
// void PedesControl::PedSpeedCB2(const std_msgs::Float32ConstPtr msg) {
//     cur_pedes_speed2 = msg->data;
// }

// void PedesControl::PedOdomCB2(const nav_msgs::OdometryConstPtr msg) {
//     current_pose2 = msg->pose.pose;
// }

// void PedesControl::setPath() {
//     std::vector<geometry_msgs::PoseStamped> vec_poses;

//     geometry_msgs::PoseStamped first_pose;
//     first_pose.pose.position.x = 128.79124450683594;
//     first_pose.pose.position.y = 296.5928649902344;
//     first_pose.pose.position.z = 0.17040801048278809;
//     first_pose.pose.orientation.x = 0.0016982;
//     first_pose.pose.orientation.y = -0.000932;
//     first_pose.pose.orientation.z = 0.8269665;
//     first_pose.pose.orientation.w = 0.5622478;

//     geometry_msgs::PoseStamped second_pose;
//     second_pose.pose.position.x = 124.82673645019531;
//     second_pose.pose.position.y = 310.50830078125;
//     second_pose.pose.position.z = 0.17005035281181335;
//     second_pose.pose.orientation.x = -0.000316818;
//     second_pose.pose.orientation.y = -0.000529124;
//     second_pose.pose.orientation.z = -0.578414468;
//     second_pose.pose.orientation.w = 0.815742803;

//     for(int i=0; i<100; i++)
//     {
//         if(i%2 != 0) {
//             vec_poses.push_back(second_pose);
//         }
//         else {
//             vec_poses.push_back(first_pose);
//         }
//     }

//     pedes_path_msg.poses = vec_poses;
// }

void PedesControl::PedControl() {
    // if(waypoint_idx >= pedes_path_msg.poses.size()) {
    //     return;
    // }
    
    // geometry_msgs::PoseStamped waypoint = pedes_path_msg.poses[waypoint_idx];
    // double direction_x = waypoint.pose.position.x - current_pose.position.x;
    // double direction_y = waypoint.pose.position.y - current_pose.position.y;
    // double direction_norm = sqrt( pow(direction_x, 2) + pow(direction_y, 2) );

    // if(direction_norm > MIN_DISTANCE) {
    //     pedes_cmd_msg.speed = set_pedes_speed;
    //     pedes_cmd_msg.direction.x = direction_x / direction_norm;
    //     pedes_cmd_msg.direction.y = direction_y / direction_norm;
    // }
    // else {
    //     waypoint_idx++;
    // }


    // !!!!!!!!!! pedestrian 1 !!!!!!!!!!!!!
    geometry_msgs::PoseStamped next_pose;
    next_pose.pose.position.x = -10.6;
    next_pose.pose.position.y = 187.8;

    double direction_x = next_pose.pose.position.x - current_pose.position.x;
    double direction_y = next_pose.pose.position.y - current_pose.position.y;
    double direction_norm = sqrt( pow(direction_x, 2) + pow(direction_y, 2) );

    // calculate distance between vehicle and pedes
    double distance_vehicle_pedestrian_x;
    double distance_vehicle_pedestrian_y;
    double distance_vehicle_pedestrian;
    distance_vehicle_pedestrian_x = current_pose.position.x - vehicle_pose.position.x;
    distance_vehicle_pedestrian_y = current_pose.position.y - vehicle_pose.position.y;
    distance_vehicle_pedestrian = sqrt( pow(distance_vehicle_pedestrian_x, 2) + pow(distance_vehicle_pedestrian_y, 2) );

    ROS_INFO_STREAM(distance_vehicle_pedestrian);

    if(distance_vehicle_pedestrian <= 17) {
        pedes_cmd_msg.speed = 4;
        pedes_cmd_msg.direction.x = direction_x / direction_norm;
        pedes_cmd_msg.direction.y = direction_y / direction_norm;
    }
    else {
        pedes_cmd_msg.speed = 0;
        pedes_cmd_msg.direction.x = direction_x / direction_norm;
        pedes_cmd_msg.direction.y = direction_y / direction_norm;
    }
    
        
    pub_WalkerControl.publish(pedes_cmd_msg);
}

// void PedesControl::PedControl_2() {

//     geometry_msgs::PoseStamped next_pose;
//     next_pose.pose.position.x = 295.5;
//     next_pose.pose.position.y = 200.11;

//     double direction_x = next_pose.pose.position.x - current_pose2.position.x;
//     double direction_y = next_pose.pose.position.y - current_pose2.position.y;
//     double direction_norm = sqrt( pow(direction_x, 2) + pow(direction_y, 2) );

//     // calculate distance between vehicle and pedes
//     double distance_vehicle_pedestrian_x;
//     double distance_vehicle_pedestrian_y;
//     double distance_vehicle_pedestrian;
//     distance_vehicle_pedestrian_x = current_pose2.position.x - vehicle_pose.position.x;
//     distance_vehicle_pedestrian_y = current_pose2.position.y - vehicle_pose.position.y;
//     distance_vehicle_pedestrian = sqrt( pow(distance_vehicle_pedestrian_x, 2) + pow(distance_vehicle_pedestrian_y, 2) );

//     if(distance_vehicle_pedestrian <= 22) {
//         pedes_cmd_msg2.speed = 3;
//         pedes_cmd_msg2.direction.x = direction_x / direction_norm;
//         pedes_cmd_msg2.direction.y = direction_y / direction_norm;
//     }
//     else {
//         pedes_cmd_msg2.speed = 0;
//         pedes_cmd_msg2.direction.x = direction_x / direction_norm;
//         pedes_cmd_msg2.direction.y = direction_y / direction_norm;
//     }

//     if(direction_norm < 11 && jump_once <= 1) {
//         jump_once++;
//         pedes_cmd_msg2.jump = true;
//     }
//     else {
//         pedes_cmd_msg2.jump = false;
//     }

//     std::cout << "distance_vehicle_pedestrian : " << distance_vehicle_pedestrian << "\n";
//     pub_WalkerControl2.publish(pedes_cmd_msg2);

// }

void PedesControl::Run() {
    PedControl();
    // PedControl_2();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pedestrian_control");
    ros::Time::init();

    PedesControl main_task;

    ros::Rate loop_rate(10);

    while(ros::ok())
    {   
        ros::spinOnce();
        main_task.Run();
        loop_rate.sleep();
    }

    return 0;
}