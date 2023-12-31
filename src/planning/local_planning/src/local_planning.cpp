#include <local_planning.hpp>

LocalPlanning::LocalPlanning(ros::NodeHandle &nh_) : lanelet_utm_projector(Origin(lanelet_origin)){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/planning.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    p_trajectory_pub = nh_.advertise<kucudas_msgs::Trajectory>("/trajectory", 100);
    p_rviz_trajectory_pub = nh_.advertise<visualization_msgs::MarkerArray>("/hmi/trajectory", 10);

    s_global_route1_sub = nh_.subscribe("/adas/planning/global_route1", 1000, &LocalPlanning::Route1Callback, this);
    s_global_route2_sub = nh_.subscribe("/adas/planning/global_route2", 1000, &LocalPlanning::Route2Callback, this);
    s_loop_route_sub = nh_.subscribe("/adas/planning/loop_route", 1000, &LocalPlanning::Route3Callback, this);
    // s_odom_sub = nh_.subscribe("/carla/ego_vehicle/odometry", 10, &LocalPlanning::OdomCallback, this);
    // s_gnss_sub = nh_.subscribe("/carla/ego_vehicle/gnss", 1, &LocalPlanning::GnssCallback, this);
    s_tf_sub = nh_.subscribe("/tf",1000,&LocalPlanning::TfCallback,this);
    s_vs_sub = nh_.subscribe("/carla/ego_vehicle/vehicle_status",10,&LocalPlanning::VsCallback,this);
    s_object_block_sub = nh_.subscribe("/adas/planning/block", 10, &LocalPlanning::AheadVehicleCallback, this);
    s_mission_sub = nh_.subscribe("/adas/planning/mission", 10, &LocalPlanning::MissionCallback, this);
    s_traffic_light_sub = nh_.subscribe("/adas/planning/traffic", 10, &LocalPlanning::TrafficLightCallback, this);

    Init();
}

LocalPlanning::~LocalPlanning(){}

void LocalPlanning::Route1Callback(const geometry_msgs::PoseArrayConstPtr &in_route1_msg)
{
    if(m_lane_1_vec.size() == 0)
    {
        for(auto waypoint : in_route1_msg->poses)
        {
            m_lane_1_vec.push_back(waypoint.position);
        }
        get_global_route1 = true;
        m_waypoint_vec = m_lane_1_vec;
    }
}

void LocalPlanning::Route2Callback(const geometry_msgs::PoseArrayConstPtr &in_route2_msg)
{
    if(m_lane_2_vec.size() == 0)
    {
        for(auto waypoint : in_route2_msg->poses)
        {
            m_lane_2_vec.push_back(waypoint.position);
        }
        get_global_route2 = true;
    }
}

void LocalPlanning::Route3Callback(const geometry_msgs::PoseArrayConstPtr &in_route3_msg)
{
    if(m_lane_loop_vec.size() == 0)
    {
        for(auto waypoint : in_route3_msg->poses)
        {
            m_lane_loop_vec.push_back(waypoint.position);
        }
        get_loop_route = true;
    }
}

// void LocalPlanning::OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg){
//     // m_odom.pose = in_odom_msg->pose;
//     m_odom.twist = in_odom_msg->twist;
// }

// void LocalPlanning::GnssCallback(const sensor_msgs::NavSatFixConstPtr &in_gnss_msg){
//     m_location_xyz= lanelet_utm_projector.forward(lanelet::GPSPoint{in_gnss_msg->latitude,in_gnss_msg->longitude,0});     // my gnss projection result
//     m_odom.pose.pose.position.x=m_location_xyz.x();
//     m_odom.pose.pose.position.x=m_location_xyz.y();
//     // m_gnss.latitude = in_gnss_msg->latitude;
//     // m_gnss.longitude = in_gnss_msg->longitude;
// }

void LocalPlanning::TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg) {
    listener.lookupTransform("map","ego_vehicle",ros::Time(0),m_ego_vehicle_transform);
    m_odom.pose.pose.position.x = m_ego_vehicle_transform.getOrigin().x();
    m_odom.pose.pose.position.y = m_ego_vehicle_transform.getOrigin().y();
    m_odom.pose.pose.position.z = m_ego_vehicle_transform.getOrigin().z();
    m_odom.pose.pose.orientation.x = m_ego_vehicle_transform.getRotation().x();
    m_odom.pose.pose.orientation.y = m_ego_vehicle_transform.getRotation().y();
    m_odom.pose.pose.orientation.z = m_ego_vehicle_transform.getRotation().z();
    m_odom.pose.pose.orientation.w = m_ego_vehicle_transform.getRotation().w();
}

void LocalPlanning::VsCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_vs_msg){
    vs_velocity = in_vs_msg->velocity * 3.6;        // kph
}

void LocalPlanning::AheadVehicleCallback(const std_msgs::Int8ConstPtr &in_ahead_vehicle_info_msg){
    m_block = in_ahead_vehicle_info_msg->data;
}

void LocalPlanning::MissionCallback(const std_msgs::Int8ConstPtr &in_mission_msg){
    m_mission = in_mission_msg->data;
}

void LocalPlanning::TrafficLightCallback(const std_msgs::Int8ConstPtr &traffic_light_msg){
    // if traffic_signal == True => STOP!!!!!!  (case : Red, Yellow)
    traffic_signal = traffic_light_msg->data;
}


void LocalPlanning::Init(){
    ProcessINI();  

    
}

void LocalPlanning::Run(){
    ProcessINI();
    if(get_global_route1 && get_global_route2 && get_loop_route)
    {
        UpdateState();
        SelectWaypoint();
        MakeTrajectory();
        UpdateRvizTrajectory(m_trajectory);
        if(m_print_count++ % 10 == 0)
        {
            ROS_INFO_STREAM("Local Planning is running...");
        }
    }
    else
    {
        ROS_WARN_STREAM("Waiting Global route...");
    }
}

void LocalPlanning::Publish(){
    p_trajectory_pub.publish(m_trajectory);
    p_rviz_trajectory_pub.publish(m_trajectory_marker_array);
}

void LocalPlanning::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("local_planning", "trajectory_length",
                                    local_planning_params_.trajectory_length);
        v_ini_parser_.ParseConfig("local_planning", "max_speed",
                                    local_planning_params_.max_speed);
        v_ini_parser_.ParseConfig("local_planning", "max_lateral_accelation",
                                    local_planning_params_.max_lateral_accelation);
        v_ini_parser_.ParseConfig("local_planning", "use_acc_mode",
                                    local_planning_params_.use_acc_mode);
        v_ini_parser_.ParseConfig("local_planning", "acc_active_distance",
                                    local_planning_params_.acc_active_distance);
        ROS_WARN("[Local Planning] Ini file is updated!\n");
    }
}

void LocalPlanning::SelectWaypoint(){

    // TBD
    if(m_block==_1_BLOCK){
        m_waypoint_vec = m_lane_2_vec;    
    }
    else if(m_block==_2_BLOCK){
        m_waypoint_vec = m_lane_1_vec;
    }

}

void LocalPlanning::UpdateState()
{
    m_ego_x = m_odom.pose.pose.position.x;
    m_ego_y = m_odom.pose.pose.position.y;

    tf::Quaternion q(m_odom.pose.pose.orientation.x,m_odom.pose.pose.orientation.y,m_odom.pose.pose.orientation.z,m_odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    m_yaw = yaw;

    m_velocity = vs_velocity;  // kph
}

void LocalPlanning::MakeTrajectory()
{
    kucudas_msgs::Trajectory trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = ros::Time::now();
    
    m_closest_point = FindClosestPoint();
    m_last_id = FindLastIndex(m_closest_point, local_planning_params_.trajectory_length);
    for(int i = m_closest_id; i <= m_last_id ; i++)
    {
        kucudas_msgs::TrajectoryPoint trajectory_point;
        trajectory_point.x = m_waypoint_vec[i].x;
        trajectory_point.y = m_waypoint_vec[i].y;
        trajectory_point.yaw = CalculateYaw(i);
        trajectory_point.curvature = CalculateCurvature(i);
        trajectory_point.speed = SpeedProfiling(trajectory_point.curvature);

        trajectory.point.push_back(trajectory_point);
    }

    // ROS_WARN_STREAM("closest curvature : "<<trajectory.point[0].curvature);
    m_trajectory = trajectory;
}

geometry_msgs::Point LocalPlanning::FindClosestPoint(){
    int closest_id = 0;
    double min_distance = HUGE_VAL;
    int index=0;
    for (auto waypoint : m_waypoint_vec)
    {
        double distance = sqrt(pow(waypoint.x-m_ego_x,2)+pow(waypoint.y-m_ego_y,2));
        if(min_distance>distance)
        {
            closest_id = index;
            min_distance = distance;
        }
        index++;
    }
    m_closest_id = closest_id;

    return m_waypoint_vec[closest_id];
}

int LocalPlanning::FindLastIndex(geometry_msgs::Point closest_point, double trajectory_length){
    int target_id;
    double point2point_distance;
    for(target_id = m_closest_id; target_id < m_waypoint_vec.size(); target_id++)
    {
        geometry_msgs::Point searching_point = m_waypoint_vec[target_id];
        point2point_distance = sqrt(pow(closest_point.x-searching_point.x,2)+pow(closest_point.y-searching_point.y,2));
        if(point2point_distance>trajectory_length) {break;}
    }

    return target_id;
}

double LocalPlanning::CalculateYaw(int index)
{
    double yaw;
    if(index == m_waypoint_vec.size()- 1)
    {
        yaw = atan2(m_waypoint_vec[0].y-m_waypoint_vec[index].y,m_waypoint_vec[0].x-m_waypoint_vec[index].x);
    }
    else
    {
        yaw = atan2(m_waypoint_vec[index+1].y-m_waypoint_vec[index].y,m_waypoint_vec[index+1].x-m_waypoint_vec[index].x);
    }

    return yaw;
}

double LocalPlanning::CalculateCurvature(int index)
{
    double curvature;
    if(index == m_waypoint_vec.size()-2)
    {
        geometry_msgs::Point point1 = m_waypoint_vec.at(index);
        geometry_msgs::Point point2 = m_waypoint_vec.at(index+1);
        geometry_msgs::Point point3 = m_waypoint_vec.at(0);

        double d1 = sqrt(pow((point1.x-point2.x),2)+pow((point1.y-point2.y),2));
        double d2 = sqrt(pow((point2.x-point3.x),2)+pow((point2.y-point3.y),2));
        double d3 = sqrt(pow((point3.x-point1.x),2)+pow((point3.y-point1.y),2));

        double a1,a2,a3,b1,b2,b3;

        a1 = point2.x - point1.x;
        a2 = point2.y - point1.y;
        a3 = 0.;
        b1 = point3.x - point1.x;
        b2 = point3.y - point1.y;
        b3 = 0.;

        double area = sqrt(pow(a2*b3-b2*a3,2)+pow(b1*a3-a1*b3,2)+pow(a1*b2-b1*a2,2))/2;

        curvature = 4*area/(d1*d2*d3);       
    }
    else if(index == m_waypoint_vec.size()-1)
    {
        geometry_msgs::Point point1 = m_waypoint_vec.at(index);
        geometry_msgs::Point point2 = m_waypoint_vec.at(0);
        geometry_msgs::Point point3 = m_waypoint_vec.at(1);

        double d1 = sqrt(pow((point1.x-point2.x),2)+pow((point1.y-point2.y),2));
        double d2 = sqrt(pow((point2.x-point3.x),2)+pow((point2.y-point3.y),2));
        double d3 = sqrt(pow((point3.x-point1.x),2)+pow((point3.y-point1.y),2));

        double a1,a2,a3,b1,b2,b3;

        a1 = point2.x - point1.x;
        a2 = point2.y - point1.y;
        a3 = 0.;
        b1 = point3.x - point1.x;
        b2 = point3.y - point1.y;
        b3 = 0.;

        double area = sqrt(pow(a2*b3-b2*a3,2)+pow(b1*a3-a1*b3,2)+pow(a1*b2-b1*a2,2))/2;

        curvature = 4*area/(d1*d2*d3);    
    }
    else
    {
        geometry_msgs::Point point1 = m_waypoint_vec.at(index);
        geometry_msgs::Point point2 = m_waypoint_vec.at(index+1);
        geometry_msgs::Point point3 = m_waypoint_vec.at(index+2);

        double d1 = sqrt(pow((point1.x-point2.x),2)+pow((point1.y-point2.y),2));
        double d2 = sqrt(pow((point2.x-point3.x),2)+pow((point2.y-point3.y),2));
        double d3 = sqrt(pow((point3.x-point1.x),2)+pow((point3.y-point1.y),2));

        double a1,a2,a3,b1,b2,b3;

        a1 = point2.x - point1.x;
        a2 = point2.y - point1.y;
        a3 = 0.;
        b1 = point3.x - point1.x;
        b2 = point3.y - point1.y;
        b3 = 0.;

        double area = sqrt(pow(a2*b3-b2*a3,2)+pow(b1*a3-a1*b3,2)+pow(a1*b2-b1*a2,2))/2;

        curvature = 4*area/(d1*d2*d3);
    }

    return curvature;
}

double LocalPlanning::SpeedProfiling(double curvature)
{
    double speed = local_planning_params_.max_speed/3.6;
    if(local_planning_params_.use_acc_mode)
    {
        double ahead_vehicle_distance = sqrt(pow(m_ahead_vehicle_info.position_x,2)+pow(m_ahead_vehicle_info.position_y,2));
        if(ahead_vehicle_distance<local_planning_params_.acc_active_distance)
        {
            // TO DO
        }
        else
        {
            if(fabs(curvature) > 0.001) {
                double curv_speed = sqrt(local_planning_params_.max_lateral_accelation/fabs(curvature));
                if(curv_speed < speed) {
                    speed = curv_speed;
                }                    
            }
        }
    }
    else
    {
        // v = sqrt(friction * 9.8 / (curvature))
        if(fabs(curvature) > 0.001) {
            double curv_speed = sqrt(local_planning_params_.max_lateral_accelation/fabs(curvature));
            if(curv_speed < speed) {
                speed = curv_speed;
            }                    
        }
    }


    if((((m_mission == TRAFFIC_LIGHT_1) || (m_mission == TRAFFIC_LIGHT_2) || (m_mission == TRAFFIC_LIGHT_3) || (m_mission == TRAFFIC_LIGHT_4) ||
    (m_mission == TRAFFIC_LIGHT_5) || (m_mission == TRAFFIC_LIGHT_6) || (m_mission == TRAFFIC_LIGHT_7) || (m_mission == TRAFFIC_LIGHT_8) ) && (traffic_signal==RED || traffic_signal==YELLOW)) || (m_block == _1_2_BLOCK))
    {
        speed = 0.;
    }

    return speed*3.6;

}

void LocalPlanning::UpdateRvizTrajectory(const kucudas_msgs::Trajectory& trajectory)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker path_marker;
    visualization_msgs::Marker speed_marker;

    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "spatial";
    path_marker.id = 0;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.lifetime = ros::Duration(0.1);

    speed_marker.header.frame_id = "map";
    speed_marker.header.stamp = ros::Time::now();
    speed_marker.ns = "temporal";
    speed_marker.id = 1;
    speed_marker.action = visualization_msgs::Marker::ADD;
    speed_marker.type = visualization_msgs::Marker::LINE_LIST;
    speed_marker.lifetime = ros::Duration(0.1);

    // Line width
    path_marker.scale.x = 0.2f;
    speed_marker.scale = path_marker.scale;

    // Color space
    path_marker.color.r = 0.0f;
    path_marker.color.g = 0.78f;
    path_marker.color.b = 0.58f;
    path_marker.color.a = 0.5f;
    speed_marker.color = path_marker.color;

    for (uint16_t i = 0; i < trajectory.point.size(); i++) {
        geometry_msgs::Point point;
        point.x = trajectory.point[i].x;
        point.y = trajectory.point[i].y;
        point.z = 0.0;
        path_marker.points.push_back(point);
        speed_marker.points.push_back(point);
        point.z = trajectory.point[i].speed / 3.0;
        speed_marker.points.push_back(point);
    }
    marker_array.markers.push_back(path_marker);
    marker_array.markers.push_back(speed_marker);

    m_trajectory_marker_array = marker_array;
}

int main(int argc, char** argv)
{
    std::string node_name = "local_planning";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    LocalPlanning local(nh);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {   
        local.Run();
        local.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}