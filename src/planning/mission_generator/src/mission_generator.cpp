#include <mission_generator.hpp>

MissionGenerator::MissionGenerator(ros::NodeHandle &nh_){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/planning.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());
    
    p_global_route_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route", 10);
    p_rviz_lane_pub = nh_.advertise<visualization_msgs::MarkerArray>("/hmi/lane", 10);
    p_mission_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/mission", 10);
    p_rviz_mission_pub = nh_.advertise<jsk_rviz_plugins::OverlayText>("/hmi/mission", 10);

    s_odom_sub = nh_.subscribe("/carla/ego_vehicle/odometry", 10, &MissionGenerator::OdomCallback, this);

    Init();
}

MissionGenerator::~MissionGenerator(){}

void MissionGenerator::OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg){
    mutex_odom.lock();
    m_odom.pose = in_odom_msg->pose;
    m_odom.twist = in_odom_msg->twist;
    mutex_odom.unlock();
}

void MissionGenerator::Init(){
    ProcessINI();
    ReadCSVFile();
}

void MissionGenerator::Run(){
    ProcessINI();
    UpdateState();
    MakeGlobalRoute();
    m_closest_point = FindClosestPoint();
    GenerateMission();
    UpdateRviz();
    if(m_print_count++ % 10 == 0)
    {
        ROS_INFO_STREAM("Mission Generator is running...");
    }

}

void MissionGenerator::Publish(){
    p_global_route_pub.publish(global_route_msg);
    p_rviz_lane_pub.publish(lane_marker_array_msg);
    p_mission_pub.publish(mission_msg);
    p_rviz_mission_pub.publish(rviz_mission_msg);
}

void MissionGenerator::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        // v_ini_parser_.ParseConfig("mission_generator", "xxx",
        //                             mission_generator_params_.xxx);

        ROS_WARN("[Mission Generator] Ini file is updated!\n");
    }
}

void MissionGenerator::ReadCSVFile(){
    std::string currentFilePath = __FILE__;

    size_t lastSlashPos = currentFilePath.find_last_of('/');
    std::string srcDir = currentFilePath.substr(0, lastSlashPos);

    lastSlashPos = srcDir.find_last_of('/');
    std::string pkgDir = srcDir.substr(0, lastSlashPos);

    lastSlashPos = pkgDir.find_last_of('/');
    std::string appDir = pkgDir.substr(0, lastSlashPos);

    lastSlashPos = appDir.find_last_of('/');
    std::string ws_srcDir = appDir.substr(0, lastSlashPos);

    lastSlashPos = ws_srcDir.find_last_of('/');
    std::string wsDir = ws_srcDir.substr(0, lastSlashPos);

    std::string osmPath_lane = "/resources/waypoint.csv";

    std::string map_path_lane = wsDir + osmPath_lane;

    std::ifstream in_lane(map_path_lane);

    if (!in_lane.is_open()) 
    {
      ROS_INFO("Lane File not found");
    }


    std::string s_1;

    while(in_lane)
    {
        getline(in_lane, s_1);
        std::istringstream iss(s_1);
        std::vector<double> temp;


        std::vector<std::string> buf = transForm::split(s_1, ',');
        for (std::vector<std::string>::iterator itr = buf.begin(); itr != buf.end(); ++itr) 
        {
            temp.push_back(transForm::toNumber(*itr));
        }

        geometry_msgs::Point waypoint;
        waypoint.x = temp[0];
        waypoint.y = temp[1];
        m_waypoint_vec.push_back(waypoint);

    }

}

void MissionGenerator::MakeGlobalRoute()
{   
    geometry_msgs::PoseArray temp_global_route;
    temp_global_route.header.frame_id = "map";
    temp_global_route.header.stamp = ros::Time::now();
    for(auto element : m_waypoint_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_global_route.poses.push_back(point);
    }


    global_route_msg = temp_global_route;
}

void MissionGenerator::GenerateMission(){
    if(m_closest_id<350)
    {
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<470)
    { 
        m_mission = STATIC_OBSTACLE_1;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<515)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<600)
    { 
        m_mission = TRAFFIC_LIGHT;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<700)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<970)
    { 
        m_mission = ROTARY;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<1100)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    } 
    else if(m_closest_id<1170)
    { 
        m_mission = TRAFFIC_LIGHT;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<1200)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<1280)
    { 
        m_mission = DYNAMIC_OBSTACLE;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<1420)
    { 
        m_mission = PARKING;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<1590)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<1990)
    { 
        m_mission = TUNNEL;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<2070)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<2270)
    { 
        m_mission = STATIC_OBSTACLE_3;
        mission_msg.data = m_mission;
    }
    else
    {
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }
}

void MissionGenerator::UpdateState()
{
    m_ego_x = m_odom.pose.pose.position.x;
    m_ego_y = m_odom.pose.pose.position.y;

    tf::Quaternion q(m_odom.pose.pose.orientation.x,m_odom.pose.pose.orientation.y,m_odom.pose.pose.orientation.z,m_odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    m_yaw = yaw;

    m_velocity = sqrt(pow(m_odom.twist.twist.linear.x,2)+pow(m_odom.twist.twist.linear.y,2)) * 3.6;  // kph
}

geometry_msgs::Point MissionGenerator::FindClosestPoint(){
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

    ROS_INFO_STREAM("m_closest_id : "<<m_closest_id);

    return m_waypoint_vec[closest_id];
}

void MissionGenerator::UpdateRviz()
{
    // LANE VISUALIZATION

    visualization_msgs::MarkerArray lanes;

    visualization_msgs::Marker lane_center_1;
    lane_center_1.header.frame_id = "map";
    lane_center_1.header.stamp = ros::Time::now();
    lane_center_1.id = 4;
    lane_center_1.action = visualization_msgs::Marker::ADD;
    lane_center_1.type = visualization_msgs::Marker::LINE_LIST;
    lane_center_1.lifetime = ros::Duration();
    lane_center_1.scale.x = 0.3;

    lane_center_1.color.r = 1.;
    lane_center_1.color.g = 1.;
    lane_center_1.color.b = 0.;
    lane_center_1.color.a = 1.;

    lane_center_1.pose.orientation.x = 0.0;
    lane_center_1.pose.orientation.y = 0.0;
    lane_center_1.pose.orientation.z = 0.0;
    lane_center_1.pose.orientation.w = 1.0;

    for(auto point : m_waypoint_vec)
    {
        lane_center_1.points.push_back(point);
    }

    lanes.markers.push_back(lane_center_1);

    visualization_msgs::Marker lane_center_1_road;
    lane_center_1_road.header.frame_id = "map";
    lane_center_1_road.header.stamp = ros::Time::now();
    lane_center_1_road.id = 6;
    lane_center_1_road.action = visualization_msgs::Marker::ADD;
    lane_center_1_road.type = visualization_msgs::Marker::LINE_STRIP;
    lane_center_1_road.lifetime = ros::Duration();
    lane_center_1_road.scale.x = 3.5;

    lane_center_1_road.color.r = 1.;
    lane_center_1_road.color.g = 1.;
    lane_center_1_road.color.b = 1.;
    lane_center_1_road.color.a = 0.5;

    lane_center_1_road.pose.orientation.x = 0.0;
    lane_center_1_road.pose.orientation.y = 0.0;
    lane_center_1_road.pose.orientation.z = 0.0;
    lane_center_1_road.pose.orientation.w = 1.0;

    for(auto point : m_waypoint_vec)
    {
        lane_center_1_road.points.push_back(point);
    }

    lanes.markers.push_back(lane_center_1_road);

    lane_marker_array_msg = lanes;

    // MISSION VISUALIZATION
    jsk_rviz_plugins::OverlayText temp_rviz_mission_msg;
    if(m_mission == STATIC_OBSTACLE_1)
    {
        temp_rviz_mission_msg.text = "STATIC_OBSTACLE_1";
    }
    else if(m_mission == TRAFFIC_LIGHT)
    {
        temp_rviz_mission_msg.text = "TRAFFIC_LIGHT";
    }
    else if(m_mission == ROTARY)
    {
        temp_rviz_mission_msg.text = "ROTARY";
    }
    else if(m_mission == DYNAMIC_OBSTACLE)
    {
        temp_rviz_mission_msg.text = "DYNAMIC_OBSTACLE";
    }
    else if(m_mission == PARKING)
    {
        temp_rviz_mission_msg.text = "PARKING";
    }
    else if(m_mission == TUNNEL)
    {
        temp_rviz_mission_msg.text = "TUNNEL";
    }    
    else if(m_mission == STATIC_OBSTACLE_3)
    {
        temp_rviz_mission_msg.text = "STATIC_OBSTACLE_3";
    }
    else
    {
        temp_rviz_mission_msg.text = "NORMAL_DRIVE";
    }

    rviz_mission_msg = temp_rviz_mission_msg;
}

int main(int argc, char** argv)
{
    std::string node_name = "mission_generator";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    MissionGenerator mission(nh);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {   
        mission.Run();
        mission.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}