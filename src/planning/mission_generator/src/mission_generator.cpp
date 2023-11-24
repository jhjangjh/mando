#include <mission_generator.hpp>

MissionGenerator::MissionGenerator(ros::NodeHandle &nh_) : lanelet_utm_projector(Origin(lanelet_origin)){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/planning.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());
    
    p_global_route1_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route1", 10);
    p_global_route2_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route2", 10);
    p_global_route3_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route3", 10);
    p_rviz_lane_pub = nh_.advertise<visualization_msgs::MarkerArray>("/hmi/lane", 10);
    p_mission_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/mission", 10);
    p_rviz_mission_pub = nh_.advertise<jsk_rviz_plugins::OverlayText>("/hmi/mission", 10);

    // s_odom_sub = nh_.subscribe("/carla/ego_vehicle/odometry", 10, &MissionGenerator::OdomCallback, this);
    // s_gnss_sub = nh_.subscribe("/carla/ego_vehicle/gnss", 1, &MissionGenerator::GnssCallback, this);
    s_tf_sub = nh_.subscribe("/tf",10,&MissionGenerator::TfCallback,this);
    s_vs_sub = nh_.subscribe("/carla/ego_vehicle/vehicle_status",10,&MissionGenerator::VsCallback,this);
    Init();
}

MissionGenerator::~MissionGenerator(){}

// void MissionGenerator::OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg){
//     // m_odom.pose = in_odom_msg->pose;
//     m_odom.twist = in_odom_msg->twist;
// }

// void MissionGenerator::GnssCallback(const sensor_msgs::NavSatFixConstPtr &in_gnss_msg){
//     m_location_xyz= lanelet_utm_projector.forward(lanelet::GPSPoint{in_gnss_msg->latitude,in_gnss_msg->longitude,0});     // my gnss projection result
//     m_odom.pose.pose.position.x=m_location_xyz.x();
//     m_odom.pose.pose.position.x=m_location_xyz.y();
//     double x = m_odom.pose.pose.position.x;
//     double y = m_odom.pose.pose.position.y;
//     // m_gnss.latitude = in_gnss_msg->latitude;
//     // m_gnss.longitude = in_gnss_msg->longitude;
//     std::cout << "odom_x,y: " << x << ", " << y << std::endl;
// }

void MissionGenerator::TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg) {
    listener.lookupTransform("map","ego_vehicle",ros::Time(0),m_ego_vehicle_transform);
    m_odom.pose.pose.position.x = m_ego_vehicle_transform.getOrigin().x();
    m_odom.pose.pose.position.y = m_ego_vehicle_transform.getOrigin().y();
    m_odom.pose.pose.position.z = m_ego_vehicle_transform.getOrigin().z();
    m_odom.pose.pose.orientation.x = m_ego_vehicle_transform.getRotation().x();
    m_odom.pose.pose.orientation.y = m_ego_vehicle_transform.getRotation().y();
    m_odom.pose.pose.orientation.z = m_ego_vehicle_transform.getRotation().z();
    m_odom.pose.pose.orientation.w = m_ego_vehicle_transform.getRotation().w();
}

void MissionGenerator::VsCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_vs_msg){
    vs_velocity = in_vs_msg->velocity * 3.6;        // kph
}

void MissionGenerator::Init(){
    ProcessINI();
    ReadOSMFile(); 
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
    p_global_route1_pub.publish(global_route1_msg);
    p_global_route2_pub.publish(global_route2_msg);
    p_global_route3_pub.publish(global_route3_msg);
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

void MissionGenerator::ReadOSMFile(){
    // std::string pwd_dir(getenv("PWD"));
    // std::string map_dir("/resources/"+map_name+".osm");
    // std::string map_path = pwd_dir+map_dir;

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

    std::string osmPath = "/resources/Town03_competition.osm";

    std::string map_path = wsDir + osmPath;

    ErrorMessages errors;
    ROS_INFO_STREAM("Reading "<<map_path<<"...");
    lanelet_map = load(map_path, lanelet_utm_projector, &errors);
    assert(errors.empty());

    LineString3d left_boundary, right_boundary, lane_left, lane_center, lane_right;

    left_boundary = lanelet_map->lineStringLayer.get(LEFT_BOUNDARY_ID);
    for (auto point : left_boundary)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_left_boundary_vec.push_back(temp);
    }        
    // ROS_INFO_STREAM("m_lanelet_lane1_vec size : " << m_lanelet_lane1_vec.size());

    right_boundary = lanelet_map->lineStringLayer.get(RIGHT_BOUNDARY_ID);     
    for (auto point : right_boundary)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_right_boundary_vec.push_back(temp);
    }

    lane_left = lanelet_map->lineStringLayer.get(LANE_LEFT_ID);     
    for (auto point : lane_left)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lane_left_vec.push_back(temp);
    }

    lane_center = lanelet_map->lineStringLayer.get(LANE_CENTER_ID);     
    for (auto point : lane_center)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lane_center_vec.push_back(temp);
    }

    lane_right = lanelet_map->lineStringLayer.get(LANE_RIGHT_ID);     
    for (auto point : lane_right)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lane_right_vec.push_back(temp);
    }
 
    ROS_WARN_STREAM("Map Loaded!!!");

    m_waypoint_vec = m_lane_center_vec;

}

void MissionGenerator::MakeGlobalRoute()
{   
    geometry_msgs::PoseArray temp_global_route1;
    temp_global_route1.header.frame_id = "map";
    temp_global_route1.header.stamp = ros::Time::now();
    for(auto element : m_lane_left_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_global_route1.poses.push_back(point);
    }

    geometry_msgs::PoseArray temp_global_route2;
    temp_global_route2.header.frame_id = "map";
    temp_global_route2.header.stamp = ros::Time::now();
    for(auto element : m_lane_center_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_global_route2.poses.push_back(point);
    }

    geometry_msgs::PoseArray temp_global_route3;
    temp_global_route2.header.frame_id = "map";
    temp_global_route2.header.stamp = ros::Time::now();
    for(auto element : m_lane_right_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_global_route3.poses.push_back(point);
    }

    global_route1_msg = temp_global_route1;
    global_route2_msg = temp_global_route2;
    global_route3_msg = temp_global_route3;
}

void MissionGenerator::GenerateMission(){
    if(m_closest_id<180)
    {
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<230)
    { 
        m_mission = STATIC_OBSTACLE_1;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<240)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<252)
    { 
        m_mission = TRAFFIC_LIGHT;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<308)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<393)
    { 
        m_mission = ROTARY;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<425)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    } 
    else if(m_closest_id<440)
    { 
        m_mission = TRAFFIC_LIGHT;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<470)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<495)
    { 
        m_mission = DYNAMIC_OBSTACLE;
        mission_msg.data = m_mission;
    }             
    // else if(m_closest_id<1420)
    // { 
    //     m_mission = PARKING;
    //     mission_msg.data = m_mission;
    // }             
    else if(m_closest_id<555)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<717)
    { 
        m_mission = TUNNEL;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<735)
    { 
        m_mission = NORMAL_DRIVE;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<821)
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

    m_velocity = vs_velocity;  // kph
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

    visualization_msgs::Marker lane1;
    lane1.header.frame_id = "map";
    lane1.header.stamp = ros::Time::now();
    lane1.id = 1;
    lane1.action = visualization_msgs::Marker::ADD;
    lane1.type = visualization_msgs::Marker::LINE_STRIP;
    lane1.lifetime = ros::Duration();
    lane1.scale.x = 0.3;

    lane1.color.r = 1.;
    lane1.color.g = 1.;
    lane1.color.b = 1.;
    lane1.color.a = 1.;

    lane1.pose.orientation.x = 0.0;
    lane1.pose.orientation.y = 0.0;
    lane1.pose.orientation.z = 0.0;
    lane1.pose.orientation.w = 1.0;

    for(auto point : m_left_boundary_vec)
    {
        lane1.points.push_back(point);
    }

    lanes.markers.push_back(lane1);

    visualization_msgs::Marker lane2;
    lane2.header.frame_id = "map";
    lane2.header.stamp = ros::Time::now();
    lane2.id = 2;
    lane2.action = visualization_msgs::Marker::ADD;
    lane2.type = visualization_msgs::Marker::LINE_STRIP;
    lane2.lifetime = ros::Duration();
    lane2.scale.x = 0.3;

    lane2.color.r = 1.;
    lane2.color.g = 1.;
    lane2.color.b = 1.;
    lane2.color.a = 1.;

    lane2.pose.orientation.x = 0.0;
    lane2.pose.orientation.y = 0.0;
    lane2.pose.orientation.z = 0.0;
    lane2.pose.orientation.w = 1.0;

    for(auto point : m_right_boundary_vec)
    {
        lane2.points.push_back(point);
    }

    lanes.markers.push_back(lane2);

    visualization_msgs::Marker lane_left;
    lane_left.header.frame_id = "map";
    lane_left.header.stamp = ros::Time::now();
    lane_left.id = 3;
    lane_left.action = visualization_msgs::Marker::ADD;
    lane_left.type = visualization_msgs::Marker::LINE_LIST;
    lane_left.lifetime = ros::Duration();
    lane_left.scale.x = 0.3;

    lane_left.color.r = 1.;
    lane_left.color.g = 1.;
    lane_left.color.b = 0.;
    lane_left.color.a = 1.;

    lane_left.pose.orientation.x = 0.0;
    lane_left.pose.orientation.y = 0.0;
    lane_left.pose.orientation.z = 0.0;
    lane_left.pose.orientation.w = 1.0;

    for(auto point : m_lane_left_vec)
    {
        lane_left.points.push_back(point);
    }

    lanes.markers.push_back(lane_left);

    visualization_msgs::Marker lane_center;
    lane_center.header.frame_id = "map";
    lane_center.header.stamp = ros::Time::now();
    lane_center.id = 4;
    lane_center.action = visualization_msgs::Marker::ADD;
    lane_center.type = visualization_msgs::Marker::LINE_LIST;
    lane_center.lifetime = ros::Duration();
    lane_center.scale.x = 0.3;

    lane_center.color.r = 1.;
    lane_center.color.g = 1.;
    lane_center.color.b = 0.;
    lane_center.color.a = 1.;

    lane_center.pose.orientation.x = 0.0;
    lane_center.pose.orientation.y = 0.0;
    lane_center.pose.orientation.z = 0.0;
    lane_center.pose.orientation.w = 1.0;

    for(auto point : m_lane_center_vec)
    {
        lane_center.points.push_back(point);
    }

    lanes.markers.push_back(lane_center);

    visualization_msgs::Marker lane_right;
    lane_right.header.frame_id = "map";
    lane_right.header.stamp = ros::Time::now();
    lane_right.id = 5;
    lane_right.action = visualization_msgs::Marker::ADD;
    lane_right.type = visualization_msgs::Marker::LINE_LIST;
    lane_right.lifetime = ros::Duration();
    lane_right.scale.x = 0.3;

    lane_right.color.r = 1.;
    lane_right.color.g = 1.;
    lane_right.color.b = 0.;
    lane_right.color.a = 1.;

    lane_right.pose.orientation.x = 0.0;
    lane_right.pose.orientation.y = 0.0;
    lane_right.pose.orientation.z = 0.0;
    lane_right.pose.orientation.w = 1.0;

    for(auto point : m_lane_right_vec)
    {
        lane_right.points.push_back(point);
    }

    lanes.markers.push_back(lane_right);

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

    for(auto point : m_lane_left_vec)
    {
        lane_center_1_road.points.push_back(point);
    }

    lanes.markers.push_back(lane_center_1_road);

    visualization_msgs::Marker lane_center_2_road;
    lane_center_2_road.header.frame_id = "map";
    lane_center_2_road.header.stamp = ros::Time::now();
    lane_center_2_road.id = 7;
    lane_center_2_road.action = visualization_msgs::Marker::ADD;
    lane_center_2_road.type = visualization_msgs::Marker::LINE_STRIP;
    lane_center_2_road.lifetime = ros::Duration();
    lane_center_2_road.scale.x = 3.5;

    lane_center_2_road.color.r = 1.;
    lane_center_2_road.color.g = 1.;
    lane_center_2_road.color.b = 1.;
    lane_center_2_road.color.a = 0.5;

    lane_center_2_road.pose.orientation.x = 0.0;
    lane_center_2_road.pose.orientation.y = 0.0;
    lane_center_2_road.pose.orientation.z = 0.0;
    lane_center_2_road.pose.orientation.w = 1.0;

    for(auto point : m_lane_right_vec)
    {
        lane_center_2_road.points.push_back(point);
    }

    lanes.markers.push_back(lane_center_2_road);

    lane_marker_array_msg = lanes;    

    // visualization_msgs::Marker lane_center_1;
    // lane_center_1.header.frame_id = "map";
    // lane_center_1.header.stamp = ros::Time::now();
    // lane_center_1.id = 4;
    // lane_center_1.action = visualization_msgs::Marker::ADD;
    // lane_center_1.type = visualization_msgs::Marker::LINE_LIST;
    // lane_center_1.lifetime = ros::Duration();
    // lane_center_1.scale.x = 0.3;

    // lane_center_1.color.r = 1.;
    // lane_center_1.color.g = 1.;
    // lane_center_1.color.b = 0.;
    // lane_center_1.color.a = 1.;

    // lane_center_1.pose.orientation.x = 0.0;
    // lane_center_1.pose.orientation.y = 0.0;
    // lane_center_1.pose.orientation.z = 0.0;
    // lane_center_1.pose.orientation.w = 1.0;

    // for(auto point : m_waypoint_vec)
    // {
    //     lane_center_1.points.push_back(point);
    // }

    // lanes.markers.push_back(lane_center_1);

    // visualization_msgs::Marker lane_center_1_road;
    // lane_center_1_road.header.frame_id = "map";
    // lane_center_1_road.header.stamp = ros::Time::now();
    // lane_center_1_road.id = 6;
    // lane_center_1_road.action = visualization_msgs::Marker::ADD;
    // lane_center_1_road.type = visualization_msgs::Marker::LINE_STRIP;
    // lane_center_1_road.lifetime = ros::Duration();
    // lane_center_1_road.scale.x = 3.5;

    // lane_center_1_road.color.r = 1.;
    // lane_center_1_road.color.g = 1.;
    // lane_center_1_road.color.b = 1.;
    // lane_center_1_road.color.a = 0.5;

    // lane_center_1_road.pose.orientation.x = 0.0;
    // lane_center_1_road.pose.orientation.y = 0.0;
    // lane_center_1_road.pose.orientation.z = 0.0;
    // lane_center_1_road.pose.orientation.w = 1.0;

    // for(auto point : m_waypoint_vec)
    // {
    //     lane_center_1_road.points.push_back(point);
    // }

    // lanes.markers.push_back(lane_center_1_road);

    // lane_marker_array_msg = lanes;

    // MISSION VISUALIZATION
    jsk_rviz_plugins::OverlayText temp_rviz_mission_msg;
    if(m_mission == STATIC_OBSTACLE_1)
    {
        temp_rviz_mission_msg.text = "[STATIC_OBSTACLE_1]";
    }
    else if(m_mission == TRAFFIC_LIGHT)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT]";
    }
    else if(m_mission == ROTARY)
    {
        temp_rviz_mission_msg.text = "[ROTARY]";
    }
    else if(m_mission == DYNAMIC_OBSTACLE)
    {
        temp_rviz_mission_msg.text = "[DYNAMIC_OBSTACLE]";
    }
    else if(m_mission == PARKING)
    {
        temp_rviz_mission_msg.text = "[PARKING]";
    }
    else if(m_mission == TUNNEL)
    {
        temp_rviz_mission_msg.text = "[TUNNEL]";
    }    
    else if(m_mission == STATIC_OBSTACLE_3)
    {
        temp_rviz_mission_msg.text = "[STATIC_OBSTACLE_3]";
    }
    else
    {
        temp_rviz_mission_msg.text = "[NORMAL_DRIVE]";
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