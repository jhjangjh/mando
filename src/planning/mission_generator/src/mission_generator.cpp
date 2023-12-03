#include <mission_generator.hpp>

MissionGenerator::MissionGenerator(ros::NodeHandle &nh_) : lanelet_utm_projector(Origin(lanelet_origin)){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/planning.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());
    
    p_global_route1_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route1", 10);
    p_global_route2_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route2", 10);
    p_loop_route_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/loop_route", 10);
    p_rviz_lane_pub = nh_.advertise<visualization_msgs::MarkerArray>("/hmi/lane", 10);
    p_mission_pub = nh_.advertise<std_msgs::Int8>("/adas/planning/mission", 10);
    p_rviz_mission_pub = nh_.advertise<jsk_rviz_plugins::OverlayText>("/hmi/mission", 10);

    // s_odom_sub = nh_.subscribe("/carla/ego_vehicle/odometry", 10, &MissionGenerator::OdomCallback, this);
    // s_gnss_sub = nh_.subscribe("/carla/ego_vehicle/gnss", 1, &MissionGenerator::GnssCallback, this);
    s_tf_sub = nh_.subscribe("/tf",10,&MissionGenerator::TfCallback,this);
    s_vs_sub = nh_.subscribe("/carla/ego_vehicle/vehicle_status",10,&MissionGenerator::VsCallback,this);
    s_ce_sub = nh_.subscribe("/carla/ego_vehicle/collision",10,&MissionGenerator::CeCallback,this);
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

void MissionGenerator::CeCallback(const carla_msgs::CarlaCollisionEventConstPtr &in_ce_msg){
    collision = true;
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
    p_loop_route_pub.publish(loop_route_msg);
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

    std::string osmPath = "/resources/Mando.osm";

    std::string map_path = wsDir + osmPath;

    ErrorMessages errors;
    ROS_INFO_STREAM("Reading "<<map_path<<"...");
    lanelet_map = load(map_path, lanelet_utm_projector, &errors);

    // assert(errors.empty());


    LineString3d left_boundary, right_boundary, lane_1, lane_loop, lane_2;

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

    lane_1 = lanelet_map->lineStringLayer.get(LANE_1_ID);     
    for (auto point : lane_1)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lane_1_vec.push_back(temp);
    }

    lane_2 = lanelet_map->lineStringLayer.get(LANE_2_ID);     
    for (auto point : lane_2)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lane_2_vec.push_back(temp);
    }

    lane_loop = lanelet_map->lineStringLayer.get(LANE_LOOP_ID);     
    for (auto point : lane_loop)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lane_loop_vec.push_back(temp);
    }

    ROS_WARN_STREAM("Map Loaded!!!");

    m_waypoint_vec = m_lane_1_vec;

}

void MissionGenerator::MakeGlobalRoute()
{   
    geometry_msgs::PoseArray temp_global_route1;
    temp_global_route1.header.frame_id = "map";
    temp_global_route1.header.stamp = ros::Time::now();
    for(auto element : m_lane_1_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_global_route1.poses.push_back(point);
    }

    geometry_msgs::PoseArray temp_global_route2;
    temp_global_route2.header.frame_id = "map";
    temp_global_route2.header.stamp = ros::Time::now();
    for(auto element : m_lane_2_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_global_route2.poses.push_back(point);
    }

    geometry_msgs::PoseArray temp_loop_route;
    temp_loop_route.header.frame_id = "map";
    temp_loop_route.header.stamp = ros::Time::now();
    for(auto element : m_lane_loop_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_loop_route.poses.push_back(point);
    }



    global_route1_msg = temp_global_route1;
    global_route2_msg = temp_global_route2;
    loop_route_msg = temp_loop_route;
}

void MissionGenerator::GenerateMission(){           
    
    ROS_WARN_STREAM("index: "<<m_closest_id);

    // if(m_closest_id<894)
    if(m_closest_id<910)
    {
        m_mission = NORMAL_DRIVE_01;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<935)
    { 
        m_mission = TRAFFIC_LIGHT_1;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<1155)
    { 
        m_mission = NORMAL_DRIVE_12;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<1190)
    { 
        m_mission = TRAFFIC_LIGHT_2;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<1715)
    { 
        m_mission = NORMAL_DRIVE_23;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<1742)
    { 
        m_mission = TRAFFIC_LIGHT_3;
        mission_msg.data = m_mission;
    }          
    else if(m_closest_id<1915)
    { 
        m_mission = NORMAL_DRIVE_34;
        mission_msg.data = m_mission;
    } 
    else if(m_closest_id<1935)
    { 
        m_mission = TRAFFIC_LIGHT_4;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<2040)
    { 
        m_mission = NORMAL_DRIVE_45;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<2053)
    { 
        m_mission = TRAFFIC_LIGHT_5;
        mission_msg.data = m_mission;
    } 
    else if(m_closest_id<2161)
    { 
        m_mission = NORMAL_DRIVE_56;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<2182)
    { 
        m_mission = TRAFFIC_LIGHT_6;
        mission_msg.data = m_mission;
    }      
    else if(m_closest_id<2280)
    { 
        m_mission = NORMAL_DRIVE_67;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<2301)
    { 
        m_mission = TRAFFIC_LIGHT_7;
        mission_msg.data = m_mission;
    }
    else if(m_closest_id<2355)
    { 
        m_mission = NORMAL_DRIVE_910;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<2789)
    { 
        m_mission = TUNNEL;
        mission_msg.data = m_mission;
    }      
    else if(m_closest_id<2928)
    { 
        m_mission = NORMAL_DRIVE_78;
        mission_msg.data = m_mission;
    }             
    else if(m_closest_id<2957)
    { 
        m_mission = TRAFFIC_LIGHT_8;
        mission_msg.data = m_mission;
    } 
    else
    { 
        m_mission = NORMAL_DRIVE_910;
        mission_msg.data = m_mission;
    }   


    if (250 < m_closest_id && m_closest_id <280){
        m_mission = STOP;
        mission_msg.data = m_mission;
    }

    if (1100 < m_closest_id && m_closest_id <1140){
        m_mission = STOP;
        mission_msg.data = m_mission;
    }

    if (2040 < m_closest_id && m_closest_id <2070){
        m_mission = STOP;
        mission_msg.data = m_mission;
    }

    if (2770 < m_closest_id && m_closest_id <2800){
        m_mission = STOP;
        mission_msg.data = m_mission;
    }

    if(collision){
        m_mission = FAIL;
        mission_msg.data = m_mission;
    }
              
}

// void MissionGenerator::GenerateMission(){
//     if(m_closest_id<894)
//     {
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }
//     else if(m_closest_id<923)
//     { 
//         m_mission = TRAFFIC_LIGHT_1;
//         mission_msg.data = m_mission;
//     }
//     else if(m_closest_id<1155)
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }
//     else if(m_closest_id<1190)
//     { 
//         m_mission = TRAFFIC_LIGHT_2;
//         mission_msg.data = m_mission;
//     }          
//     else if(m_closest_id<1715)
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }          
//     else if(m_closest_id<1742)
//     { 
//         m_mission = TRAFFIC_LIGHT_3;
//         mission_msg.data = m_mission;
//     }          
//     else if(m_closest_id<1900)
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     } 
//     else if(m_closest_id<1935)
//     { 
//         m_mission = TRAFFIC_LIGHT_4;
//         mission_msg.data = m_mission;
//     }             
//     else if(m_closest_id<2040)
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }             
//     else if(m_closest_id<2053)
//     { 
//         m_mission = TRAFFIC_LIGHT_5;
//         mission_msg.data = m_mission;
//     } 
//     else if(m_closest_id<2161)
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }             
//     else if(m_closest_id<2182)
//     { 
//         m_mission = TRAFFIC_LIGHT_6;
//         mission_msg.data = m_mission;
//     }      
//     else if(m_closest_id<2280)
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }             
//     else if(m_closest_id<2301)
//     { 
//         m_mission = TRAFFIC_LIGHT_7;
//         mission_msg.data = m_mission;
//     }
//     else if(m_closest_id<2355)
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }             
//     else if(m_closest_id<2789)
//     { 
//         m_mission = TUNNEL;
//         mission_msg.data = m_mission;
//     }      
//     else if(m_closest_id<2928)
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }             
//     else if(m_closest_id<2957)
//     { 
//         m_mission = TRAFFIC_LIGHT_8;
//         mission_msg.data = m_mission;
//     } 
//     else
//     { 
//         m_mission = NORMAL_DRIVE;
//         mission_msg.data = m_mission;
//     }             
// }

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

    visualization_msgs::Marker lane_1;
    lane_1.header.frame_id = "map";
    lane_1.header.stamp = ros::Time::now();
    lane_1.id = 3;
    lane_1.action = visualization_msgs::Marker::ADD;
    lane_1.type = visualization_msgs::Marker::LINE_LIST;
    lane_1.lifetime = ros::Duration();
    lane_1.scale.x = 0.3;

    lane_1.color.r = 1.;
    lane_1.color.g = 1.;
    lane_1.color.b = 0.;
    lane_1.color.a = 1.;

    lane_1.pose.orientation.x = 0.0;
    lane_1.pose.orientation.y = 0.0;
    lane_1.pose.orientation.z = 0.0;
    lane_1.pose.orientation.w = 1.0;

    for(auto point : m_lane_1_vec)
    {
        lane_1.points.push_back(point);
    }

    lanes.markers.push_back(lane_1);

    visualization_msgs::Marker lane_2;
    lane_2.header.frame_id = "map";
    lane_2.header.stamp = ros::Time::now();
    lane_2.id = 5;
    lane_2.action = visualization_msgs::Marker::ADD;
    lane_2.type = visualization_msgs::Marker::LINE_LIST;
    lane_2.lifetime = ros::Duration();
    lane_2.scale.x = 0.3;

    lane_2.color.r = 1.;
    lane_2.color.g = 1.;
    lane_2.color.b = 0.;
    lane_2.color.a = 1.;

    lane_2.pose.orientation.x = 0.0;
    lane_2.pose.orientation.y = 0.0;
    lane_2.pose.orientation.z = 0.0;
    lane_2.pose.orientation.w = 1.0;

    for(auto point : m_lane_2_vec)
    {
        lane_2.points.push_back(point);
    }

    lanes.markers.push_back(lane_2);

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

    for(auto point : m_lane_1_vec)
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

    for(auto point : m_lane_2_vec)
    {
        lane_center_2_road.points.push_back(point);
    }

    lanes.markers.push_back(lane_center_2_road);


    visualization_msgs::Marker lane_loop;
    lane_loop.header.frame_id = "map";
    lane_loop.header.stamp = ros::Time::now();
    lane_loop.id = 8;
    lane_loop.action = visualization_msgs::Marker::ADD;
    lane_loop.type = visualization_msgs::Marker::LINE_LIST;
    lane_loop.lifetime = ros::Duration();
    lane_loop.scale.x = 0.7;

    lane_loop.color.r = 1.;
    lane_loop.color.g = 1.;
    lane_loop.color.b = 0.;
    lane_loop.color.a = 1.;

    lane_loop.pose.orientation.x = 0.0;
    lane_loop.pose.orientation.y = 0.0;
    lane_loop.pose.orientation.z = 0.0;
    lane_loop.pose.orientation.w = 1.0;

    for(auto point : m_lane_loop_vec)
    {
        lane_loop.points.push_back(point);
    }

    lanes.markers.push_back(lane_loop);

    lane_marker_array_msg = lanes;    

    // MISSION VISUALIZATION
    jsk_rviz_plugins::OverlayText temp_rviz_mission_msg;
    temp_rviz_mission_msg.fg_color.r = 1.0f;
    temp_rviz_mission_msg.fg_color.g = 1.0f;
    temp_rviz_mission_msg.fg_color.b = 1.0f;
    temp_rviz_mission_msg.fg_color.a = 1.0f;

    if(m_mission == TRAFFIC_LIGHT_1)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT_1]";
    }
    else if(m_mission == TRAFFIC_LIGHT_2)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT_2]";
    }
    else if(m_mission == TRAFFIC_LIGHT_3)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT_3]";
    }
    else if(m_mission == TRAFFIC_LIGHT_4)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT_4]";
    }
    else if(m_mission == TRAFFIC_LIGHT_5)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT_5]";
    }
    else if(m_mission == TRAFFIC_LIGHT_6)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT_6]";
    }    
    else if(m_mission == TRAFFIC_LIGHT_7)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT_7]";
    }
    else if(m_mission == TRAFFIC_LIGHT_8)
    {
        temp_rviz_mission_msg.text = "[TRAFFIC_LIGHT_8]";
    }
    else if(m_mission == LOOP)
    {
        temp_rviz_mission_msg.text = "[LOOP]";
    }
    else if(m_mission == TUNNEL)
    {
        temp_rviz_mission_msg.text = "[TUNNEL]";
    }
    else if(m_mission == STOP)
    {
        temp_rviz_mission_msg.text = "[SUCCESS]";
        temp_rviz_mission_msg.fg_color.r = 0.0f;
        temp_rviz_mission_msg.fg_color.g = 1.0f;
        temp_rviz_mission_msg.fg_color.b = 0.0f;
        temp_rviz_mission_msg.fg_color.a = 1.0f;
    }
    else if(m_mission == FAIL)
    {
        temp_rviz_mission_msg.text = "[FAIL]";
        temp_rviz_mission_msg.fg_color.r = 1.0f;
        temp_rviz_mission_msg.fg_color.g = 0.0f;
        temp_rviz_mission_msg.fg_color.b = 0.0f;
        temp_rviz_mission_msg.fg_color.a = 1.0f;
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