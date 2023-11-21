#include <mission_generator.hpp>

GlobalPlanning::GlobalPlanning(ros::NodeHandle &nh_) : lanelet_utm_projector(Origin(lanelet_origin)){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/planning.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());
    
    nh_.getParam("map",map_name);

    p_global_route1_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route_1", 10);
    p_global_route2_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route_2", 10);
    p_rviz_lane_pub = nh_.advertise<visualization_msgs::MarkerArray>("/hmi/lane", 10);

    Init();
}

GlobalPlanning::~GlobalPlanning(){}

void GlobalPlanning::Init(){
    ProcessINI();
    ReadOsmFile();                      
    // ReadCSVFile();
}

void GlobalPlanning::Run(){
    ProcessINI();
    MakeGlobalRoute();
    UpdateRvizLane();
    ROS_INFO_STREAM("Global Planning is running...");

}

void GlobalPlanning::Publish(){
    p_global_route1_pub.publish(global_route1_msg);
    p_global_route2_pub.publish(global_route2_msg);
    p_rviz_lane_pub.publish(lane_marker_array_msg);
}

void GlobalPlanning::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        // v_ini_parser_.ParseConfig("global_planning", "xxx",
        //                             global_planning_params_.xxx);

        ROS_WARN("[Global Planning] Ini file is updated!\n");
    }
}

void GlobalPlanning::ReadOsmFile(){
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

    std::string osmPath = "/resources/"+map_name+".osm";

    std::string map_path = wsDir + osmPath;

    ErrorMessages errors;
    ROS_INFO_STREAM("Reading "<<map_path<<"...");
    lanelet_map = load(map_path, lanelet_utm_projector, &errors);
    assert(errors.empty());

    LineString3d lane1, lane2, lane3, lane_center_1, lane_center_2;

    lane1 = lanelet_map->lineStringLayer.get(LANE1_ID);
    for (auto point : lane1)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lanelet_lane1_vec.push_back(temp);
    }        
    // ROS_INFO_STREAM("m_lanelet_lane1_vec size : " << m_lanelet_lane1_vec.size());

    lane2 = lanelet_map->lineStringLayer.get(LANE2_ID);     
    for (auto point : lane2)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lanelet_lane2_vec.push_back(temp);
    }

    lane3 = lanelet_map->lineStringLayer.get(LANE3_ID);     
    for (auto point : lane3)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lanelet_lane3_vec.push_back(temp);
    }

    lane_center_1 = lanelet_map->lineStringLayer.get(LANE1_CENTER_ID);     
    for (auto point : lane_center_1)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lane1_vec.push_back(temp);
    }

    lane_center_2 = lanelet_map->lineStringLayer.get(LANE2_CENTER_ID);     
    for (auto point : lane_center_2)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        m_lane2_vec.push_back(temp);
    }
 

}

void GlobalPlanning::ReadCSVFile(){
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

    std::string osmPath_lane1 = "/resources/lane1.csv";
    std::string osmPath_lane2 = "/resources/lane2.csv";

    std::string map_path_lane1 = wsDir + osmPath_lane1;
    std::string map_path_lane2 = wsDir + osmPath_lane2;

    std::ifstream in_lane1(map_path_lane1);
    std::ifstream in_lane2(map_path_lane2);

    if (!in_lane1.is_open()) 
    {
      ROS_INFO("Lane 1 File not found");
    }

    if (!in_lane2.is_open()) 
    {
      ROS_INFO("Lane 2 File not found");
    }

    std::string s_1, s_2;

    while(in_lane1)
    {
        getline(in_lane1, s_1);
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
        m_lane1_vec.push_back(waypoint);

    }


    while(in_lane2)
    {
        getline(in_lane2, s_2);
        std::istringstream iss(s_2);
        std::vector<double> temp;

        std::vector<std::string> buf = transForm::split(s_2, ',');
        for (std::vector<std::string>::iterator itr = buf.begin(); itr != buf.end(); ++itr) 
        {
            temp.push_back(transForm::toNumber(*itr));
        }

        geometry_msgs::Point waypoint;
        waypoint.x = temp[0];
        waypoint.y = temp[1];
        m_lane2_vec.push_back(waypoint);
    }

}

void GlobalPlanning::MakeGlobalRoute()
{   
    geometry_msgs::PoseArray temp_global_route1;
    temp_global_route1.header.frame_id = "map";
    temp_global_route1.header.stamp = ros::Time::now();
    for(auto element : m_lane1_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_global_route1.poses.push_back(point);
    }

    geometry_msgs::PoseArray temp_global_route2;
    temp_global_route2.header.frame_id = "map";
    temp_global_route2.header.stamp = ros::Time::now();
    for(auto element : m_lane2_vec)
    {
        geometry_msgs::Pose point;
        point.position = element;
        temp_global_route2.poses.push_back(point);
    }

    global_route1_msg = temp_global_route1;
    global_route2_msg = temp_global_route2;
}

void GlobalPlanning::UpdateRvizLane()
{
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

    for(auto point : m_lanelet_lane1_vec)
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

    for(auto point : m_lanelet_lane2_vec)
    {
        lane2.points.push_back(point);
    }

    lanes.markers.push_back(lane2);

    visualization_msgs::Marker lane3;
    lane3.header.frame_id = "map";
    lane3.header.stamp = ros::Time::now();
    lane3.id = 3;
    lane3.action = visualization_msgs::Marker::ADD;
    lane3.type = visualization_msgs::Marker::LINE_STRIP;
    lane3.lifetime = ros::Duration();
    lane3.scale.x = 0.3;

    lane3.color.r = 1.;
    lane3.color.g = 1.;
    lane3.color.b = 1.;
    lane3.color.a = 1.;

    lane3.pose.orientation.x = 0.0;
    lane3.pose.orientation.y = 0.0;
    lane3.pose.orientation.z = 0.0;
    lane3.pose.orientation.w = 1.0;

    for(auto point : m_lanelet_lane3_vec)
    {
        lane3.points.push_back(point);
    }

    lanes.markers.push_back(lane3);

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

    for(auto point : m_lane1_vec)
    {
        lane_center_1.points.push_back(point);
    }

    lanes.markers.push_back(lane_center_1);

    visualization_msgs::Marker lane_center_2;
    lane_center_2.header.frame_id = "map";
    lane_center_2.header.stamp = ros::Time::now();
    lane_center_2.id = 5;
    lane_center_2.action = visualization_msgs::Marker::ADD;
    lane_center_2.type = visualization_msgs::Marker::LINE_LIST;
    lane_center_2.lifetime = ros::Duration();
    lane_center_2.scale.x = 0.3;

    lane_center_2.color.r = 1.;
    lane_center_2.color.g = 1.;
    lane_center_2.color.b = 0.;
    lane_center_2.color.a = 1.;

    lane_center_2.pose.orientation.x = 0.0;
    lane_center_2.pose.orientation.y = 0.0;
    lane_center_2.pose.orientation.z = 0.0;
    lane_center_2.pose.orientation.w = 1.0;

    for(auto point : m_lane2_vec)
    {
        lane_center_2.points.push_back(point);
    }

    lanes.markers.push_back(lane_center_2);

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

    for(auto point : m_lane1_vec)
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

    for(auto point : m_lane2_vec)
    {
        lane_center_2_road.points.push_back(point);
    }

    lanes.markers.push_back(lane_center_2_road);

    lane_marker_array_msg = lanes;
}

int main(int argc, char** argv)
{
    std::string node_name = "global_planning";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    GlobalPlanning global(nh);
    ros::Rate loop_rate(1);

    while(ros::ok())
    {   
        global.Run();
        global.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}