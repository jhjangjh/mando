#include <mission_generator.hpp>

MissionGenerator::MissionGenerator(ros::NodeHandle &nh_){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/planning.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());
    
    p_global_route_pub = nh_.advertise<geometry_msgs::PoseArray>("/adas/planning/global_route", 10);
    p_rviz_lane_pub = nh_.advertise<visualization_msgs::MarkerArray>("/hmi/lane", 10);

    Init();
}

MissionGenerator::~MissionGenerator(){}

void MissionGenerator::Init(){
    ProcessINI();
    ReadCSVFile();
}

void MissionGenerator::Run(){
    ProcessINI();
    MakeGlobalRoute();
    UpdateRvizLane();
    ROS_INFO_STREAM("Mission Generator is running...");

}

void MissionGenerator::Publish(){
    p_global_route_pub.publish(global_route_msg);
    p_rviz_lane_pub.publish(lane_marker_array_msg);
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

    ROS_INFO_STREAM("Map loaded");

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

void MissionGenerator::UpdateRvizLane()
{
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
}

int main(int argc, char** argv)
{
    std::string node_name = "mission_generator";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    MissionGenerator mission(nh);
    ros::Rate loop_rate(1);

    while(ros::ok())
    {   
        mission.Run();
        mission.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}