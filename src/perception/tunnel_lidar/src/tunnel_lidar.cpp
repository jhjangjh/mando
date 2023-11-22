#include <tunnel_lidar.hpp>

TunnelLidar::TunnelLidar(ros::NodeHandle &nh_){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/perception.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());
    
    s_lidar_sub = nh_.subscribe("/carla/ego_vehicle/lidar", 10, &TunnelLidar::LidarCallback, this);

    Init();
}

TunnelLidar::~TunnelLidar(){}

void TunnelLidar::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg){
    mutex_lidar.lock();

    mutex_lidar.unlock();
}

void TunnelLidar::Init(){
    ProcessINI();
}

void TunnelLidar::Run(){
    ProcessINI();
    UpdateRviz();
    if(m_print_count++ % 10 == 0)
    {
        ROS_INFO_STREAM("Tunnel Lidar is running...");
    }

}

void TunnelLidar::Publish(){

}

void TunnelLidar::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        // v_ini_parser_.ParseConfig("tunnel_lidar", "xxx",
        //                             tunnel_lidar_params_.xxx);

        ROS_WARN("[Tunnel Lidar] Ini file is updated!\n");
    }
}

void TunnelLidar::UpdateRviz()
{

}

int main(int argc, char** argv)
{
    std::string node_name = "tunnel_lidar";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    TunnelLidar lidar(nh);
    ros::Rate loop_rate(30);

    while(ros::ok())
    {   
        lidar.Run();
        lidar.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}