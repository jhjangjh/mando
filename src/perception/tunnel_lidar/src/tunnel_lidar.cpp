#include <tunnel_lidar.hpp>

TunnelLidar::TunnelLidar(ros::NodeHandle &nh_){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/perception.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    p_roi_lidar_pub = nh_.advertise<pcl::PCLPointCloud2>("roi_lidar",100);
    
    s_lidar_sub = nh_.subscribe("/carla/ego_vehicle/lidar", 10, &TunnelLidar::LidarCallback, this);

    Init();
}

TunnelLidar::~TunnelLidar(){}

void TunnelLidar::LidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_lidar_msg){
    mutex_lidar.lock();
    pcl::fromROSMsg(*in_lidar_msg,*m_cloud_raw_ptr);
    mutex_lidar.unlock();
}

void TunnelLidar::Init(){
    ProcessINI();

    m_cloud_raw_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_voxelized_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_cluster_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void TunnelLidar::Run(){
    ProcessINI();
    VoxelizeData();
    SetROI();
    UpdateRviz();
    if(m_print_count++ % 10 == 0)
    {
        ROS_INFO_STREAM("Tunnel Lidar is running...");
    }

}

void TunnelLidar::Publish(){
    p_roi_lidar_pub.publish(m_output_roi);
}

void TunnelLidar::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        // v_ini_parser_.ParseConfig("tunnel_lidar", "xxx",
        //                             tunnel_lidar_params_.xxx);
        v_ini_parser_.ParseConfig("tunnel_lidar", "voxelsize",
                                    tunnel_lidar_params_.voxelsize);
        v_ini_parser_.ParseConfig("tunnel_lidar", "roi_distance",
                                    tunnel_lidar_params_.roi_distance);
        v_ini_parser_.ParseConfig("tunnel_lidar", "offset",
                                    tunnel_lidar_params_.offset);

        ROS_WARN("[Tunnel Lidar] Ini file is updated!\n");
    }
}

double TunnelLidar::GRTheta(double x, double y)
{
    double r;
    double theta;

    r = sqrt((x*x)+(y*y));
    theta = acos(x/r)*180/M_PI;
    return theta;
}

void TunnelLidar::VoxelizeData()
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    float voxelsize = tunnel_lidar_params_.voxelsize;

    voxel_filter.setInputCloud(m_cloud_raw_ptr);
    voxel_filter.setLeafSize(voxelsize,voxelsize,voxelsize);
    voxel_filter.filter(*m_voxelized_ptr);
    pcl::PCLPointCloud2 temp;
    pcl::toPCLPointCloud2(*m_voxelized_ptr,temp);
    pcl_conversions::fromPCL(temp,m_output_voxel);
}

void TunnelLidar::SetROI()
{

    float filter_limit = 0.0;

    // Create a PassThrough filter for x-axis
    pcl::PassThrough<pcl::PointXYZ> xfilter;
    xfilter.setInputCloud(m_voxelized_ptr); // Assuming m_voxelized_ptr is your input cloud
    xfilter.setFilterFieldName("x");
    xfilter.setFilterLimits(filter_limit,FLT_MAX);
    xfilter.filter(*m_voxelized_ptr); // Update the filtered points in m_voxelized_ptr

    // Create a PassThrough filter for y-axis
    pcl::PassThrough<pcl::PointXYZ> yfilter;
    yfilter.setInputCloud(m_voxelized_ptr); // Assuming m_voxelized_ptr is your input cloud
    yfilter.setFilterFieldName("y");
    yfilter.setFilterLimits(-FLT_MAX,filter_limit);
    yfilter.filter(*m_voxelized_ptr); // Update the filtered points in m_voxelized_ptr

    // Create a PassThrough filter for z-axis
    pcl::PassThrough<pcl::PointXYZ> zfilter;
    zfilter.setInputCloud(m_voxelized_ptr); // Assuming m_voxelized_ptr is your input cloud
    zfilter.setFilterFieldName("z");
    zfilter.setFilterLimits(0.1,1.);
    zfilter.filter(*m_voxelized_ptr); // Update the filtered points in m_voxelized_ptr

    pcl::toPCLPointCloud2(*m_voxelized_ptr, m_cloud_p);
    pcl_conversions::fromPCL(m_cloud_p, m_output_gr);
    m_output_gr.header.frame_id = "ego_vehicle/lidar";

    pcl::fromROSMsg(m_output_gr,m_laser_cloud_in);

 
    for(unsigned int j=0; j<m_laser_cloud_in.points.size(); j++)
    {

        if(GRTheta(m_laser_cloud_in.points[j].x , m_laser_cloud_in.points[j].y) < 30)
        {
            m_laser_cloud_in.points[j].x = -10.;
            m_laser_cloud_in.points[j].y = 0;
            m_laser_cloud_in.points[j].z = 0;
        }

        if(sqrt(pow(m_laser_cloud_in.points[j].x,2)+pow(m_laser_cloud_in.points[j].y,2)) > tunnel_lidar_params_.roi_distance)
        {
            m_laser_cloud_in.points[j].x = -10.;
            m_laser_cloud_in.points[j].y = 0;
            m_laser_cloud_in.points[j].z = 0;
        }

        if(m_laser_cloud_in.points[j].x < 0.1 && m_laser_cloud_in.points[j].y < 0.1 && m_laser_cloud_in.points[j].z < 0.1)
        {
            m_laser_cloud_in.points[j].x = -10.;
            m_laser_cloud_in.points[j].y = 0;
            m_laser_cloud_in.points[j].z = 0;
        }
        else
        {
            m_laser_cloud_in.points[j].y = m_laser_cloud_in.points[j].y + tunnel_lidar_params_.offset;
        }

    }

    if(m_laser_cloud_in.size() > 0)
    {
        m_cluster_ptr = m_laser_cloud_in.makeShared();
    }


    // Create a PassThrough filter for x-axis
    pcl::PassThrough<pcl::PointXYZ> remove_filter;
    remove_filter.setInputCloud(m_cluster_ptr); // Assuming m_voxelized_ptr is your input cloud
    remove_filter.setFilterFieldName("x");
    remove_filter.setFilterLimits(-5.,FLT_MAX);
    remove_filter.filter(*m_cluster_ptr); // Update the filtered points in m_voxelized_ptr


    pcl::PCLPointCloud2 temp;
    pcl::toPCLPointCloud2(*m_cluster_ptr,temp);
    pcl_conversions::fromPCL(temp,m_output_roi);

    m_output_roi.header.frame_id = "ego_vehicle/lidar";
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