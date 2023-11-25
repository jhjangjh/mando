#include <vehicle_control.hpp>

VehicleControl::VehicleControl(ros::NodeHandle &nh_) : lanelet_utm_projector(Origin(lanelet_origin)){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    p_control_cmd_pub = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);
    p_current_speed_pub = nh_.advertise<std_msgs::Float32>("/hmi/current_speed", 100);
    p_target_speed_pub = nh_.advertise<std_msgs::Float32>("/hmi/target_speed", 100);
    p_speed_error_pub = nh_.advertise<std_msgs::Float32>("/hmi/speed_error", 100);
    p_cross_track_error_pub = nh_.advertise<std_msgs::Float32>("/hmi/cross_track_error", 100);
    p_yaw_error_pub = nh_.advertise<std_msgs::Float32>("/hmi/yaw_error", 100);

    // s_odom_sub = nh_.subscribe("/carla/ego_vehicle/odometry", 1, &VehicleControl::OdomCallback, this);
    // s_gnss_sub = nh_.subscribe("/carla/ego_vehicle/gnss", 1, &VehicleControl::GnssCallback, this);
    s_tf_sub = nh_.subscribe("/tf",1000,&VehicleControl::TfCallback,this);
    s_vs_sub = nh_.subscribe("/carla/ego_vehicle/vehicle_status",10,&VehicleControl::VsCallback,this);
    s_trajectory_sub = nh_.subscribe("/trajectory",1, &VehicleControl::TrajectoryCallback, this);
    s_mission_sub = nh_.subscribe("/adas/planning/mission", 10, &VehicleControl::MissionCallback, this);
    s_tunnel_point_sub = nh_.subscribe("/tunnel_target_point", 10, &VehicleControl::TunnelPointCallback, this);

    Init();
}

VehicleControl::~VehicleControl(){}

void VehicleControl::MissionCallback(const std_msgs::Int8ConstPtr &in_mission_msg){
    m_mission = in_mission_msg->data;
}

void VehicleControl::TunnelPointCallback(const geometry_msgs::PointConstPtr &in_point_msg){
    m_tunnel_point = *in_point_msg;
}

// void VehicleControl::OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg){
//     // m_odom.pose = in_odom_msg->pose;
//     m_odom.twist = in_odom_msg->twist;
// }

// void VehicleControl::GnssCallback(const sensor_msgs::NavSatFixConstPtr &in_gnss_msg){
//     m_location_xyz= lanelet_utm_projector.forward(lanelet::GPSPoint{in_gnss_msg->latitude,in_gnss_msg->longitude,0});     // my gnss projection result
//     m_odom.pose.pose.position.x=m_location_xyz.x();
//     m_odom.pose.pose.position.x=m_location_xyz.y();
//     // m_gnss.latitude = in_gnss_msg->latitude;
//     // m_gnss.longitude = in_gnss_msg->longitude;
// }

void VehicleControl::TfCallback(const tf::tfMessage::ConstPtr& in_tf_msg) {
    // std::cout << in_tf_msg << std::endl;
    listener.lookupTransform("map","ego_vehicle",ros::Time(0),m_ego_vehicle_transform);
    m_odom.pose.pose.position.x = m_ego_vehicle_transform.getOrigin().x();
    m_odom.pose.pose.position.y = m_ego_vehicle_transform.getOrigin().y();
    m_odom.pose.pose.position.z = m_ego_vehicle_transform.getOrigin().z();
    m_odom.pose.pose.orientation.x = m_ego_vehicle_transform.getRotation().x();
    m_odom.pose.pose.orientation.y = m_ego_vehicle_transform.getRotation().y();
    m_odom.pose.pose.orientation.z = m_ego_vehicle_transform.getRotation().z();
    m_odom.pose.pose.orientation.w = m_ego_vehicle_transform.getRotation().w();
}

void VehicleControl::VsCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &in_vs_msg){
    vs_velocity = in_vs_msg->velocity * 3.6;        // kph
}

void VehicleControl::TrajectoryCallback(const kucudas_msgs::TrajectoryConstPtr &in_trajectory_msg){
    m_trajectory = *in_trajectory_msg;
}

void VehicleControl::Init(){
    ProcessINI();  
}

void VehicleControl::Run(){
    ProcessINI();
    UpdateState();

    // TUNNEL LIDAR ON MODE 
    if(longitudinal_control_params_.use_tunnel_lidar)
    {
        if(m_mission==TUNNEL)
        {
            double pid_error;
            pid_error = PIDControl(longitudinal_control_params_.tunnel_velocity);
            double steering_angle;
            m_target_point.x = m_tunnel_point.x;
            m_target_point.y = m_tunnel_point.y;
            steering_angle = PurePursuit(m_target_point);

            SetControlCmd(pid_error, steering_angle);
            UpdateControlState();

            if(m_print_count++ % 10 == 0)
            {
                ROS_WARN_STREAM(" [Tunnel] Vehicle Control is running...");
            }
        }
        else
        {
            if(m_trajectory.point.size()!=0)
            {
                m_closest_point = FindClosestPoint();
                double lookahead_distance = SetLookAheadDistance();
                m_target_point = FindTargetPoint(m_closest_point, lookahead_distance);

                double pid_error;
                if(longitudinal_control_params_.use_manual_desired_velocity)
                {
                    pid_error = PIDControl(longitudinal_control_params_.manual_desired_velocity);
                }
                else
                {
                    pid_error = PIDControl(m_target_point.speed);
                }

                double steering_angle;
                steering_angle = PurePursuit(m_target_point);

                SetControlCmd(pid_error, steering_angle);
                UpdateControlState();

                if(m_print_count++ % 10 == 0)
                {
                    ROS_INFO_STREAM("Vehicle Control is running...");
                }
            }
            else
            {
                ROS_WARN_STREAM("Waiting Trajectory...");
            }
        }
    }

    // TUNNEL LIDAR OFF MODE 
    else
    {
        if(m_trajectory.point.size()!=0)
        {
            m_closest_point = FindClosestPoint();
            double lookahead_distance = SetLookAheadDistance();
            ROS_WARN_STREAM("lookahead_distance : "<<lookahead_distance);
            m_target_point = FindTargetPoint(m_closest_point, lookahead_distance);

            double pid_error;
            if(longitudinal_control_params_.use_manual_desired_velocity)
            {
                pid_error = PIDControl(longitudinal_control_params_.manual_desired_velocity);
            }
            else
            {
                pid_error = PIDControl(m_target_point.speed);
            }

            double steering_angle;
            steering_angle = PurePursuit(m_target_point);

            SetControlCmd(pid_error, steering_angle);
            UpdateControlState();

            if(m_print_count++ % 10 == 0)
            {
                ROS_INFO_STREAM("Vehicle Control is running...");
            }
        }
        else
        {
            ROS_WARN_STREAM("Waiting Trajectory...");
        }
    }
    

    
}

void VehicleControl::Publish(){
    p_control_cmd_pub.publish(m_control_msg);
    p_current_speed_pub.publish(m_current_speed_msg);
    p_target_speed_pub.publish(m_target_speed_msg);
    p_speed_error_pub.publish(m_speed_error_msg);
    p_cross_track_error_pub.publish(m_cross_track_error_msg);
    p_yaw_error_pub.publish(m_yaw_error_msg);
}

void VehicleControl::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("longitudinal_control", "P_gain",
                                    longitudinal_control_params_.P_gain);
        v_ini_parser_.ParseConfig("longitudinal_control", "I_gain",
                                    longitudinal_control_params_.I_gain);
        v_ini_parser_.ParseConfig("longitudinal_control", "D_gain",
                                    longitudinal_control_params_.D_gain);
        v_ini_parser_.ParseConfig("longitudinal_control", "use_manual_desired_velocity",
                                    longitudinal_control_params_.use_manual_desired_velocity);
        v_ini_parser_.ParseConfig("longitudinal_control", "manual_desired_velocity",
                                    longitudinal_control_params_.manual_desired_velocity);
        v_ini_parser_.ParseConfig("longitudinal_control", "tunnel_velocity",
                                    longitudinal_control_params_.tunnel_velocity);
        v_ini_parser_.ParseConfig("longitudinal_control", "use_tunnel_lidar",
                                    longitudinal_control_params_.use_tunnel_lidar);
        v_ini_parser_.ParseConfig("lateral_control", "max_lookahead_distance",
                                    lateral_control_params_.max_lookahead_distance);
        v_ini_parser_.ParseConfig("lateral_control", "min_lookahead_distance",
                                    lateral_control_params_.min_lookahead_distance);
        ROS_WARN("[Control] Ini file is updated!\n");
    }
}

void VehicleControl::UpdateState()
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

double VehicleControl::PIDControl(double desired_velocity)
{
    // ROS_WARN_STREAM("desired_velocity : "<< desired_velocity);
    // ROS_WARN_STREAM("m_velocity : "<< m_velocity);

    pid_Kp = longitudinal_control_params_.P_gain;
    pid_Ki = longitudinal_control_params_.I_gain;
    pid_Kd = longitudinal_control_params_.D_gain;

    double pid_error = desired_velocity - m_velocity;
    pid_Pout = pid_Kp * pid_error;

    pid_integral +=  pid_error * pid_dt;
    pid_Iout = pid_Ki * pid_integral;
    
    pid_derivative = (pid_error - pid_pre_error) / pid_dt;
    pid_Dout = pid_Kd * pid_derivative;

    double pid_speed_error = pid_Pout + pid_Iout + pid_Dout;
    pid_pre_error = pid_error;
    
    return pid_speed_error;
}

double VehicleControl::PurePursuit(kucudas_msgs::TrajectoryPoint m_target_point)
{
    double steering_angle = GetSteeringAngle(m_target_point);
    return steering_angle;
    // ROS_INFO_STREAM("steering angle : "<<steering_angle);
}

void VehicleControl::SetControlCmd(double pid_speed_error, double steering_angle)
{
    double throttle = 0.;
    double brake = 0.;
    
    if(pid_speed_error>1)
    {
        throttle = 1;  
        brake = 0;      
    }
    else if(pid_speed_error>0)
    {
        throttle = pid_speed_error;
        brake = 0;
    }
    else if(pid_speed_error>-1)
    {
        throttle = 0;
        brake = -pid_speed_error;
    }
    else
    {
        throttle = 0;  
        brake = 0; 
    }

    if(m_target_point.speed<1)
    {
        throttle = 0;  
        brake = 1;         
    }

    m_control_msg.throttle = throttle;
    m_control_msg.brake = brake;
    m_control_msg.steer = steering_angle;
}

double VehicleControl::SetLookAheadDistance()
{
    double lookahead_distance;
    double min_distance = lateral_control_params_.min_lookahead_distance;                     // TBD
    double max_distance = lateral_control_params_.max_lookahead_distance;                    // TBD
    double min_velocity = 7.;

    lookahead_distance = ((max_distance-min_distance)/min_velocity*m_velocity) + min_distance;

    if(lookahead_distance < min_distance)
    {
        lookahead_distance = min_distance;
    }
    else if(lookahead_distance > max_distance)
    {
        lookahead_distance = max_distance;
    }

    return lookahead_distance;
}

kucudas_msgs::TrajectoryPoint VehicleControl::FindClosestPoint(){
    int closest_id = 0;
    double min_distance = HUGE_VAL;
    int index=0;

    // ROS_INFO_STREAM("size :"<<m_trajectory.point.size());
    for (auto waypoint : m_trajectory.point)
    {
        
        double distance = sqrt(pow(waypoint.x-m_ego_x,2)+pow(waypoint.y-m_ego_y,2));
        if(min_distance>distance)
        {
            closest_id = index;
            min_distance = distance;
        }
        index++;
    }
    // ROS_INFO_STREAM(closest_id);

    m_closest_id = closest_id;

    return m_trajectory.point.at(closest_id);

}

kucudas_msgs::TrajectoryPoint VehicleControl::FindTargetPoint(kucudas_msgs::TrajectoryPoint closest_point, double lookahead_distance){
    int target_id = 0;
    double point2point_distance;
    for(target_id = m_closest_id; target_id < m_trajectory.point.size()-1; target_id++)
    {
        kucudas_msgs::TrajectoryPoint searching_point = m_trajectory.point[target_id];
        point2point_distance = sqrt(pow(closest_point.x-searching_point.x,2)+pow(closest_point.y-searching_point.y,2));
        if(point2point_distance>lookahead_distance) {break;}
    }
    // ROS_INFO_STREAM("m_trajectory.point.size() : "<<m_trajectory.point.size());
    // ROS_INFO_STREAM("target_id : "<<target_id);
    return m_trajectory.point[target_id];
}

double VehicleControl::GetCrossTrackError(kucudas_msgs::TrajectoryPoint target_point)
{
    geometry_msgs::PointStamped lookahead_point_world;
    lookahead_point_world.header.frame_id = "map";
    lookahead_point_world.point.x = target_point.x;
    lookahead_point_world.point.y = target_point.y;
    lookahead_point_world.point.z = 0.0;
    geometry_msgs::PointStamped lookahead_point_ego;
    try{
        tfListenr.transformPoint("ego_vehicle",lookahead_point_world,lookahead_point_ego);
    }catch (tf::TransformException &ex) {
            // ROS_ERROR(ex.what());
    }
    double target_lateral_error = lookahead_point_ego.point.y;
    return target_lateral_error;
}

double VehicleControl::GetSteeringAngle(kucudas_msgs::TrajectoryPoint target_point)
{
    double steering_angle;
    double lookahead_point_distance = sqrt(pow(target_point.x-m_ego_x,2)+pow(target_point.y-m_ego_y,2));
    if(longitudinal_control_params_.use_tunnel_lidar)
    {
        if(m_mission == TUNNEL)
        {
            m_target_lateral_error = target_point.y;
            ROS_INFO_STREAM("target_point.x" << target_point.x);
            ROS_INFO_STREAM("target_point.y" << target_point.y);
            ROS_INFO_STREAM("m_target_lateral_error : "<<m_target_lateral_error);
            lookahead_point_distance = sqrt(pow(target_point.x,2)+pow(target_point.y,2));
        }
        else
        {
            m_target_lateral_error = GetCrossTrackError(target_point);
            lookahead_point_distance = sqrt(pow(target_point.x-m_ego_x,2)+pow(target_point.y-m_ego_y,2));
        }
    }
    else
    {
        m_target_lateral_error = GetCrossTrackError(target_point);
    }

    // ROS_INFO_STREAM("m_target_lateral_error : "<<m_target_lateral_error);
    
    double wheel_base = 1.2501934625522466 + 1.256300229233517;
    steering_angle = -atan2(2 * wheel_base * m_target_lateral_error,pow(lookahead_point_distance,2));

    ROS_INFO_STREAM("steering_angle : " << steering_angle);
    return steering_angle;
}

void VehicleControl::UpdateControlState()
{
    m_current_speed_msg.data = m_velocity;
    if(longitudinal_control_params_.use_manual_desired_velocity)
    {
        m_target_speed_msg.data = longitudinal_control_params_.manual_desired_velocity;    
    }
    else
    {
        m_target_speed_msg.data = m_target_point.speed;    
    }
    m_speed_error_msg.data = m_current_speed_msg.data - m_target_speed_msg.data;
    m_cross_track_error_msg.data = m_target_lateral_error;
    m_yaw_error_msg.data = m_target_point.yaw - m_yaw;
}

int main(int argc, char** argv)
{
    std::string node_name = "vehicle_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    VehicleControl control(nh);
    ros::Rate loop_rate(100);

    while(ros::ok())
    {   
        control.Run();
        control.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}