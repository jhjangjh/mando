#include <vehicle_control.hpp>

VehicleControl::VehicleControl(ros::NodeHandle &nh_){
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    p_control_cmd_pub = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);

    s_odom_sub = nh_.subscribe("/carla/ego_vehicle/odometry", 1, &VehicleControl::OdomCallback, this);
    s_trajectory_sub = nh_.subscribe("/trajectory",1, &VehicleControl::TrajectoryCallback, this);

    Init();
}

VehicleControl::~VehicleControl(){}

void VehicleControl::OdomCallback(const nav_msgs::OdometryConstPtr &in_odom_msg){
    mutex_odom.lock();
    m_odom.pose = in_odom_msg->pose;
    m_odom.twist = in_odom_msg->twist;
    mutex_odom.unlock();
}

void VehicleControl::TrajectoryCallback(const kucudas_msgs::TrajectoryConstPtr &in_trajectory_msg){
    mutex_trajectory.lock();
    m_trajectory = *in_trajectory_msg;
    mutex_trajectory.unlock();
}

void VehicleControl::Init(){
    ProcessINI();  
}

void VehicleControl::Run(){
    ProcessINI();
    UpdateState();
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

void VehicleControl::Publish(){
    p_control_cmd_pub.publish(m_control_msg);
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

    m_velocity = sqrt(pow(m_odom.twist.twist.linear.x,2)+pow(m_odom.twist.twist.linear.y,2)) * 3.6;  // kph
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
    else
    {
        throttle = 0;  
        brake = 0; 
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
    m_target_lateral_error = GetCrossTrackError(target_point);
    double wheel_base = 1.2501934625522466 + 1.256300229233517;
    double lookahead_point_distance = sqrt(pow(target_point.x-m_ego_x,2)+pow(target_point.y-m_ego_y,2));
    steering_angle = -atan2(2 * wheel_base * m_target_lateral_error,pow(lookahead_point_distance,2));
    return steering_angle;
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