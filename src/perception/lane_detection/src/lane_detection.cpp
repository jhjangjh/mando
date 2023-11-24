#include <lane_detection/lane_detection.hpp>

LaneDetection::LaneDetection(ros::NodeHandle &nh_){
    sub_image = nh_.subscribe("/carla/ego_vehicle/rgb_front/image", 10, &LaneDetection::ImageCallback, this);
}

LaneDetection::~LaneDetection() {}

void LaneDetection::ImageCallback(const sensor_msgs::ImageConstPtr &in_image_msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
	{
        cv_ptr = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
        origin_img = cv_ptr->image;             
        imshow("original_img", origin_img);		
		waitKey(3);
	}
	catch(const cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    Pipeline();
}

void LaneDetection::Pipeline() {
    
    hsv_img = origin_img;
    cvtColor(hsv_img, hsv_img, COLOR_BGR2HSV);
    Scalar lower_yellow = cv::Scalar(25, 80, 80);
	Scalar upper_yellow = cv::Scalar(32, 255, 255);
    inRange(hsv_img, lower_yellow, upper_yellow, yellow_mask);

    imshow("yellow_mask", yellow_mask);

    roi_img = yellow_mask(Rect(0,300,400,280));
    imshow("roi", roi_img);
    

   /*
   cvtColor(origin_img, gray_img, COLOR_BGR2GRAY);
   imshow("b", gray_img);
   threshold(gray_img, binary_img, 135, 220, THRESH_BINARY);
   imshow("a", binary_img);
   */
}

void LaneDetection::Run() {
    Pipeline();
}

void LaneDetection::Publish() {

}


int main(int argc, char** argv)
{
    std::string node_name = "lane_detection_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    LaneDetection LD(nh);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {   
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    return 0;
}