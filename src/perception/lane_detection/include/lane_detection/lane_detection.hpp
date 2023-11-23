#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/PoseArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <std_msgs/Int32.h>
#include <iostream>
#include <vector>

using namespace cv;

class LaneDetection{
public:
    LaneDetection(ros::NodeHandle &nh_);
    ~LaneDetection();

    void ImageCallback(const sensor_msgs::ImageConstPtr &in_image_msg);
    void Pipeline();
    void Run();
    void Publish();

private:
    // Publisher

    // Subscriber
    ros::Subscriber sub_image;

    // Variables
    sensor_msgs::ImageConstPtr i_origin_img_state;
    Mat origin_img;
    Mat hsv_img;
    Mat yellow_mask;
    Mat gray_img;
    Mat binary_img;
    Mat roi_img;
};