#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// for yolov5
#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>

// for publishing red light detection
#include <std_msgs/Bool.h>

using namespace cv;

class Yolov5 {
public:
    Yolov5();
    ~Yolov5(); 

    void BoundingBoxCB(const detection_msgs::BoundingBoxesConstPtr& msg);
    void YoloImgCB(const sensor_msgs::ImageConstPtr& msg);

    // for img
    void findROI();

    // for finding front traffic light with red signal
    void findREDLight(std::vector<detection_msgs::BoundingBox> boundingboxes);

    // for publish
    void Publish();

    // for overall pipeline
    void run();

private:
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher pub_red_light;

    // Subscriber
    ros::Subscriber sub_boundingboxes;
    ros::Subscriber sub_yolo_img;           // not necessary // just for finding roi with bagfile !!!

    // for img
    Mat origin_img;
    Mat roi_img;
    int xmin_roi, ymin_roi, w_roi, h_roi, xmax_roi, ymax_roi;        // for roi

    // for finding front traffic light with red signal
    bool REDLIGHT_DETECTED = false;

    // for publishing red light detection
    std_msgs::Bool REDLIGHT_DETECTED_msg;
    
};