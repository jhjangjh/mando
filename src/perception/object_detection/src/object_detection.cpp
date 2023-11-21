#include "object_detection/object_detection.hpp"

Yolov5::Yolov5() {
    sub_boundingboxes = nh.subscribe("/yolov5/detections", 100, &Yolov5::BoundingBoxCB, this);
    sub_yolo_img = nh.subscribe("/yolov5/image_out", 1000, &Yolov5::YoloImgCB, this);   
    pub_red_light = nh.advertise<std_msgs::Bool>("/yolov5/traffic_light_signal", 100);
}

Yolov5::~Yolov5() {}

void Yolov5::YoloImgCB(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try
	{
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        origin_img = cv_ptr->image;             // 800x600
        roi_img = cv_ptr->image;
        // imshow("origin_img", origin_img);		
        
        findROI();

		waitKey(3);
	}
	catch(const cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
};

void Yolov5::BoundingBoxCB(const detection_msgs::BoundingBoxesConstPtr& msg) {
    std::vector<detection_msgs::BoundingBox> temp_boundingboxes;
    temp_boundingboxes = msg->bounding_boxes;

    // std::vector<detection_msgs::BoundingBox>::iterator itr;
    // for(itr = temp_boundingboxes.begin(); itr != temp_boundingboxes.end(); ++itr) {
    //     std::cout << *itr << "\n";
    // }
    
    findREDLight(temp_boundingboxes);
}

void Yolov5::findREDLight(std::vector<detection_msgs::BoundingBox> boundingboxes) {
    REDLIGHT_DETECTED = false;
    REDLIGHT_DETECTED_msg.data = false;

    detection_msgs::BoundingBox cur_boundingbox;
    std::vector<detection_msgs::BoundingBox>::iterator itr;
    for(itr = boundingboxes.begin(); itr != boundingboxes.end(); ++itr) {
        cur_boundingbox = *itr;
        
        std::string cur_class = cur_boundingbox.Class;
        double cur_prob = cur_boundingbox.probability;
        int cur_xmin = cur_boundingbox.xmin;
        int cur_ymin = cur_boundingbox.ymin;
        int cur_xmax = cur_boundingbox.xmax;
        int cur_ymax = cur_boundingbox.ymax;

        // if detected red sign is included in ROI, 'publish "true" to stop' to control node
        if (cur_class == "red" || cur_class == "yellow") {
            if ( (xmin_roi < cur_xmin) && (cur_xmax < xmax_roi) && (ymin_roi < cur_ymin) && (cur_ymax < ymax_roi) ) {
                REDLIGHT_DETECTED = true;
                REDLIGHT_DETECTED_msg.data = true;
            }
        }
    }

    if (REDLIGHT_DETECTED) {
        std::cout << "red light detected!!!!!\n";
    } 
    else {
        std::cout << "no red light....\n";
    }
}


void Yolov5::findROI() {
    xmin_roi = 300;
    ymin_roi = 150;
    w_roi = 200;
    h_roi = 120;
    xmax_roi = xmin_roi + w_roi;
    ymax_roi = ymin_roi + h_roi;

    roi_img = roi_img(Rect(xmin_roi, ymin_roi, w_roi, h_roi));
    // imshow("roi_img", roi_img);
}

void Yolov5::Publish() {
    pub_red_light.publish(REDLIGHT_DETECTED_msg);
}

void Yolov5::run() {
    Publish();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_detection");
    ros::Time::init();

    Yolov5 main_task;
    
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        main_task.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}