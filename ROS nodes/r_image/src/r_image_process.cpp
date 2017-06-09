/**
 *      @file  r_image_process.cpp
 *      @brief  
 *
 * 		Image subscriber to get the camera image (Flea3), treat image and extract values to navigate
 *
 *      @author   Bruno Vieira - bruno.v@ua.pt
 *
 *   	@internal
 *     	Created  10-Mar-2017
 *     	Company  University of Aveiro
 *   	Copyright  Copyright (c) 2017, Live session user
 *
 * =====================================================================================
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace cv;

/**
 * @brief  Callback used to visualize and process the collected image form Flea3 camera
 * @param[in] 
 * @author B.Vieira
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {

        //boost::shared_ptr<CvImage const> original_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        //CvImagePtr toCvCopy(msg, const std::string& encoding = std::string());

        cv_bridge::CvImageConstPtr img_original = cv_bridge::toCvShare(msg, "mono8");
        cv::Mat img_proc;
        cv::imshow("view", img_original->image);


        cv::threshold(img_original->image, img_proc, 128, 255, 0);

        cv::imshow("view2", img_proc);
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



// ========================================================================================= //
// =======================================  MAIN  ========================================== //
// ========================================================================================= //

int main(int argc, char **argv) {
    ros::init(argc, argv, "r_image_proc");

    ros::NodeHandle nh;
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("RawImage", 1, imageCallback);

    ros::spin();
    cv::destroyWindow("view");


}
