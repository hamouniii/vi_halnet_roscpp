// #include "image_capture.h"

// ImageCapture::ImageCapture(image_transport::ImageTransport it)//:it_(nh_)
// {
//     // d435i_color_sub_ = nh_.subscribe("/d435i/color/image_raw", 1, &ImageCapture::d435i_color_cb, this);
//     // d435i_depth_sub_ = nh_.subscribe("/d435i/aligned_depth_to_color/image_raw", 1, &ImageCapture::d435i_depth_cb, this);

//     d435i_color_sub__ = it.subscribe("/d435i/color/image_raw", 1, &ImageCapture::d435i_color_cb, this);
//     d435i_depth_sub__ = it.subscribe("/d435i/aligned_depth_to_color/image_raw", 1, &ImageCapture::d435i_depth_cb, this);

// }

// void ImageCapture::d435i_color_cb(const sensor_msgs::ImageConstPtr &msg){
//     color_msg_ = msg;
// }

// void ImageCapture::d435i_depth_cb(const sensor_msgs::ImageConstPtr &msg){
//     depth_msg_ = msg;
// }

// // void ImageCapture::prepare_images(cv::Mat& color, cv::Mat& depth, float& dt){
// void ImageCapture::prepare_images(cv::Mat& color, cv::Mat& depth){
//     // double now = color_msg_->header.stamp.toNSec()*1e-9;
//     // if(last_time == -1){

//     //     dt = 0.1;
//     // }
//     // else{

//     //     dt = now - last_time; 
//     // }
//     // last_time = now;

//     cv::Mat color480x640 = cv_bridge::toCvShare(color_msg_, "bgr8")->image;     
//     cv::Mat depth480x640 = cv_bridge::toCvShare(depth_msg_, "16UC1")->image;        

//     cv::Mat color240x320(240, 320, CV_8UC3);
//     cv::Mat depth240x320(240, 320, CV_16UC1);

//     cv::resize(color480x640, color240x320, color240x320.size(), 0, 0, cv::INTER_NEAREST);
//     cv::resize(depth480x640, depth240x320, depth240x320.size(), 0, 0, cv::INTER_LINEAR);

//     color = color240x320;
//     depth = depth240x320; 
// }



// bool ImageCapture::ready(){
//     while(depth_msg_ == NULL || color_msg_ == NULL){}
//     return true;
// }