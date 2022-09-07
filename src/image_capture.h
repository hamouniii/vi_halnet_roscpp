// #ifndef Image_Capture_H_
// #define Image_Capture_H_


// #include <sensor_msgs/Image.h>
// #include <image_transport/image_transport.h>
// #include "utils.h"

// #include <cv_bridge/cv_bridge.h>
// // #include <sensor_msgs/image_encodings.h>

// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// // #include "halnet_infer.h"


// class ImageCapture{
//     public:
//         ImageCapture(image_transport::ImageTransport it);
//         bool ready();
//         // void prepare_images(cv::Mat& color, cv::Mat& depth, float& dt);
//         void prepare_images(cv::Mat& color, cv::Mat& depth);



//     private:

//         ros::NodeHandle nh_;
//         image_transport::ImageTransport it_(ros::NodeHandle nh);

//         image_transport::Subscriber d435i_color_sub__;
//         image_transport::Subscriber d435i_depth_sub__;

//         ros::Subscriber d435i_color_sub_;
//         ros::Subscriber d435i_depth_sub_;
        
//         double last_time = -1;
        

//         void d435i_color_cb(const sensor_msgs::ImageConstPtr &msg);
//         void d435i_depth_cb(const sensor_msgs::ImageConstPtr &msg);

//         sensor_msgs::ImageConstPtr color_msg_;
//         sensor_msgs::ImageConstPtr depth_msg_;  

//         Point2D ttest;

//         bool first_data = true;

// };


// #endif
