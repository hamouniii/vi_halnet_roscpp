#ifndef HALNetInfer_H_
#define HALNetInfer_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/dnn.hpp>
#include "opencv2/video/tracking.hpp"

// #include "opencv2/core.hpp"


#include "utils.h"

#include <math.h> 

#include <chrono>




class HandDetection{
    public:
        HandDetection(image_transport::ImageTransport it);
        Point2D find_hand(cv::Mat color, cv::Mat depth);

        void start();

    private:

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_(ros::NodeHandle nh);

        image_transport::Subscriber d435i_color_sub__;
        image_transport::Subscriber d435i_depth_sub__;

        ros::Subscriber d435i_color_sub_;
        ros::Subscriber d435i_depth_sub_;
        
        void d435i_color_cb(const sensor_msgs::ImageConstPtr &msg);
        void d435i_depth_cb(const sensor_msgs::ImageConstPtr &msg);

        sensor_msgs::ImageConstPtr color_msg_;
        sensor_msgs::ImageConstPtr depth_msg_;

        cv::Mat color480x640;
        cv::Mat depth480x640;

        cv::dnn::Net halnet_model_;
        cv::dnn::Net jornet_model_;
        
        const std::string HALNET_CONFIG = "HALNet_deploy.prototxt";
        const std::string HALNET_WEIGHTS = "HALNet_weights.caffemodel";

        const std::string JORNET_CONFIG = "JORNet_deploy.prototxt";
        const std::string JORNET_WEIGHTS = "JORNet_weights.caffemodel";

        const float MAX_DEPTH = 1000.0;
        const int HALF_WINDOW_SIZE = 2;
        const float CROP_FACTOR = 1.15 * 23.5;

        const float CONFIDENCE_VAL_THRESHOLD = 0.15;
        const int CONFIDENCE_PIXEL_THRESHOLD = 20;
        const float DECAY_FACTOR = 0.98;

        const int ROOT_INDEX = 9;
        const int WRIST_INDEX = 0;

        float MES_VAR_VAL = 0.01;
        float PROC_VAR_VAL = 10.1;

        float ROOT_BINARY_THRESH = 0.05;
        float WRIST_BINARY_THRESH = 0.04;


        Point2D last_, confident_1_, confident_2_;


        void infer(cv::Mat color, cv::Mat depth, int& wrist_row, int& wrist_col);
        cv::Mat halnet_preprocess(cv::Mat color, cv::Mat depth);
        Point2D find_max_point(cv::Mat mat, double& max_val);
        float find_depth_mean(cv::Mat depth_mat, Point2D point, int ws);
        cv::Mat jornet_preprocess(cv::Mat color, cv::Mat depth, Point2D crop_around , int& rad_crop, Point2D& offset, cv::Mat& vis_crop);
                // cv::Mat jornet_preprocess(cv::Mat color, cv::Mat depth, int max_row, int max_col, int& rad_crop, int& offset_r, int& offset_c, cv::Mat& vis_crop);

        cv::Mat crop(cv::Mat mat, Point2D crop_around, int& rad_crop, Point2D& offset);
        void find_2d_loc();
        void halnet_infer();
        
        Point2D update_uncertain();
        Frame check_frame(Point2D this_frame, double max_val);

        Point2D find_suitable_point_for_hand_root(const cv::Mat& root);
        Point2D find_suitable_point_for_wrist(const cv::Mat& wrist_hm);

        Point2D find_contours(cv::Mat& hm, bool right_first);//return far right

        Point2D find_far_right_contour_center(const cv::Mat& root);
        Point2D find_far_left_contour_center(const cv::Mat& wrist_hm);
        void prepare_images(cv::Mat& color, cv::Mat& depth);




};





#endif
