// #include "hand_detection_action_server.h"

// HandDetectionActionServer::HandDetectionActionServer(ImageCapture& ic, HandDetection& hd):
//     hand_detection_action_server_(nh_, "my_action", boost::bind(&HandDetectionActionServer::exec_action_cb, this, _1), false),
//     image_capture_(ic),
//     hand_detection_(hd)
// {
//     ROS_INFO_STREAM("ACTION SERVER constructor");
// }

// void HandDetectionActionServer::start_server(){
//     hand_detection_action_server_.start();
// }

// void HandDetectionActionServer::exec_action_cb(const user_defined_msgs::HandDetectionInferenceGoalConstPtr& goal){
//     ROS_INFO_STREAM("goal recieved!!!");
//     Point2D left_hand;
//     cv::Mat left_color, left_depth;
//     cv::Mat right_color, right_depth;
//     float delta_time;

//     image_capture_.prepare_images(left_color, left_depth, delta_time);
//     left_hand = hand_detection_.find_hand(left_color, left_depth, delta_time);

//     user_defined_msgs::HandDetectionInferenceResult res;
 
//     res.yl = left_hand.getRow();
//     res.xl = left_hand.getCol();
 
//     // ROS_WARN_STREAM(res.yl);
//     // ROS_WARN_STREAM(res.xl);
//     // hand_detection_action_server_.setSucceeded(res);

// }