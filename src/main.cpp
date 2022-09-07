
#include "ros/ros.h"
#include "halnet_infer.h"



int main(int argc, char **argv){
    ros::init(argc, argv, "halnet_roscpp");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    image_transport::ImageTransport it(nh);

    HandDetection hd(it);
    
  
    ros::waitForShutdown();
    return 0;
}