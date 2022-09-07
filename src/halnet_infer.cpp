#include "halnet_infer.h"




HandDetection::HandDetection(image_transport::ImageTransport it):last_(Point2D(120, 160)), confident_1_(Point2D(120, 160)), confident_2_(Point2D(121, 161))    
{
    halnet_model_ = cv::dnn::readNet(HALNET_WEIGHTS, HALNET_CONFIG, "Caffe");
    jornet_model_ = cv::dnn::readNet(JORNET_WEIGHTS, JORNET_CONFIG, "Caffe");

    d435i_color_sub__ = it.subscribe("/d435i/color/image_raw", 1, &HandDetection::d435i_color_cb, this);
    d435i_depth_sub__ = it.subscribe("/d435i/aligned_depth_to_color/image_raw", 1, &HandDetection::d435i_depth_cb, this);


}



void HandDetection::prepare_images(cv::Mat& color, cv::Mat& depth){

    cv::Mat color480x640 = cv_bridge::toCvShare(color_msg_, "bgr8")->image;     
    cv::Mat depth480x640 = cv_bridge::toCvShare(depth_msg_, "16UC1")->image;        

    cv::Mat color240x320(240, 320, CV_8UC3);
    cv::Mat depth240x320(240, 320, CV_16UC1);

    cv::resize(color480x640, color240x320, color240x320.size(), 0, 0, cv::INTER_NEAREST);
    cv::resize(depth480x640, depth240x320, depth240x320.size(), 0, 0, cv::INTER_LINEAR);

    color = color240x320;
    depth = depth240x320; 
}


void HandDetection::d435i_color_cb(const sensor_msgs::ImageConstPtr &msg){
        ROS_WARN_STREAM("color");

    color_msg_ = msg;

    Point2D left_hand, right_hand;


    cv::Mat left_color, left_depth;
    cv::Mat right_color, right_depth;
    prepare_images(left_color, left_depth);
    left_hand = find_hand(left_color, left_depth);

    // prepare_images(right_color, right_depth)

}

void HandDetection::d435i_depth_cb(const sensor_msgs::ImageConstPtr &msg){
    ROS_WARN_STREAM("dep");

    depth_msg_ = msg;
    
}


Point2D HandDetection::update_uncertain(){
    Point2D p = this->confident_1_ - this->confident_2_;
    double norm_p = p.norm();

    Point2D q;
    q.setRow(round(DECAY_FACTOR * (p.getRow()/norm_p)));
    q.setCol(round(DECAY_FACTOR * (p.getCol()/norm_p)));

    Point2D ret;
    ret = this->last_ + q;

    return ret;
}

Frame HandDetection::check_frame(Point2D this_frame, double max_val){
    bool too_small = false, too_far = false;
    if(max_val < CONFIDENCE_VAL_THRESHOLD) {too_small = true;}
    
    if(abs(this_frame.getRow() - last_.getRow()) > CONFIDENCE_PIXEL_THRESHOLD || 
                            abs(this_frame.getCol() - last_.getCol()) > CONFIDENCE_PIXEL_THRESHOLD){
        too_far = true;
    }

    if(too_small && too_far) return UNCERTAIN;
    else return CONFIDENT;
}


Point2D HandDetection::find_max_point(cv::Mat mat, double& max_val){

    double minVal; 
    double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;

    cv::minMaxLoc(mat, &minVal, &maxVal, &minLoc, &maxLoc);
    
    Point2D ret(maxLoc.y, maxLoc.x);
    max_val = maxVal;
    
    return ret;
}


cv::Mat HandDetection::crop(cv::Mat mat, Point2D crop_around, int& rad_crop, Point2D& offset){

    int row = crop_around.getRow();
    int col = crop_around.getCol();

    if(rad_crop >119)rad_crop = 119;
    int height = mat.size[0];
    int width = mat.size[1];

    cv::Mat cropped_mat;

    int r = row-rad_crop;
    int c = col-rad_crop;

    if(r < 0)r=0;
    if(c < 0)c=0;

    if((r + 2*rad_crop) >= height) r = height - 2*rad_crop -1;
    if((c + 2*rad_crop) >= width) c = width - 2*rad_crop -1; 
    
    cv::Rect crop_region(c, r, 2*rad_crop, 2*rad_crop);
    cropped_mat = mat(crop_region);

    offset.setRow(r);
    offset.setCol(c);
    
    return cropped_mat;    
}


float HandDetection::find_depth_mean(cv::Mat depth_mat, Point2D point, int ws){
    int row = point.getRow();
    int col = point.getCol();
    float mean;

    //about ws: window size = 2ws + 1
    int height = depth_mat.size[0];
    int width = depth_mat.size[1];

    int rr, cc;
    rr = ((row - ws) < 0) ? 0 : row-ws;
    cc = ((col - ws) < 0) ? 0 : col-ws; 

    int valid_number = 0;
    float sum = 0;
    for(int r=rr; r <= (row+ws) && r < height; r++){
        for(int c = cc ; c <= (col+ws) && c < width; c++){
            float val = depth_mat.at<float>(r,c); 
            if(val < 1.0){
                sum += val;
                valid_number++;
            }
        }
    }
    if(sum != 0 && valid_number != 0)
        mean = sum/valid_number;
    else 
        mean = 0.0;
    return mean;
}



cv::Mat HandDetection::halnet_preprocess(cv::Mat color, cv::Mat depth){
    cv::threshold(depth, depth, MAX_DEPTH, MAX_DEPTH, CV_THRESH_TRUNC);
    depth.convertTo(depth, CV_32F, 1/MAX_DEPTH);

    // color.convertTo(color, CV_32F, 1.0/255.0);

    std::vector<cv::Mat> drgb_vec;
    std::vector<cv::Mat> channels(3);
    cv::split(color, channels);

    channels[0].convertTo(channels[0], CV_32F, 1.0/255.0);
    channels[1].convertTo(channels[1], CV_32F, 1.0/255.0);
    channels[2].convertTo(channels[2], CV_32F, 1.0/255.0);


    drgb_vec.push_back(depth);
    drgb_vec.push_back(channels[2]);
    drgb_vec.push_back(channels[0]);
    drgb_vec.push_back(channels[1]);

    cv::Mat drgb;
    cv::merge(drgb_vec, drgb);
    cv::Mat halnet_input;
    halnet_input = cv::dnn::blobFromImage(drgb);

    return halnet_input;
}
 

cv::Mat HandDetection::jornet_preprocess(cv::Mat color, cv::Mat depth, Point2D crop_around, int& rad_crop, Point2D& offset, cv::Mat& vis_cropped){
    
    cv::threshold(depth, depth, MAX_DEPTH, MAX_DEPTH, CV_THRESH_TRUNC);
    depth.convertTo(depth, CV_32F, 1/MAX_DEPTH);

    // color.convertTo(color, CV_32F, 1.0/255.0);
    
    float z = depth.at<float>(crop_around.getRow(), crop_around.getCol());

    float mean_depth = find_depth_mean(depth, crop_around, HALF_WINDOW_SIZE);
    if(mean_depth != 0 )
        rad_crop = (int)(CROP_FACTOR / mean_depth);
    else 
        rad_crop = 20;   

    cv::Mat cropped_color = crop(color, crop_around, rad_crop, offset);
    cv::Mat cropped_depth = crop(depth, crop_around, rad_crop, offset);

    cropped_depth = cropped_depth - z;

    vis_cropped = cropped_color.clone();

    cv::Mat color128x128(128, 128, CV_32F);
    cv::Mat depth128x128(128, 128, CV_32F);

    cv::resize(cropped_color, color128x128, color128x128.size(), 0, 0, cv::INTER_LINEAR);
    cv::resize(cropped_depth, depth128x128, depth128x128.size(), 0, 0, cv::INTER_NEAREST);


    std::vector<cv::Mat> drgb_vec;
    std::vector<cv::Mat> channels(3);
    cv::split(color128x128, channels);


    channels[0].convertTo(channels[0], CV_32F, 1.0/255.0);
    channels[1].convertTo(channels[1], CV_32F, 1.0/255.0);
    channels[2].convertTo(channels[2], CV_32F, 1.0/255.0);


    drgb_vec.push_back(depth128x128);
    drgb_vec.push_back(channels[2]);
    drgb_vec.push_back(channels[1]);
    drgb_vec.push_back(channels[0]);


    cv::Mat drgb;
    cv::merge(drgb_vec, drgb);


    cv::Mat jornet_input;
    jornet_input = cv::dnn::blobFromImage(drgb);

    return jornet_input;

}



Point2D HandDetection::find_far_right_contour_center(const cv::Mat& root){
    cv::Mat binary_root;

    cv::threshold(root, binary_root, ROOT_BINARY_THRESH, 1.0, cv::THRESH_BINARY);
    binary_root.convertTo(binary_root, CV_8U, 255);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_root, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);


    Point2D selected_point(0, 0);    
    for(std::vector<cv::Point>& contour : contours) {
        cv::Moments m = cv::moments(contour, true);
        Point2D tmp(m.m01/m.m00, m.m10/m.m00);
        if(selected_point.getCol() < tmp.getCol()){
            selected_point.setRow(tmp.getRow());
            selected_point.setCol(tmp.getCol());
        }
    }


    cv::circle(binary_root, cv::Point(selected_point.getCol(), selected_point.getRow()), 2, cv::Scalar(125, 125, 125), 2);
    cv::imshow("binary_root", binary_root);

    return selected_point;

}


Point2D HandDetection::find_suitable_point_for_hand_root(const cv::Mat& root){
    Point2D suitable_point;
    suitable_point = find_far_right_contour_center(root);

    return suitable_point;
}

Point2D HandDetection::find_far_left_contour_center(const cv::Mat& wrist_hm){
    cv::Mat binary_wrist;
    cv::threshold(wrist_hm, binary_wrist, WRIST_BINARY_THRESH, 1.0, cv::THRESH_BINARY);
    binary_wrist.convertTo(binary_wrist, CV_8U, 255);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_wrist, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        
    Point2D selected_point(wrist_hm.size[1], wrist_hm.size[0]);    
    for(std::vector<cv::Point>& contour : contours) {
        cv::Moments m = cv::moments(contour, true);
        Point2D tmp(m.m01/m.m00, m.m10/m.m00);
        // ROS_INFO_STREAM(tmp.to_string());
        if(selected_point.getCol() > tmp.getCol()){
            selected_point.setRow(tmp.getRow());
            selected_point.setCol(tmp.getCol());
        }
    }
    cv::circle(binary_wrist, cv::Point(selected_point.getCol(), selected_point.getRow()), 2, cv::Scalar(125, 125, 125), 2);
    cv::imshow("binary_wrist", binary_wrist);

    return selected_point;
}


Point2D HandDetection::find_suitable_point_for_wrist(const cv::Mat& wrist_hm){
    Point2D suitable_point;
    suitable_point = find_far_left_contour_center(wrist_hm);
    return suitable_point;
}


//dims 240 x 320
Point2D HandDetection::find_hand(cv::Mat color, cv::Mat depth){
    Point2D ret;
    Point2D root_max;
    Point2D offset;

    double root_max_value, wrist_max_value;
    int rad_crop;

    cv::Mat jornet_color = color.clone();
    cv::Mat jornet_depth = depth.clone();

    cv::Mat halnet_input = halnet_preprocess(color, depth);

    halnet_model_.setInput(halnet_input);
    cv::Mat halnet_output = halnet_model_.forward("upsample_heatmap_4f");

    cv::Mat root_hm_tmp(30, 40, CV_32F, halnet_output.ptr<float>(0, ROOT_INDEX));
    cv::Mat root(240, 320, CV_32F);//CV_8UC1
    cv::resize(root_hm_tmp, root, root.size(), 0, 0, cv::INTER_CUBIC);

    Point2D this_frame_point2d = find_suitable_point_for_hand_root(root);
        // Point2D this_frame_point2d = find_max_point(root, root_max_value);


    cv::Mat vis_crop;
    cv::Mat jornet_input = jornet_preprocess(jornet_color, jornet_depth, this_frame_point2d, rad_crop, offset, vis_crop);    
    jornet_model_.setInput(jornet_input);
    cv::Mat jornet_output = jornet_model_.forward("upsample_heatmap_final");



    //jornet post process  
    cv::Mat wrist_hm_tmp(32, 32, CV_32F, jornet_output.ptr<float>(0, WRIST_INDEX));
    cv::Mat wrist_2radcropx2radcrop(2*rad_crop, 2*rad_crop, CV_8UC1);//CV_8UC1
    cv::resize(wrist_hm_tmp, wrist_2radcropx2radcrop, wrist_2radcropx2radcrop.size(), 0, 0, cv::INTER_CUBIC);

    Point2D wrist_point2d;
    wrist_point2d = find_suitable_point_for_wrist(wrist_2radcropx2radcrop);
    // wrist_point2d = find_max_point(wrist_2radcropx2radcrop, wrist_max_value);
    
    Point2D unfiltered_wrist(wrist_point2d.getRow() + offset.getRow(), wrist_point2d.getCol() + offset.getCol());

    cv::Mat mes(2,1, CV_32F, {(float)unfiltered_wrist.getRow(), (float)unfiltered_wrist.getCol()});
    // cv::Mat tmp = kfw_.do_filter(mes, dt);
    // Point2D filtered_wrist((int) tmp.at<float>(0), (int) tmp.at<float>(2));


    // //visualisation
    root.convertTo(root, CV_8U, 255);
    cv::Mat color_root;
    cv::applyColorMap(root, color_root, cv::COLORMAP_HOT);

    
    // cv::circle(root, cv::Point(this_frame_point2d.getCol(), this_frame_point2d.getRow()), 2, cv::Scalar(0, 255, 0), 2);
    cv::circle(color, cv::Point(this_frame_point2d.getCol(), this_frame_point2d.getRow()), 2, cv::Scalar(0, 255, 0), 2);
    cv::circle(color, cv::Point(this_frame_point2d.getCol(), this_frame_point2d.getRow()), 2, cv::Scalar(255, 255, 255), 2);

    cv::circle(color_root, cv::Point(this_frame_point2d.getCol(), this_frame_point2d.getRow()), 2, cv::Scalar(0, 255, 0), 2);
    cv::circle(color, cv::Point(unfiltered_wrist.getCol(), unfiltered_wrist.getRow()), 2, cv::Scalar(255, 0, 0), 2);
    cv::circle(vis_crop, cv::Point(wrist_point2d.getCol(), wrist_point2d.getRow()), 2, cv::Scalar(255, 0, 0), 2);


    // cv::circle(root, cv::Point(filtered_root.getCol(), filtered_root.getRow()), 2, cv::Scalar(255, 255, 255), 2);
    // cv::circle(color, cv::Point(filtered_root.getCol(), filtered_root.getRow()), 2, cv::Scalar(255, 255, 255), 2);
    // cv::circle(color_root, cv::Point(filtered_root.getCol(), filtered_root.getRow()), 2, cv::Scalar(255, 255, 255), 2);


    // cv::imshow("root", root);
    cv::imshow("color_root", color_root);
    cv::imshow("original", color);
    cv::imshow("wrist_hm", wrist_2radcropx2radcrop);
    cv::imshow("cropped_hand", vis_crop);
    // cv::imshow("binary", bin);
    cv::waitKey(3);



    // ret.setRow(filtered_wrist.getRow());
    // ret.setCol(filtered_wrist.getCol());

    return ret;



}





































