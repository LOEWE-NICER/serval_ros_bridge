#include "image_to_serval/image_to_serval.h"

namespace serval_ros_bridge{

ImageToServal::ImageToServal(ros::NodeHandle& n,ros::NodeHandle& p_n){

    //ros::NodeHandle n;
    //ros::NodeHandle p_n("~");//private nh
    image_transport::ImageTransport it(n);
    image_transport::ImageTransport p_it(p_n);

    sub_ = it.subscribeCamera("image", 1, &ImageToServal::imageCallback,this);

    //sub_mapping_ = n.subscribe("thermal/mapping",1, &ImageToServal::mappingCallback,this);

}

ImageToServal::~ImageToServal(){}


void ImageToServal::imageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info){


     //Read image with cvbridge
     cv_bridge::CvImageConstPtr cv_ptr;
     cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
     cv::Mat img_filtered(cv_ptr->image);

}

}


/*
void ImageToServal::mappingCallback(const thermaleye_msgs::Mapping& mapping){
   mapping_ = mapping;
   mappingDefined_ = true;
   ROS_INFO("Mapping received");
}
*/

