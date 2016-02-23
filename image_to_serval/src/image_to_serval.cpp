#include "image_to_serval/image_to_serval.h"

namespace serval_ros_bridge{

ImageToServal::ImageToServal(ros::NodeHandle& n,ros::NodeHandle& p_n){

    //ros::NodeHandle n;
    //ros::NodeHandle p_n("~");//private nh
    image_transport::ImageTransport it(n);
    image_transport::ImageTransport p_it(p_n);

    sub_ = it.subscribe("image", 1, &ImageToServal::imageCallback,this);

    //sub_mapping_ = n.subscribe("thermal/mapping",1, &ImageToServal::mappingCallback,this);

}

ImageToServal::~ImageToServal(){}

void ImageToServal::writeLatestImageToFile()
{
  if (last_img_.get()){
    //Read image with cvbridge
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(last_img_);
  }else{
    ROS_WARN_THROTTLE(10.0,"No latest image data, cannot write image!. This message is throttled.");
  }
}


void ImageToServal::imageCallback(const sensor_msgs::ImageConstPtr& img){

  last_img_ = img;

}




}

