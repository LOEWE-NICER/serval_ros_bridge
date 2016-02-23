#include "ros/ros.h"
#include "image_to_serval/image_to_serval.h"


int main(int argc, char **argv)
{

 // cv::namedWindow("Converted Image");

  ros::init(argc, argv, "image_to_serval");

  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");
  serval_ros_bridge::ImageToServal hd(nh_, pnh_);

  ros::spin();

  return 0;
}

