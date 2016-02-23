#ifndef _IMAGE_TO_SERVAL_H_
#define _IMAGE_TO_SERVAL_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


namespace serval_ros_bridge{

class ImageToServal{
public:
    ImageToServal(ros::NodeHandle& nh_,ros::NodeHandle& pnh_);
    ~ImageToServal();
private:
    void imageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& info);
    //void mappingCallback(const thermaleye_msgs::Mapping& mapping);
    image_transport::CameraSubscriber sub_;
    image_transport::CameraPublisher pub_detection_;
};

}

#endif
