#ifndef _IMAGE_TO_SERVAL_H_
#define _IMAGE_TO_SERVAL_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <topic_tools/shape_shifter.h>


namespace serval_ros_bridge{

class ImageToServal{
public:
    ImageToServal(ros::NodeHandle& nh_,ros::NodeHandle& pnh_);
    ~ImageToServal();
private:
    void imageCallback(const sensor_msgs::ImageConstPtr& img);
    void trigger_subscriber(const boost::shared_ptr<const topic_tools::ShapeShifter> &message);

    //void mappingCallback(const thermaleye_msgs::Mapping& mapping);

    void writeLatestImageToFile();
    image_transport::Subscriber sub_;
    ros::Subscriber trigger_subscriber_;

    sensor_msgs::ImageConstPtr last_img_;
    
    std::string p_save_folder_;
    std::string p_script_folder_;
};

}

#endif
