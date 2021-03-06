#ifndef _IMAGE_TO_SERVAL_H_
#define _IMAGE_TO_SERVAL_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <topic_tools/shape_shifter.h>
#include <sensor_msgs/NavSatFix.h>
#include <exiftool_ros/exiftool_ros.h>


namespace serval_ros_bridge{

class ImageToServal{
public:
    ImageToServal(ros::NodeHandle& nh_,ros::NodeHandle& pnh_);
    ~ImageToServal();
private:
    void imageCallback(const sensor_msgs::ImageConstPtr& img);
    void trigger_subscriber(const boost::shared_ptr<const topic_tools::ShapeShifter> &message);

    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
    void navSatFixCallback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix);

    //void mappingCallback(const thermaleye_msgs::Mapping& mapping);

    void writeLatestImageToFile();
    image_transport::Subscriber sub_;
    ros::Subscriber trigger_subscriber_;

    ros::Subscriber pose_sub_;
    ros::Subscriber nav_sat_fix_sub_;

    ros::Publisher serval_update_pub_;

    sensor_msgs::ImageConstPtr last_img_;

    geometry_msgs::PoseStampedConstPtr pose_ptr_;
    sensor_msgs::NavSatFixConstPtr nav_sat_fix_ptr_;

    boost::shared_ptr<exiftool_ros::ExifTool> exiftool_;
    
    std::string p_save_folder_;
    std::string p_save_sub_folder_;
    std::string p_remote_save_folder_;
    std::string p_add_script_executable_name_;
    std::string p_scripts_folder_;
    std::string p_image_name_;
    int p_rotate_image_;
    std::string p_format_string_;

    std::stringstream filename_ss_;

    int rotate_flag_;

    ros::NodeHandle n_;
};

}

#endif
