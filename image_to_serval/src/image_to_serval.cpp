//=================================================================================================
// Copyright (c) 2015, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "image_to_serval/image_to_serval.h"
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>




namespace serval_ros_bridge{

ImageToServal::ImageToServal(ros::NodeHandle& n,ros::NodeHandle& p_n){  

    rotate_flag_ = -1;

    p_n.param("save_folder", p_save_folder_, std::string("UNSET"));
    p_n.param("save_sub_folder", p_save_sub_folder_, std::string(""));
    p_n.param("remote_save_folder", p_remote_save_folder_, std::string("/home/pi/"));
    p_n.param("scripts_folder", p_scripts_folder_, std::string("UNSET"));
    p_n.param("add_script_executable_name", p_add_script_executable_name_, std::string("/s_addfolder "));    
    p_n.param("image_name", p_image_name_, std::string("default_image.jpg"));
    p_n.param("rotate_image_deg", p_rotate_image_, 0);
    p_n.param("format_string", p_format_string_, std::string("%Y-%m-%d_%H-%M-%S%F%Q"));


    if (p_rotate_image_ != 0){
      if (p_rotate_image_ == 90){
        rotate_flag_ = cv::ROTATE_90_COUNTERCLOCKWISE;
      }else if (p_rotate_image_ == -90){
        rotate_flag_ = cv::ROTATE_90_CLOCKWISE;
      }else {
        ROS_ERROR("Rotation of %d degree not supported, not rotating images!", p_rotate_image_);
      }
    }

    boost::posix_time::time_facet*  facet = new boost::posix_time::time_facet(p_format_string_.c_str());
    filename_ss_.imbue(std::locale(filename_ss_.getloc(), facet));

    //const boost::posix_time::ptime boost_time = ros::Time::now().toBoost();
    //filename_ss_ << boost_time;
    //ROS_INFO("Debug %s", filename_ss_.str().c_str());

    pose_sub_ = n_.subscribe("robot_pose", 1, &ImageToServal::poseCallback, this);
    nav_sat_fix_sub_ = n_.subscribe("/gps_calibration/gps/fix", 1, &ImageToServal::navSatFixCallback, this);

    serval_update_pub_ = n_.advertise<std_msgs::String>("/serval_update", 10, false);
  
    image_transport::ImageTransport p_it(p_n);

    sub_ = p_it.subscribe("image", 1, &ImageToServal::imageCallback,this);

    trigger_subscriber_ = p_n.subscribe<topic_tools::ShapeShifter>("trigger_topic", 1, &ImageToServal::trigger_subscriber, this);

    exiftool_.reset(new exiftool_ros::ExifTool());

}

ImageToServal::~ImageToServal(){}

void ImageToServal::writeLatestImageToFile()
{
  if (last_img_.get()){

    filename_ss_.str("");

    //Read image with cvbridge
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(last_img_);

    //@TODO: This probably is not platform independent
    const boost::posix_time::ptime boost_time = ros::Time::now().toBoost();



    filename_ss_  << p_save_sub_folder_;

    ROS_DEBUG_STREAM(filename_ss_.str());

    // Below creates the subfolder. Is non-op after it has been created once
    boost::filesystem::path dir(p_save_folder_ + "/" + filename_ss_.str());
    ROS_DEBUG_STREAM(dir.generic_string());

    boost::system::error_code error_code;
    if (!boost::filesystem::create_directory(dir, error_code)){
      // @TODO: Find a way to do this without ugly String comparison
      if (error_code.message() != "Success"){
        ROS_ERROR("Could not create folder %s, error_code message: %s. Aborting writing image.", dir.generic_string().c_str(), error_code.message().c_str());
        return;
      }
    }

    // Below creates time-based subfolder.
    filename_ss_  << "/" << boost_time;
    ROS_DEBUG_STREAM(filename_ss_.str());
    dir = boost::filesystem::path(p_save_folder_ + "/" + filename_ss_.str());
    ROS_DEBUG_STREAM(dir.generic_string());

    if (!boost::filesystem::create_directory(dir, error_code)){
      // @TODO: Find a way to do this without ugly String comparison
      if (error_code.message() != "Success"){
        ROS_ERROR("Could not create folder %s, error_code message: %s. Aborting writing image.", dir.generic_string().c_str(), error_code.message().c_str());
        return;
      }
    }

    std::string full_file_path_and_name = dir.generic_string() + "/" + p_image_name_;
    std::string remote_full_file_path_and_name = p_remote_save_folder_ + "/" + filename_ss_.str();

    if (rotate_flag_!= -1){
      cv::Mat tmp;
      cv::rotate(cv_ptr->image, tmp, rotate_flag_);
      cv::imwrite(full_file_path_and_name, tmp);
    }else{
      cv::imwrite(full_file_path_and_name, cv_ptr->image);
    }

    ROS_INFO("Wrote image to %s", full_file_path_and_name.c_str());

    exiftool_->writeGpsData(full_file_path_and_name, nav_sat_fix_ptr_);

    /*
    if (exiftool_available_){
      if (nav_sat_fix_ptr_){

        std::stringstream sys_command;
        // See https://www.sno.phy.queensu.ca/~phil/exiftool/faq.html#Q14
        // http://www.awaresystems.be/imaging/tiff/tifftags/privateifd/gps.html
        sys_command << "exiftool -exif:gpslatitude=" << nav_sat_fix_ptr_->latitude << " -exif:gpslatituderef=N -exif:gpslongitude="
                    << nav_sat_fix_ptr_->longitude << " -exif:gpslongituderef=E  -exif:gpsaltitude="
                    << nav_sat_fix_ptr_->altitude <<  " -exif:gpsaltituderef=above " << full_file_path_and_name.c_str();

        if (system(sys_command.str().c_str()) < 0){
          ROS_ERROR("Failed to use system call: %s", sys_command.str().c_str());
        }else{
          ROS_INFO("Successfully used system call: %s", sys_command.str().c_str());
        }
      }else{
        ROS_WARN("No navsatfix available!");
      }
    }else{
      ROS_WARN("Not adding geotag information as exiftool is not available!");
    }
    */

    std_msgs::String serval_update_str;
    std::stringstream serval_update_ss;

    serval_update_ss << "command=CREATE_FILE;path=" << remote_full_file_path_and_name << ";filename=" << p_image_name_ << ";timestamp=" << last_img_->header.stamp.toSec(); // << ";map_resolution=" << map->info.resolution << ";map_origin_pos_x=" << map->info.origin.position.x << ";map_origin_pos_y=" << map->info.origin.position.y;

    if (pose_ptr_.get()){
      serval_update_ss << ";pose_x=" << pose_ptr_->pose.position.x << ";pose_y=" << pose_ptr_->pose.position.y << ";pose_z=" << pose_ptr_->pose.position.z;
    }else{
      ROS_WARN("No pose data available, not adding to Serval metadata!");
    }
    serval_update_str.data = serval_update_ss.str();

    serval_update_pub_.publish(serval_update_str);
    
  }else{
    ROS_WARN_THROTTLE(10.0,"No latest image data, cannot write image!. This message is throttled.");
  }
}

void ImageToServal::trigger_subscriber(const boost::shared_ptr<const topic_tools::ShapeShifter>& message)
{
  this->writeLatestImageToFile();
}


void ImageToServal::imageCallback(const sensor_msgs::ImageConstPtr& img){
  last_img_ = img;
}

//We assume the robot position is available as a PoseStamped here (querying tf would be the more general option)
void ImageToServal::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
{
  pose_ptr_ = pose;
}


void ImageToServal::navSatFixCallback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix)
{
  nav_sat_fix_ptr_ = nav_sat_fix;
}



}

