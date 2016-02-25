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

namespace serval_ros_bridge{

ImageToServal::ImageToServal(ros::NodeHandle& n,ros::NodeHandle& p_n){

    p_n.param("save_folder", p_save_folder_, std::string("UNSET"));
    p_n.param("scripts_folder", p_script_folder_, std::string("UNSET"));
  
    image_transport::ImageTransport p_it(p_n);

    sub_ = p_it.subscribe("image", 1, &ImageToServal::imageCallback,this);

    trigger_subscriber_ = p_n.subscribe<topic_tools::ShapeShifter>("trigger_topic", 1, &ImageToServal::trigger_subscriber, this);

}

ImageToServal::~ImageToServal(){}

void ImageToServal::writeLatestImageToFile()
{
  if (last_img_.get()){
    //Read image with cvbridge
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(last_img_);

    //boost::filesystem::path dir(p_save_folder_ + ros::Time::now().toBoost());

    boost::filesystem::path dir(p_save_folder_ + "/serval_test");

    boost::filesystem::create_directory(dir);

    std::string full_file_path_and_name = dir.generic_string() + "/test.jpg";

    cv::imwrite(full_file_path_and_name, cv_ptr->image);

    ROS_INFO("Wrote image to %s", full_file_path_and_name.c_str());
    
    
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




}

