//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
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

#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Geometry>

#include <hector_map_tools/HectorMapTools.h>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <opencv2/opencv.hpp>

using namespace std;

/**
 * @brief This node provides occupancy grid maps as images via image_transport, so the transmission consumes less bandwidth.
 * The provided code is a incomplete proof of concept.
 */
class MapAsImageProvider
{
public:
  MapAsImageProvider()
    : pn_("~")
  {
    pn_.param("save_folder", p_save_folder_, std::string("UNSET"));
    pn_.param("scripts_folder", p_scripts_folder_, std::string("UNSET"));
    pn_.param("add_script_executable_name", p_add_script_executable_name_, std::string("/s_addfolder "));
    pn_.param("agent_name", p_agent_name_, std::string("ugv001"));
    pn_.param("map_name", p_map_name_, std::string("default_image.jpg"));
    pn_.param("format_string", p_format_string_, std::string("%Y-%m-%d_%H-%M-%S%F%Q"));

    boost::posix_time::time_facet*  facet = new boost::posix_time::time_facet(p_format_string_.c_str());
    filename_ss_.imbue(std::locale(filename_ss_.getloc(), facet));

    latest_update_pose_.pose.position.x = std::numeric_limits<double>::max();
    latest_update_pose_.pose.position.y = std::numeric_limits<double>::max();
    latest_update_time_ = ros::Time(0);

    image_transport_ = new image_transport::ImageTransport(pn_);
    image_transport_publisher_full_ = image_transport_->advertise("map_image/full", 1);
    //image_transport_publisher_tile_ = image_transport_->advertise("map_image/tile", 1);

    pose_sub_ = n_.subscribe("robot_pose", 1, &MapAsImageProvider::poseCallback, this);
    map_sub_ = n_.subscribe("map", 1, &MapAsImageProvider::mapCallback, this);

    //Which frame_id makes sense?
    cv_img_full_.header.frame_id = "map_image";
    cv_img_full_.encoding = sensor_msgs::image_encodings::MONO8;

    //cv_img_tile_.header.frame_id = "map_image";
    //cv_img_tile_.encoding = sensor_msgs::image_encodings::MONO8;

    //Fixed cell width for tile based image, use dynamic_reconfigure for this later
    p_size_tiled_map_image_x_ = 64;
    p_size_tiled_map_image_y_ = 64;

    ROS_INFO("Map to Image node started.");
  }

  ~MapAsImageProvider()
  {
    delete image_transport_;
  }

  //We assume the robot position is available as a PoseStamped here (querying tf would be the more general option)
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
  {
    pose_ptr_ = pose;
  }

  //The map->image conversion runs every time a new map is received at the moment
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
  {
    if (!pose_ptr_)
      return;

    double diff_x = pose_ptr_->pose.position.x - latest_update_pose_.pose.position.x;
    double diff_y = pose_ptr_->pose.position.y - latest_update_pose_.pose.position.y;

    ros::Time now = ros::Time::now();

    double update_dist_threshold = 1.0;
    double update_time_threshold_seconds = 30.0;

    double time_since_last_update_seconds = (now - latest_update_time_).toSec();

    ROS_DEBUG ("Time %f", time_since_last_update_seconds);

    // Abort if we have not travelled further than threshold
    if (((diff_x * diff_x + diff_y * diff_y) < update_dist_threshold) && !(time_since_last_update_seconds > update_time_threshold_seconds) )
        return;

    int size_x = map->info.width;
    int size_y = map->info.height;

    if ((size_x < 3) || (size_y < 3) ){
      ROS_WARN("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
      return;
    }


    filename_ss_.str("");


    //@TODO: This probably is not platform independent
    const boost::posix_time::ptime boost_time = ros::Time::now().toBoost();


    //filename_ss_ << p_save_folder_ << "/" << boost_time;

    filename_ss_ << p_save_folder_ << "/" << p_agent_name_;


    boost::filesystem::path dir(filename_ss_.str());

    boost::filesystem::create_directory(dir);

    std::string full_file_path_and_name = dir.generic_string() + "/" + p_map_name_;


    // Only if someone is subscribed to it, do work and publish full map image
    //if (image_transport_publisher_full_.getNumSubscribers() > 0){
      cv::Mat* map_mat  = &cv_img_full_.image;

      // resize cv image if it doesn't have the same dimensions as the map
      //if ( (map_mat->rows != size_y) && (map_mat->cols != size_x)){
        *map_mat = cv::Mat(size_y, size_x, CV_8U);
      //}

      const std::vector<int8_t>& map_data (map->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      //int size_y_rev = size_y-1;

      /*
      for (int y = size_y_rev; y >= 0; --y){

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x){

          int idx = idx_img_y + x;

          switch (map_data[idx_map_y + x])
          {
          case -1:
            map_mat_data_p[idx] = 127;
            break;

          case 0:
            map_mat_data_p[idx] = 255;
            break;

          case 100:
            map_mat_data_p[idx] = 0;
            break;
          }
        }
      }
      */

      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] == 0) { //occ [0,0.1)
            map_mat_data_p[i] = 255;
          } else if (map->data[i] == +100) { //occ (0.65,1]
            map_mat_data_p[i] = 0;
          } else { //occ [0.1,0.65]
            map_mat_data_p[i] = 127;
          }
        }
      }

      if (image_transport_publisher_full_.getNumSubscribers() > 0){
        image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
      }

      latest_update_pose_ = *pose_ptr_;
      latest_update_time_ = now;

      std::string mapimagedatafile = full_file_path_and_name + ".png";

      ROS_INFO("Writing map occupancy data to %s", mapimagedatafile.c_str());
      cv::imwrite(mapimagedatafile, cv_img_full_.image);


      std::string mapmetadatafile = full_file_path_and_name + ".yaml";

      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
           FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

      /*
      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
             orientation.x,
             orientation.y,
             orientation.z,
             orientation.w
           ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);
      */

      double yaw = 0.0;

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                   p_map_name_.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      //ROS_INFO("Done\n");
    
    //}

    // Only if someone is subscribed to it, do work and publish tile-based map image Also check if pose_ptr_ is valid
    /*
    if ((image_transport_publisher_tile_.getNumSubscribers() > 0) && (pose_ptr_)){

      world_map_transformer_.setTransforms(*map);

      Eigen::Vector2f rob_position_world (pose_ptr_->pose.position.x, pose_ptr_->pose.position.y);
      Eigen::Vector2f rob_position_map (world_map_transformer_.getC2Coords(rob_position_world));

      Eigen::Vector2i rob_position_mapi (rob_position_map.cast<int>());

      Eigen::Vector2i tile_size_lower_halfi (p_size_tiled_map_image_x_ / 2, p_size_tiled_map_image_y_ / 2);

      Eigen::Vector2i min_coords_map (rob_position_mapi - tile_size_lower_halfi);

      //Clamp to lower map coords
      if (min_coords_map[0] < 0){
        min_coords_map[0] = 0;
      }

      if (min_coords_map[1] < 0){
        min_coords_map[1] = 0;
      }

      Eigen::Vector2i max_coords_map (min_coords_map + Eigen::Vector2i(p_size_tiled_map_image_x_,p_size_tiled_map_image_y_));

      //Clamp to upper map coords
      if (max_coords_map[0] > size_x){

        int diff = max_coords_map[0] - size_x;
        min_coords_map[0] -= diff;

        max_coords_map[0] = size_x;
      }

      if (max_coords_map[1] > size_y){

        int diff = max_coords_map[1] - size_y;
        min_coords_map[1] -= diff;

        max_coords_map[1] = size_y;
      }

      //Clamp lower again (in case the map is smaller than the selected visualization window)
      if (min_coords_map[0] < 0){
        min_coords_map[0] = 0;
      }

      if (min_coords_map[1] < 0){
        min_coords_map[1] = 0;
      }

      Eigen::Vector2i actual_map_dimensions(max_coords_map - min_coords_map);

      cv::Mat* map_mat  = &cv_img_tile_.image;

      // resize cv image if it doesn't have the same dimensions as the selected visualization window
      if ( (map_mat->rows != actual_map_dimensions[0]) || (map_mat->cols != actual_map_dimensions[1])){
        *map_mat = cv::Mat(actual_map_dimensions[0], actual_map_dimensions[1], CV_8U);
      }

      const std::vector<int8_t>& map_data (map->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      int y_img = max_coords_map[1]-1;

      for (int y = min_coords_map[1]; y < max_coords_map[1];++y){

        int idx_map_y = y_img-- * size_x;
        int idx_img_y = (y-min_coords_map[1]) * actual_map_dimensions.x();

        for (int x = min_coords_map[0]; x < max_coords_map[0];++x){

          int img_index = idx_img_y + (x-min_coords_map[0]);

          switch (map_data[idx_map_y+x])
          {
          case 0:
            map_mat_data_p[img_index] = 255;
            break;

          case -1:
            map_mat_data_p[img_index] = 127;
            break;

          case 100:
            map_mat_data_p[img_index] = 0;
            break;
          }
        }        
      }
      image_transport_publisher_tile_.publish(cv_img_tile_.toImageMsg());
    }
    */
  }

  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;

  image_transport::Publisher image_transport_publisher_full_;
  //image_transport::Publisher image_transport_publisher_tile_;

  image_transport::ImageTransport* image_transport_;

  geometry_msgs::PoseStampedConstPtr pose_ptr_;

  geometry_msgs::PoseStamped latest_update_pose_;
  ros::Time latest_update_time_;

  cv_bridge::CvImage cv_img_full_;
  //cv_bridge::CvImage cv_img_tile_;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;

  int p_size_tiled_map_image_x_;
  int p_size_tiled_map_image_y_;

  HectorMapTools::CoordinateTransformer<float> world_map_transformer_;

  std::string p_save_folder_;
  std::string p_add_script_executable_name_;
  std::string p_scripts_folder_;
  std::string p_map_name_;
  std::string p_agent_name_;
  std::string p_format_string_;

  std::stringstream filename_ss_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_to_image_node");

  MapAsImageProvider map_image_provider;

  ros::spin();

  return 0;
}
