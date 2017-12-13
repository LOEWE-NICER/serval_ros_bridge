#ifndef _EXIFTOOL_ROS_H_
#define _EXIFTOOL_ROS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <topic_tools/shape_shifter.h>
#include <sensor_msgs/NavSatFix.h>


namespace exiftool_ros{

    /**
     * Convenience class for wrapping functionality that writes metadata
     * to image files. Currently uses system calls, but could be refactored
     * to use the ExifTool C++ API for better performance.
     */
class ExifTool{
public:
    ExifTool() : exiftool_available_(false)
    {
      //check if exiftool exists
      if (system("exiftool -ver &>/dev/null") != 0) {
        ROS_ERROR_STREAM("exiftool not found. geotagging images will be disabled" << std::endl);
        ROS_ERROR_STREAM("On Ubuntu, use 'sudo apt-get install libimage-exiftool-perl' to install");
        exiftool_available_ = false;
      }else{
        exiftool_available_ = true;
      }    
    };
    
    ~ExifTool()
    {};
    
    
    bool writeGpsData(const std::string full_file_path_and_name, const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
    {
      if (exiftool_available_){
        if (nav_sat_fix_ptr){

          std::stringstream sys_command;
          // See https://www.sno.phy.queensu.ca/~phil/exiftool/faq.html#Q14
          // http://www.awaresystems.be/imaging/tiff/tifftags/privateifd/gps.html
          sys_command << "exiftool -exif:gpslatitude=" << nav_sat_fix_ptr->latitude << " -exif:gpslatituderef=N -exif:gpslongitude="
                      << nav_sat_fix_ptr->longitude << " -exif:gpslongituderef=E  -exif:gpsaltitude="
                      << nav_sat_fix_ptr->altitude <<  " -exif:gpsaltituderef=above " << full_file_path_and_name.c_str();

          if (system(sys_command.str().c_str()) < 0){
            ROS_ERROR("Failed to use system call: %s", sys_command.str().c_str());
            return false;
          }else{
            ROS_INFO("Successfully used system call: %s", sys_command.str().c_str());
            return true;
          }
        }else{
          ROS_WARN("No navsatfix available!");
          return false;
        }
      }else{
        ROS_WARN("Not adding geotag information as exiftool is not available!");
        return false;
      }
    };

protected:
    
    bool exiftool_available_;
    
};

}

#endif
