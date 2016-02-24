#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>

#include "ros/ros.h"
#include "ros/time.h"
#include "serval_msgs/NewServalFile.h"

int main(int argc, char **argv)
{

  if (argc != 6)
  {
    std::cout << "Wrong size of parameters " << argc << std::endl;

    return 1;
  }

  argc = 1;
  ros::init(argc, argv, "serval_to_msg");
  ros::NodeHandle nh_;
  ros::Publisher pub = nh_.advertise<serval_msgs::NewServalFile>("serval/NewFile", 1);
  ros::Rate poll_rate(100);

  ros::Time maxTime(ros::Time::now()+ros::Duration(1)); // Will wait at most 1000 ms

  int numExistingSubscribers = pub.getNumSubscribers();

  while(numExistingSubscribers == 0 && ros::Time::now() < maxTime)
  {
      poll_rate.sleep();
      numExistingSubscribers = pub.getNumSubscribers();
  }

  if (numExistingSubscribers == 0)
  {
    std::cerr << "No subscribers found, quit" << std::endl;
    return 1;
  }

  serval_msgs::NewServalFile nsf;

  nsf.filePath = argv[1];
  nsf.fileName = argv[2];
  nsf.fileSize = argv[3];
  nsf.fileHash = argv[4];
  nsf.authorSid = argv[5];

  pub.publish(nsf);

  return 1;
}

