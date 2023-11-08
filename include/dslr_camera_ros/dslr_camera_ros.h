#ifndef _DSLR_CAMERA_ROS_H_
#define _DSLR_CAMERA_ROS_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <gphoto2/gphoto2-camera.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dslr_camera_ros/Capture.h>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <fcntl.h>

class CameraDSLR
{
public:
  CameraDSLR();
  ~CameraDSLR();
private:
  void capture (const char *filename);
  void capture_preview(const ros::TimerEvent&);
  bool captureCallback(dslr_camera_ros::Capture::Request  &req, dslr_camera_ros::Capture::Response &res);

  // GPHOTO2
  Camera *camera;
  GPContext *context;
  CameraFile *preview_file;
  CameraFile *file;

  ros::NodeHandle nh_;

  // Timers
  ros::Timer preview_timer;

  // Services
  ros::ServiceServer capture_srv;

  // Publishers
  image_transport::ImageTransport *it_;
  image_transport::Publisher preview_pub;
};

#endif // _DSLR_CAMERA_ROS_H_