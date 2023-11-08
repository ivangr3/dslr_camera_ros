#include "dslr_camera_ros/dslr_camera_ros.h"


/** \brief Constructor: Initialize gphoto structures, timers, services and publishers.
 */
CameraDSLR::CameraDSLR(){

  // Get node handle
  nh_ = ros::NodeHandle("~");

  // Get parameters
  bool pub_preview; double preview_rate;
  nh_.getParam("pub_preview", pub_preview);
  nh_.getParam("preview_rate", preview_rate);
  if(pub_preview) ROS_INFO("Publishing preview. Rate: %f Hz", preview_rate);

  // New camera device and context
  gp_camera_new(&camera);
  context = gp_context_new();
 
  // Autodetect camera (it connects to the first camera found)
  ROS_INFO("Searching camera...");
  int ret = gp_camera_init(camera, context);
  while (ret < GP_OK && ros::ok()) {
   ROS_ERROR("No camera auto-detected.");
   ros::Duration(1).sleep();
   ret = gp_camera_init(camera, context);
  }
  ROS_INFO("Camera detected!");

  
  it_ = new image_transport::ImageTransport(nh_);
  preview_pub = it_->advertise("preview", 1);

  // Advertise capture service
  capture_srv = nh_.advertiseService("capture", &CameraDSLR::captureCallback, this);

  // Camera file to get pictures
  gp_file_new(&file);

  if(pub_preview){
    // Camera file to get preview
    gp_file_new(&preview_file);

    // Timer for getting preview
    preview_timer = nh_.createTimer(ros::Duration(1/preview_rate), &CameraDSLR::capture_preview, this);
  }

}


/** \brief Destructor.
 */
CameraDSLR::~CameraDSLR(){
  // Unref camera device and context
  gp_camera_unref(camera);
  gp_context_unref(context);
  gp_file_free(file);
  gp_file_free(preview_file);
}


/** \brief Callback for capture service.
 */
bool CameraDSLR::captureCallback(dslr_camera_ros::Capture::Request  &req, dslr_camera_ros::Capture::Response &res){
  // Call function to take picture and save it in the received path
  capture(req.path.c_str());
  return true;
}

/** \brief Takes picture and saves it in the received path.
 *  @param filename Path to save picture. Must include picture name.
 */
void CameraDSLR::capture(const char *filename){

 ROS_INFO("Taking picture...");

 // Take picture
 CameraFilePath camera_file_path;
 gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
 
 // Copy picture from camera
 gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name, GP_FILE_TYPE_NORMAL, file, context);

 // Save picture in the received path
 gp_file_save(file, filename);
 ROS_INFO("Picture saved in %s", filename);
 
 // Remove picture from camera memory
 gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name, context);

 // Wait for camera to complete everything 
 int waittime = 2000;
 CameraEventType type;
 void *data;
 while(1) {
  gp_camera_wait_for_event(camera, waittime, &type, &data, context);
  if(type == GP_EVENT_TIMEOUT) {
   break;
  }
  else if (type == GP_EVENT_CAPTURE_COMPLETE) {
   ROS_INFO("Capture completed.");
   waittime = 100;
  }
  else if (type != GP_EVENT_UNKNOWN) {
   ROS_WARN("Unexpected event received from camera: %d", (int)type);
  }
 }
}


/** \brief Gets and publishes camera preview.
 */
void CameraDSLR::capture_preview(const ros::TimerEvent&){
 char *data;
 unsigned long int size;
 
 // Get preview image
 gp_camera_capture_preview(camera, preview_file, context);
 gp_file_get_data_and_size (preview_file, (const char**)&data, &size);
 
 // Copy data into a cv matrix
 cv::Mat matImg;
 try
 {
   matImg = cv::imdecode(cv::Mat(1, size, CV_8UC1, data), CV_LOAD_IMAGE_UNCHANGED);
 }
 catch(const std::exception& e)
 {
   std::cerr << e.what() << '\n';
 }
 
 // Publish Image
 cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
 ros::Time time = ros::Time::now();
 cv_ptr->encoding = "bgr8";
 cv_ptr->header.stamp = time;
 cv_ptr->header.frame_id = "/camera";
 cv_ptr->image = matImg;
 preview_pub.publish(cv_ptr->toImageMsg());

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dslr_camera_ros");
  CameraDSLR myCamera;
  ros::spin();
  return 0;
}