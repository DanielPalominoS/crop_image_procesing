#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
/*#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>*/
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <hector_image_processing/crop.h>
#include <hector_image_processing/image_point.h>
#include  <hector_image_processing/find_crop_features.h>
#include  <hector_image_processing/take_save_picture.h>

#include  "crop_image_processing/Find_crop_features.h"
#include  "crop_image_processing/Take_save_picture.h"
#include  "crop_image_processing/Image_point.h"
#include  "crop_image_processing/Crop.h"

#include  <string>

  #include <fstream>
  #include <sstream>
  #include <unistd.h>
  #include <stdio.h>
  #include <sys/stat.h>
  #include <glob.h>
  #include <string.h>
  #include <math.h>


#ifndef CROP_IMAGE_PROCESSOR_H
#define CROP_IMAGE_PROCESSOR_H
static const std::string LINES_WINDOW = "crop-rows window";
static const std::string ROWS_WINDOW = "Crop rows window";
static const std::string CONTOUR_WINDOW = "Crop contour window";
class crop_image_processor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::ServiceServer saveService;
  ros::ServiceServer cropService;
  cv::Mat original;
public:
  crop_image_processor();
  crop_image_processor(std::string name_);
  ~crop_image_processor();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  /*bool saveIm(hector_image_processing::take_save_picture::Request &req,
              hector_image_processing::take_save_picture::Response &res);
  bool findCropFeatures(hector_image_processing::find_crop_features::Request &req,
                        hector_image_processing::find_crop_features::Response &res);*/
  bool saveIm(crop_image_processing::Take_save_picture::Request &req,
              crop_image_processing::Take_save_picture::Response &res);
  bool findCropFeatures(crop_image_processing::Find_crop_features::Request &req,
                        crop_image_processing::Find_crop_features::Response &res);
  void hsv_mask(cv::Mat *src,cv::Mat *dst ,int low_H,int low_S,
                int low_V,int high_H,int high_S,int high_V);
  void skeleton(cv::Mat *src,cv::Mat *dst, double gsd );
  void hough_detec(cv::Mat *src,std::vector<cv::Vec4i>* p_lines,
                   std::vector<cv::Vec2f>* lines, double gsd,double rho_res,
                   double theta_res,int min_row_lenght);
  double mode(std::vector<float> data);
  void  find_contour(cv::Mat *src,std::vector<cv::Point>* contour,
                     cv::RotatedRect* minRect,double gsd);
  void  start();

};




#endif // CROP_IMAGE_PROCESSOR_H
