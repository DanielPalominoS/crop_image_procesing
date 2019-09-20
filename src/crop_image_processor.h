#ifndef CROP_IMAGE_PROCESSOR_H
#define CROP_IMAGE_PROCESSOR_H
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
/*#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>*/
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Point.h>

#include <opencv2/core/utility.hpp>
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudabgsegm.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudawarping.hpp"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/core/cuda.hpp>

#include  "crop_image_processing/Find_crop_features.h"
#include  "crop_image_processing/Take_save_picture.h"
#include  "crop_image_processing/Image_point.h"
#include  "crop_image_processing/Crop.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include  <string>
#include  <time.h>
#include  <boost/filesystem.hpp>


static const std::string LINES_WINDOW = "crop-rows window";
static const std::string ROWS_WINDOW = "Crop rows window";
static const std::string CONTOUR_WINDOW = "Crop contour window";

static const std::string kDefaultImageSubTopic = "/iris/c920/image_raw";
static const std::string kDefaultFolder = "~/Pictures";
static const std::string kDefaultTakePicTopic = "crop_image_server/take_picture";
static const std::string kDefaultFindCropFeatTopic = "crop_image_server/find_crop_features";

#ifdef SHOW_IMG
static const std::string LINES_WINDOW = "crop lines window";
static const std::string RESULT_WINDOW = "result";
static const std::string CONTOUR_WINDOW = "Crop contour window";
static const std::string MASK_WINDOW = "mask window";
static const std::string EDGES_WINDOW = "edges window";
#endif


static constexpr double kDefaultPubFreq =20.0 ;


class crop_image_processor
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::ServiceServer saveService;
  ros::ServiceServer cropService;
  boost::filesystem::path planning_dir;
  ros::Rate rate_;
  std::string   pictures_path;

  cv::Mat original;
public:
  crop_image_processor();
  crop_image_processor(std::string name_);
  ~crop_image_processor();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  bool saveIm(crop_image_processing::Take_save_picture::Request &req,
              crop_image_processing::Take_save_picture::Response &res);
  bool findCropFeatures(crop_image_processing::Find_crop_features::Request &req,
                        crop_image_processing::Find_crop_features::Response &res);
  double mode(std::vector<float> data);
  void  find_contour(cv::Mat *src,std::vector<cv::Point>* contour,
                     cv::RotatedRect* minRect,double gsd);
  void  main_task();
  void  start();
//#ifdef GPU
  void hsv_mask(cv::cuda::GpuMat *src,cv::cuda::GpuMat *dst ,int low_H,int low_S,
                int low_V,int high_H,int high_S,int high_V);
  void skeleton(cv::cuda::GpuMat *src,cv::cuda::GpuMat *dst, double gsd );
  void hough_detec(cv::cuda::GpuMat *src,std::vector<cv::Vec4i>* p_lines,
                   std::vector<cv::Vec2f>* lines, double gsd,double rho_res,
                   double theta_res,int min_row_lenght);
//#else
  void  find_contour(cv::cuda::GpuMat *src,std::vector<cv::Point>* contour,
                     cv::RotatedRect* minRect,double gsd);
  void hsv_mask(cv::Mat *src,cv::Mat *dst ,int low_H,int low_S,
                int low_V,int high_H,int high_S,int high_V);
  void skeleton(cv::Mat *src,cv::Mat *dst, double gsd );
  void hough_detec(cv::Mat *src,std::vector<cv::Vec4i>* p_lines,
                   std::vector<cv::Vec2f>* lines, double gsd,double rho_res,
                   double theta_res,int min_row_lenght);
//#endif

};

#endif // CROP_IMAGE_PROCESSOR_H
