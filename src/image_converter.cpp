
#include <ros/ros.h>
//#include  <ros/param.h>
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
/*#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>*/
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <crop_image_processing/Crop.h>
#include <crop_image_processing/Image_point.h>

#include "opencv2/core/utility.hpp"
/*#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudabgsegm.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudawarping.hpp"
#include <opencv2/cudaarithm.hpp>*/
#include <opencv2/core/cuda.hpp>
//#include <ros/rate.h>

static const std::string LINES_WINDOW = "crop lines window";
static const std::string RESULT_WINDOW = "result";
static const std::string CONTOUR_WINDOW = "Crop contour window";
static const std::string MASK_WINDOW = "mask window";
static const std::string EDGES_WINDOW = "edges window";
static const std::string kDefaultImageSubTopic = "/iris/c920/image_raw";
static constexpr double kDefaultPubFreq =1.0 ;
static const std::string kDefaultResultPubTopic = "image_converter/output_video";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Rate rate_;//(kDefaultPubFreq);
  //std::string image_sub_topic;

public:
  ImageConverter()
    : it_(nh_),
    rate_(kDefaultPubFreq)
  {
    ros::NodeHandle pnh("~");
    double publish_frequency;
    std::string image_sub_topic;
    std::string result_pub_topic;
    pnh.param("image_sub_topic", image_sub_topic, kDefaultImageSubTopic);
    pnh.param("result_pub_topic",result_pub_topic,kDefaultResultPubTopic);
    pnh.param("publish_frequency",publish_frequency,kDefaultPubFreq);
    //Subscribe to input video feed and publish output video feed
    /*image_sub_ = it_.subscribe("/down_cam1/camera/image", 1,
      &ImageConverter::imageCb, this);
    image_sub_ = it_.subscribe("/iris/down_cam/camera/image", 1,
    image_sub_ = it_.subscribe("/iris/c920/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);*/
    ROS_INFO("image sub topic:");std::cout<<image_sub_topic<<std::endl;
    image_sub_ = it_.subscribe(image_sub_topic, 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise(result_pub_topic, 1);
    rate_=ros::Rate(publish_frequency);
    cv::namedWindow(LINES_WINDOW);
    cv::namedWindow(CONTOUR_WINDOW);
    cv::namedWindow(RESULT_WINDOW);
    cv::namedWindow(MASK_WINDOW);
    cv::namedWindow(EDGES_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(LINES_WINDOW);
    cv::destroyWindow(CONTOUR_WINDOW);
    cv::destroyWindow(RESULT_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    /*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(10, 50), 10, CV_RGB(255,0,0));*/

    cv::Mat gray_image,test4;
    cv::Mat edges;
    cv::Mat colorSeg;
    //cv::Mat contours;
    cv::Mat hsv;
    cv::Mat original;
    //cv::cuda::GpuMat  test1,test2,test3;
    //cv::cuda::CannyEdgeDetector()
    original=cv_ptr->image;
    /*************** Detect the rows based on HSV Range Values ***************/
    int h_green=120/2;//Los angulos estan divididos entre 2 para poder representarlos con un entero
    int h_umbral=20;
    int low_H=h_green-h_umbral, low_S=100, low_V=50,high_H=h_green+h_umbral, high_S=255, high_V=255;
    // Convert from BGR to HSV colorspace
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    /*test1.upload(cv_ptr->image);
    test2.upload(cv_ptr->image);
    cv::Ptr<cv::cuda::HoughSegmentDetector> hough = cv::cuda::createHoughSegmentDetector(1.0f, (float) (CV_PI / 180.0f), 50, 5);
    cv::cuda::add(test1,test2,test3);

    test3.download(test4);
    test1.release();
    test2.release();
    test3.release();
    test4.release();*/

    cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), colorSeg);

    // gray image
    //cv::cvtColor(cv_ptr->image,gray_image,CV_BGR2GRAY);
    int morph_size = 3;
    int operation=0;
    cv::Mat element = cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

    /*// morphologycal transform lines
    int morph_size = 3;
    int operation=0;
    cv::Mat element = cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

    /// Apply the specified morphology operation
    cv::Mat morpho_result;
    cv::morphologyEx( colorSeg, morpho_result, operation, element );
    //cv::imshow( morpho_window, morpho_result );

    // edge detection
    cv::Canny(colorSeg,edges,100,255,3);

    //cv::waitKey(3);*/

    /***************Skeleton***************/
    cv::Mat img;
    colorSeg.copyTo(img);
    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;

    element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do
    {
      cv::erode(img, eroded, element);
      cv::dilate(eroded, temp, element); // temp = open(img)
      cv::subtract(img, temp, temp);
      cv::bitwise_or(skel, temp, skel);
      eroded.copyTo(img);

      done = (cv::countNonZero(img) == 0);
    } while (!done);


    /*************** Hough line detection ***************/

    cv::Mat skel_s;
    skel.copyTo(skel_s);


    std::vector<cv::Vec2f> lines;


    cv::HoughLines( skel, lines, 1, CV_PI/180, 150, 30, 10 );


    cv::Mat probabilistic_hough;

    std::vector<cv::Vec4i> p_lines;

    //cv::cvtColor( src_gray, probabilistic_hough, cv::COLOR_GRAY2BGR );
    //cv::cvtColor( morpho_result, probabilistic_hough, cv::COLOR_GRAY2BGR );
    cv::cvtColor( skel, probabilistic_hough, cv::COLOR_GRAY2BGR );

    /// 2. Use Probabilistic Hough Transform
    /// cv::HoughLinesP( src_gray, p_lines, 1, CV_PI/180, hoguh_min_threshold + hough_p_trackbar, 30, 10 );

    cv::HoughLinesP( skel_s, p_lines, 1, CV_PI/180, 150, 30, 10 );
    //p_lines.data().

    /// Show the result
    for( size_t i = 0; i < p_lines.size(); i++ )
       {
         cv::Vec4i l = p_lines[i];
         //cv::line( probabilistic_hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
         cv::line( original, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
       }
    /*************** Find theta mode ***************/
    std::vector<float> theta;
    for (int i=0; i<lines.size(); i++){
      theta.push_back((float)(lines[i][1]*180.0/((float)M_PI)));
    }

    std::sort(theta.begin(), theta.end());
    /*for (int i=0; i<theta.size(); i++){
     std::cout<< theta[i] << ";";
    }
    std::cout<<std::endl;*/
    float number = theta[0];
    float mode = number;
    long count = 1;
    long countMode = 1;
    for (int i=1; i<theta.size(); i++)
    {
      if (theta[i] == number)
      { // count occurrences of the current number
         ++count;
      }
      else
      { // now this is a different number
            if (count > countMode)
            {
                  countMode = count; // mode is the biggest ocurrences
                  mode = number;
            }
           count = 1; // reset count for the new number
           number = theta[i];
      }
    }

    //std::cout << "mode : " << mode << "\tcount mode : " << countMode<< std::endl;
    //std::cout << "size : " << (long)theta.size() << std::endl;

    /*************** Find CONTOUR ***************/
    morph_size = 8;
    operation=2;
    element = cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    /// Apply the specified morphology operation
    cv::Mat morpho_result_2,morpho_result_3;
    cv::morphologyEx( colorSeg, morpho_result_2, operation, element );
    //cv::cuda::mor
    cv::dilate(colorSeg,morpho_result_2,element);

    morph_size = 3;
    operation=3;
    element= cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    /// Apply the specified morphology operation
    cv::morphologyEx( morpho_result_2, morpho_result_3, operation, element );
    // edge detection
    cv::Canny(morpho_result_3,edges,100,255,3);
    //cv::imshow( morpho_window, morpho_result );

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    cv::RotatedRect minRect;
    cv::Mat draw=cv::Mat::zeros(edges.size(),CV_8UC3);
    /*for( int i = 0; i< contours.size(); i++ )
         {
           //cv::Scalar color = cv::Scalar( rng.uniform(0, 255), cv::rng.uniform(0,255), rng.uniform(0,255) );
           cv::drawContours( draw, contours, i, cv::Scalar(0,0,255), 2, 8, hierarchy, 3, cv::Point() );
         }*/
    int max_area_index=0;
    float area=0,max_area=0;
    for( int i = 0; i< contours.size(); i++ )
    {
      area=cv::contourArea(contours[i]);
      if (area>max_area){max_area_index=i;max_area=area;}
    }
    std::vector<cv::Point> contour_approx;
    //Mat(contours0[k]), contours[k]
    cv::approxPolyDP(cv::Mat(contours[max_area_index]),contour_approx,30,false);
    std::cout<<"orig cont size: "<<contours[max_area_index].size()<<std::endl;
    std::cout<<"approx cont size: "<<contour_approx.size()<<std::endl;
    cv::line( original, contour_approx[contour_approx.size()-1], contour_approx[0], cv::Scalar(255,255,0), 1, 8 );
    for( int i = 0; i < contour_approx.size()-1; i++ ){
       cv::line( original, contour_approx[i], contour_approx[i+1], cv::Scalar(255,255,0), 1, 8 );
    }


    minRect=cv::minAreaRect(cv::Mat(contours[max_area_index]));
    //cv::drawContours( original, contours, max_area_index, cv::Scalar(0,0,255), 6, 8, hierarchy, 3, cv::Point() );
    cv::drawContours( draw, contours, max_area_index, cv::Scalar(0,0,255), 2, 8, hierarchy, 3, cv::Point() );
    cv::Point2f rect_Points[4];
    minRect.points(rect_Points);
    //for( int j = 0; j < 4; j++ ){
    //          cv::line( original, rect_Points[j], rect_Points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
    //}
    //std::cout << "mode : " << mode << "\tcount mode : " << countMode<< "\tcontours : " << contours.at(max_area_index)<< "\tsequence : " << contours[max_area_index].size()<< std::endl;
    /*cv::Point* p=(cv::Point*) cv::getSeqElem(contours[max_area_index],1);
    std::cout << "x : " << p->x << "\ty : "<<p->y << std::endl;*/
    //std::cout << "RECT POINTS : " << *rect_Points << "RECT angle : " << minRect.angle<<  std::endl;
    // Update GUI Window
    cv::imshow(MASK_WINDOW, colorSeg);
    cv::imshow(LINES_WINDOW, skel);
    //cv::imshow(LINES_WINDOW, probabilistic_hough);
    cv::imshow(RESULT_WINDOW, original);
    //cv::imshow(CONTOUR_WINDOW, morpho_result_2);
    cv::imshow(CONTOUR_WINDOW, morpho_result_3);
    cv::imshow(EDGES_WINDOW, edges);
    cv::waitKey(3);
    // Output modified video stream

    crop_image_processing::Crop crop_feat;
    crop_feat.row_orientation=mode;
    crop_feat.contour.resize(4);
    for(int i=0;i<4;i++){
      //geometry_msgs::Point coor;
      crop_image_processing::Image_point coor;
      coor.x=rect_Points[i].x;
      coor.y=rect_Points[i].y;
      crop_feat.contour.at(i)=coor;
    }

    //std::cout << "crop_feat : " << crop_feat <<  std::endl;
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  void MainTask() {
    while (ros::ok()) {
      //image_pub_.publish(cv_ptr->toImageMsg());
      //ROS_INFO("image sub topic:");std::cout<<image_sub_topic<<std::endl;
      ros::spinOnce();
      rate_.sleep();
    }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  //ros::spin();
  ic.MainTask();
  return 0;
}
