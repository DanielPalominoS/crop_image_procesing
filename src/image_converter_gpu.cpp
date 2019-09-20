
#include <ros/ros.h>
//#include  <ros/param.h>

#include <sensor_msgs/image_encodings.h>
/*#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>*/
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <crop_image_processing/Crop.h>
#include <crop_image_processing/Image_point.h>

#include <opencv2/core/utility.hpp>
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudabgsegm.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudawarping.hpp"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/core/cuda.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
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
    cv::Mat temp1,temp2,original_cpu;
    cv::cuda::GpuMat gray_image,test4;
    cv::cuda::GpuMat edges_gpu;
    cv::Mat edges;
    cv::cuda::GpuMat colorSeg;
    //cv::Mat contours;
    cv::cuda::GpuMat hsv;
    cv::cuda::GpuMat original_gpu;

    original_cpu=cv_ptr->image;
    original_gpu.upload(original_cpu);
    cv::cuda::cvtColor(original_gpu, hsv, cv::COLOR_BGR2HSV);
    if(original_gpu.channels()==3){
      // gpu only support 4th channel images
      cv::cuda::GpuMat orig4ch;
      cv::cuda::cvtColor(original_gpu,orig4ch,cv::COLOR_BGR2BGRA);
      orig4ch.copyTo(original_gpu);
    }
    //cv_ptr->image.copyTo()

    /*************** Detect the rows based on HSV Range Values ***************/
    int h_green=120/2;//Los angulos estan divididos entre 2 para poder representarlos con un entero
    int h_umbral=20;
    int low_H=h_green-h_umbral, low_S=100, low_V=50,high_H=h_green+h_umbral, high_S=255, high_V=255;
    // Convert from BGR to HSV colorspace
    //cv::cuda::cvtColor(original, hsv, cv::COLOR_BGR2HSV);
    //cv::cuda::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    hsv.download(temp1);

    cv::inRange(temp1, cv::Scalar(low_H, low_S, low_V),
                cv::Scalar(high_H, high_S, high_V), temp2);
    cv::imshow(MASK_WINDOW, temp2);
    colorSeg.upload(temp2);

    // gray image
    //cv::cvtColor(cv_ptr->image,gray_image,CV_BGR2GRAY);
    int morph_size = 3;
    int operation=cv::MORPH_ERODE;//0;
    cv::Mat element ;//= cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

    /***************Skeleton***************/
    cv::cuda::GpuMat img;
    cv::cuda::threshold(colorSeg,img,127,255,cv::THRESH_BINARY);
    //cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);
    //cv::cuda::GpuMat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::cuda::GpuMat skel(img.size(), img.type(), cv::Scalar(0));
    //cv::Mat temp;
    cv::cuda::GpuMat eroded;

    element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    cv::Ptr<cv::cuda::Filter> filter1=cv::cuda::createMorphologyFilter(cv::MORPH_ERODE,
                                                                            img.type(),element);
    cv::Ptr<cv::cuda::Filter> filter2=cv::cuda::createMorphologyFilter(cv::MORPH_DILATE,
                                                                            img.type(),element);

    do
    {
      cv::cuda::GpuMat  tmp1;
      filter1->apply(img,eroded);

      filter2->apply(eroded,tmp1);
      cv::cuda::subtract(img,tmp1,tmp1);

      cv::cuda::bitwise_or(skel,tmp1,skel);

      eroded.copyTo(img);
      done=(cv::cuda::countNonZero(img)==0);
      /*cv::erode(img, eroded, element);
      cv::dilate(eroded, temp1, element); // temp = open(img)
      cv::subtract(img, temp1, temp1);
      cv::bitwise_or(skel, temp1, skel);
      eroded.copyTo(img);
      done = (cv::countNonZero(img) == 0);*/
    } while (!done);
skel.download(temp1);
cv::imshow(LINES_WINDOW, temp1);

    /*************** Hough line detection ***************/

    //cv::Mat skel_s;
    //skel.copyTo(skel_s);


    //std::vector<cv::Vec2f> lines;
    cv::cuda::GpuMat lines;

    //cv::HoughLines( skel, lines, 1, CV_PI/180, 150, 30, 10 );
    cv::Ptr<cv::cuda::HoughLinesDetector> hough=cv::cuda::createHoughLinesDetector(
          1.0f,(float) (CV_PI/180.0f),150,true);
    hough->detect(skel,lines);

    std::vector<cv::Vec2f> lines_gpu;
    if(!lines.empty()){
      lines_gpu.resize(lines.cols);
      cv::Mat temp_lines(1,lines.cols,CV_32FC2,&lines_gpu[0]);
      lines.download(temp_lines);
      temp_lines.release();
    }


    cv::Mat p_hough;

    cv::cuda::GpuMat  p_lines;


    //cv::cvtColor( src_gray, probabilistic_hough, cv::COLOR_GRAY2BGR );
    //cv::cvtColor( morpho_result, probabilistic_hough, cv::COLOR_GRAY2BGR );
    //cv::cvtColor( skel, p_hough, cv::COLOR_GRAY2BGR );

    /// 2. Use Probabilistic Hough Transform
    /// cv::HoughLinesP( src_gray, p_lines, 1, CV_PI/180, hoguh_min_threshold + hough_p_trackbar, 30, 10 );
    cv::Ptr<cv::cuda::HoughSegmentDetector> hough_p=cv::cuda::createHoughSegmentDetector(
          1.0f,(float) (CV_PI/180.0f),30,5);
    //cv::HoughLinesP( skel, p_lines, 1, CV_PI/180, 150, 30, 10 );
    //p_lines.data().
    hough_p->detect(skel,p_lines);
    std::vector<cv::Vec4i> p_lines_gpu;
    if (!p_lines.empty())
    {
        p_lines_gpu.resize(p_lines.cols);
        cv::Mat temp_lines(1, p_lines.cols, CV_32SC4, &p_lines_gpu[0]);
        p_lines.download(temp_lines);
        temp_lines.release();
    }

    /// Show the result
    for( size_t i = 0; i < p_lines_gpu.size(); i++ )
       {
         cv::Vec4i l = p_lines_gpu[i];
         //cv::line( probabilistic_hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
         cv::line( original_cpu, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
       }

    /*************** Find theta mode ***************/
    std::vector<float> theta;
    for (int i=0; i<lines_gpu.size(); i++){
      theta.push_back((float)(lines_gpu[i][1]*180.0/((float)M_PI)));
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

    std::cout << "mode : " << mode << "\tcount mode : " << countMode<< std::endl;
    //std::cout << "size : " << (long)theta.size() << std::endl;

    /*************** Find CONTOUR ***************/
    morph_size = 8;
    //operation=cv::MORPH_DILATE;//2;
    element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    /// Apply the specified morphology operation
    cv::cuda::GpuMat morpho_result_2,morpho_result_3;

    // ////cv::morphologyEx( colorSeg, morpho_result_2, operation, element );
    filter2=cv::cuda::createMorphologyFilter(cv::MORPH_DILATE,
                                                   colorSeg.type(),element);
    //cv::dilate(colorSeg,morpho_result_2,element);

    filter2->apply(colorSeg,morpho_result_2);

    morph_size = 3;
    operation=cv::MORPH_CLOSE;//3;
    element= cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    /// Apply the specified morphology operation

    //cv::morphologyEx( morpho_result_2, morpho_result_3, operation, element );
    filter1=cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE,
                                             morpho_result_2.type(),element);

    filter1->apply(morpho_result_2,morpho_result_3);
    morpho_result_3.download(temp1);
    cv::imshow(CONTOUR_WINDOW, temp1);
    // ///edge detection
    //cv::Canny(morpho_result_3,edges,100,255,3);

    cv::Ptr<cv::cuda::CannyEdgeDetector> canny=cv::cuda::createCannyEdgeDetector(100,255,3);

    canny->detect(morpho_result_3,edges_gpu);
    //cv::imshow( morpho_window, morpho_result );

    edges_gpu.download(edges);
    cv::imshow(EDGES_WINDOW, edges);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
//ROS_INFO("llego1");
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
    minRect=cv::minAreaRect(cv::Mat(contours[max_area_index]));
    cv::drawContours( original_cpu, contours, max_area_index, cv::Scalar(0,0,255), 6, 8, hierarchy, 3, cv::Point() );
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
    //cv::imshow(MASK_WINDOW, colorSeg);
    //cv::imshow(LINES_WINDOW, skel);
    //cv::imshow(LINES_WINDOW, probabilistic_hough);
    cv::imshow(RESULT_WINDOW, original_cpu);
    //cv::imshow(CONTOUR_WINDOW, morpho_result_2);
    //cv::imshow(CONTOUR_WINDOW, morpho_result_3);

    cv::waitKey(1);
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
