#include "crop_image_processor.h"

crop_image_processor::crop_image_processor()
  : it_(nh_)
{
  ROS_INFO("Ready to take pictures");
  image_sub_ = it_.subscribe("//iris/c920/image_raw", 1,
    &crop_image_processor::imageCb, this);
  //image_pub_ = it_.advertise("/image_converter/output_video", 1);
  saveService = nh_.advertiseService("crop_image_server/take_picture",&crop_image_processor::saveIm,this);
  cropService = nh_.advertiseService("crop_image_server/find_crop_features",&crop_image_processor::findCropFeatures,this);
}
crop_image_processor::crop_image_processor(std::string name_)
  : it_(nh_)
{
  ROS_INFO("Ready to take pictures");
  image_sub_ = it_.subscribe("/iris/c920/image_raw", 1,
    &crop_image_processor::imageCb, this);
  //image_pub_ = it_.advertise("/image_converter/output_video", 1);
  saveService = nh_.advertiseService(name_+"/take_picture",&crop_image_processor::saveIm,this);
  cropService = nh_.advertiseService(name_+"/find_crop_features",&crop_image_processor::findCropFeatures,this);
}
crop_image_processor::~crop_image_processor(){
}



void crop_image_processor::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

  cv::Mat gray_image;
  cv::Mat edges;
  cv::Mat colorSeg;
  //cv::Mat contours;
  cv::Mat hsv;

  original=cv_ptr->image;

  //std::cout << "crop_feat : " << crop_feat <<  std::endl;
  //image_pub_.publish(cv_ptr->toImageMsg());
}
/*bool crop_image_processor::saveIm(hector_image_processing::take_save_picture::Request &req,
            hector_image_processing::take_save_picture::Response &res){
  //pendiente incluir hora
  std::string file_name=req.storage_path+req.name+".jpg";
  ROS_INFO("saving image: ");std::cout<<file_name<<std::endl;
  std::cout<<ros::Time::now()<<std::endl;

  if(!original.empty()){
    cv::imwrite(file_name,original);
    ROS_INFO("saved image: ");std::cout<<file_name<<std::endl;
    res.success=true;
  }
  else{
    ROS_INFO("Not possible to save image: ");std::cout<<file_name<<std::endl;
    res.success=false;
    return true;
  }

  return true;
}*/
/*bool crop_image_processor::findCropFeatures(hector_image_processing::find_crop_features::Request &req,
                      hector_image_processing::find_crop_features::Response &res){
  cv::Mat color_segmented_mask;
  cv::Mat skeleton_;
  cv::Mat orig_copy;
  std::vector<cv::Vec4i> p_lines;
  std::vector<cv::Vec2f> lines;
  std::vector<float> theta;
  cv::RotatedRect minRect;
  std::vector<cv::Point> contour;
  std::string file_name;
  double  theta_mode;

  double  gsd=0.08;
  double  rho_res=1;
  double  theta_res=CV_PI/180.0;
  double  min_row_lenght=10;

  int h_green=120/2;//Los angulos estan divididos entre 2 para poder representarlos con un entero sencillo
  int h_umbral=20;
  int low_H=h_green-h_umbral, low_S=100, low_V=50,high_H=h_green+h_umbral, high_S=255, high_V=255;
  res.success=false;

  original.copyTo(orig_copy);
  hsv_mask(&orig_copy,&color_segmented_mask,low_H,low_S,low_V,high_H,high_S,high_V);
  skeleton(&color_segmented_mask,&skeleton_,gsd);
  hough_detec(&skeleton_,&p_lines,&lines,gsd,rho_res,theta_res,min_row_lenght);

  for (int i=0; i<lines.size(); i++){
    theta.push_back((float)(lines[i][1]*180.0/((float)M_PI)));
  }

  theta_mode=-mode(theta)+90;
  find_contour(&color_segmented_mask,&contour,&minRect,gsd);
  cv::Point2f rect_Points[4];
  minRect.points(rect_Points);
  if (req.save){
    file_name=req.storage_path+req.name+".jpg";*/
    /**********Paint contours and lines   ***********/
    /*for( size_t i = 0; i < p_lines.size(); i++ )
       {
         cv::Vec4i l = p_lines[i];
         //cv::line( probabilistic_hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
         cv::line( orig_copy, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 1, cv::LINE_AA);
       }
    for( int j = 0; j < 4; j++ ){
              cv::line( orig_copy, rect_Points[j], rect_Points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
    }

    cv::imwrite(file_name,orig_copy);
  }


  std::cout<<"contour size :"<<contour.size()<<std::endl;

  hector_image_processing::crop crop_feat;
  crop_feat.row_orientation=theta_mode;
  crop_feat.contour.resize(4);
  //Modificar esto para trabajar el controno completo y no solo el bounding_box
  for(int i=0;i<4;i++){
    //geometry_msgs::Point coor;
    hector_image_processing::image_point coor;
    coor.x=rect_Points[i].x;
    coor.y=rect_Points[i].y;
    crop_feat.contour.at(i)=coor;
  }
  res.crop=crop_feat;
  std::cout<<"crop coordinates: "<<rect_Points<<std::endl;
  std::cout<<"crop coordinates: "<<crop_feat<<std::endl;
  res.success=true;
  return true;
}*/

bool crop_image_processor::saveIm(crop_image_processing::Take_save_picture::Request &req,
            crop_image_processing::Take_save_picture::Response &res){
  //pendiente incluir hora
  std::string file_name=req.storage_path+req.name+".jpg";
  ROS_INFO("saving image: ");std::cout<<file_name<<std::endl;
  std::cout<<ros::Time::now()<<std::endl;

  if(!original.empty()){
    cv::imwrite(file_name,original);
    ROS_INFO("saved image: ");std::cout<<file_name<<std::endl;
    res.success=true;
  }
  else{
    ROS_INFO("Not possible to save image: ");std::cout<<file_name<<std::endl;
    res.success=false;
    return true;
  }

  return true;
}


bool crop_image_processor::findCropFeatures(crop_image_processing::Find_crop_features::Request &req,
                      crop_image_processing::Find_crop_features::Response &res){
  cv::Mat color_segmented_mask;
  cv::Mat skeleton_;
  cv::Mat orig_copy;
  std::vector<cv::Vec4i> p_lines;
  std::vector<cv::Vec2f> lines;
  std::vector<float> theta;
  cv::RotatedRect minRect;
  std::vector<cv::Point> contour;
  std::string file_name;
  double  theta_mode;

  double  gsd=0.08;
  double  rho_res=1;
  double  theta_res=CV_PI/180.0;
  double  min_row_lenght=10;

  int h_green=120/2;//Los angulos estan divididos entre 2 para poder representarlos con un entero sencillo
  int h_umbral=20;
  int low_H=h_green-h_umbral, low_S=100, low_V=50,high_H=h_green+h_umbral, high_S=255, high_V=255;
  res.success=false;

  original.copyTo(orig_copy);
  hsv_mask(&orig_copy,&color_segmented_mask,low_H,low_S,low_V,high_H,high_S,high_V);
  skeleton(&color_segmented_mask,&skeleton_,gsd);
  hough_detec(&skeleton_,&p_lines,&lines,gsd,rho_res,theta_res,min_row_lenght);

  for (int i=0; i<lines.size(); i++){
    theta.push_back((float)(lines[i][1]*180.0/((float)M_PI)));
  }

  theta_mode=-mode(theta)+90;
  find_contour(&color_segmented_mask,&contour,&minRect,gsd);
  cv::Point2f rect_Points[4];
  minRect.points(rect_Points);
  if (req.save){
    file_name=req.storage_path+req.name+".jpg";
    /**********Paint contours and lines   ***********/
    for( size_t i = 0; i < p_lines.size(); i++ )
       {
         cv::Vec4i l = p_lines[i];
         //cv::line( probabilistic_hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
         cv::line( orig_copy, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 1, cv::LINE_AA);
       }
    for( int j = 0; j < 4; j++ ){
              cv::line( orig_copy, rect_Points[j], rect_Points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
    }

    cv::imwrite(file_name,orig_copy);
  }


  std::cout<<"contour size :"<<contour.size()<<std::endl;

  crop_image_processing::Crop crop_feat;
  crop_feat.row_orientation=theta_mode;
  crop_feat.contour.resize(4);
  //Modificar esto para trabajar el controno completo y no solo el bounding_box
  for(int i=0;i<4;i++){
    //geometry_msgs::Point coor;
    crop_image_processing::Image_point coor;
    coor.x=rect_Points[i].x;
    coor.y=rect_Points[i].y;
    crop_feat.contour.at(i)=coor;
  }
  res.crop=crop_feat;
  std::cout<<"crop coordinates: "<<rect_Points<<std::endl;
  std::cout<<"crop coordinates: "<<crop_feat<<std::endl;
  res.success=true;
  return true;
}


void crop_image_processor::hsv_mask(cv::Mat *src,cv::Mat *dst ,int low_H,int low_S,int low_V,int high_H,int high_S,int high_V){
    cv::Mat hsv;
    cv::cvtColor(*src, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), *dst);
    return;
}
void crop_image_processor::skeleton(cv::Mat *src,cv::Mat *dst, double gsd ){
    cv::Mat img;
    src->copyTo(img);
    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;
    int morph_size = 3;
    cv::Mat element;

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
    skel.copyTo(*dst);
    return;
}
void crop_image_processor::hough_detec(cv::Mat *src,std::vector<cv::Vec4i>* p_lines,
                 std::vector<cv::Vec2f>* lines, double gsd,double rho_res,
                 double theta_res,int min_row_lenght){
    cv::Mat img;
    cv::Mat img_co;
    src->copyTo(img);
    img.copyTo(img_co);
    //Depending on the gsd and min_row_lenght rows will be accepted
    int threshold=(int)(min_row_lenght/gsd);
    //std::vector<cv::Vec2f> lines;
    cv::HoughLines( img, *lines, rho_res, theta_res, threshold, 30, 10 );
    //cv::HoughLines( img, l, rho_res, theta_res, 150, 30, 10 );
    //cv::Mat probabilistic_hough;
    //std::vector<cv::Vec4i> p_lines;
    //cv::cvtColor( img, probabilistic_hough, cv::COLOR_GRAY2BGR );
    cv::HoughLinesP( img_co, *p_lines, rho_res, theta_res, threshold, 30, 10 );

    return;
}
double crop_image_processor::mode(std::vector<float> data){
  /*for (int i=0; i<lines.size(); i++){
    data.push_back((float)(lines[i][1]*180.0/((float)M_PI)));
  }*/

  std::sort(data.begin(), data.end());
  /*for (int i=0; i<theta.size(); i++){
   std::cout<< theta[i] << ";";
  }
  std::cout<<std::endl;*/
  double number = data[0];
  double mode = number;
  long count = 1;
  long countMode = 1;
  for (int i=1; i<data.size(); i++)
  {
        if (data[i] == number)
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
             number = data[i];
    }
  }
  return mode;
}
void  crop_image_processor::find_contour(cv::Mat *src,std::vector<cv::Point>* contour,cv::RotatedRect* minRect,double gsd){
  int morph_size;
  int operation;
  cv::Mat element;
  cv::Mat img;
  cv::Mat edges;
  cv::Mat morpho_result_2,morpho_result_3;

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat draw;

  /******* Apply morphologycal operations to increase birght areas (crop rows) *********/
  src->copyTo(img);
  morph_size = 8;
  operation=2;
  element = cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  cv::dilate(img,morpho_result_2,element);
  morph_size = 3;
  operation=3;
  element= cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  /// Apply the specified morphology operation
  cv::morphologyEx( morpho_result_2, morpho_result_3, operation, element );
  /******* edge detection *********/
  cv::Canny(morpho_result_3,edges,100,255,3);
  /******* find contours< *********/
  cv::findContours(edges,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  /******* find maximum area ********/
  int max_area_index=0;
  float area=0,max_area=0;
  for( int i = 0; i< contours.size(); i++ )
   {
     area=cv::contourArea(contours[i]);
     if (area>max_area){max_area_index=i;max_area=area;}
   }

  *contour=contours[max_area_index];
  *minRect=cv::minAreaRect(cv::Mat(contours[max_area_index]));
  /*draw=cv::Mat::zeros(edges.size(),CV_8UC3);
  cv::drawContours( original, contours, max_area_index, cv::Scalar(0,0,255), 2, 8, hierarchy, 3, cv::Point() );
  cv::drawContours( draw, contours, max_area_index, cv::Scalar(0,0,255), 2, 8, hierarchy, 3, cv::Point() );
  cv::Point2f rect_Points[4];
  minRect.points(rect_Points);*/
  return;

}
void crop_image_processor::start (){
  ros::spinOnce ();
}

int main(int argc, char **argv)
{
  std::setprecision (10);
  std::string name("crop_image_server");
  ros::init(argc, argv, "crop_image_server");
  //crop_image_processor  ip;
  crop_image_processor  ip(name);
  //ip.start ();
  ros::spin();

  return 0;
}
