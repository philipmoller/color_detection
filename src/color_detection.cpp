#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include <nav_msgs/Odometry.h>

using namespace cv;

static const std::string OPENCV_WINDOW = "Processed image";

int LowH = 1;
int LowS = 0;
int LowV = 0;
int HighH = 179;
int HighS = 255;
int HighV = 255;

bool anomaly = false;

class ImageConverter{
 ros::NodeHandle nh_;
 image_transport::ImageTransport it_;
 image_transport::Subscriber image_sub_;
 image_transport::Publisher image_pub_;

public:
 ImageConverter()
   : it_(nh_){
   image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
   image_pub_ = it_.advertise("/camera/image_processed", 1);

   cv::namedWindow(OPENCV_WINDOW);
 }

 ~ImageConverter(){
   cv::destroyWindow(OPENCV_WINDOW);
 }

 void imageCb(const sensor_msgs::ImageConstPtr& msg){
   cv_bridge::CvImagePtr cv_ptr;
   try{
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
   }
   catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   cv::Mat hsv_img, mask_img;
   cvtColor(cv_ptr->image, hsv_img, COLOR_RGB2HSV);
   cv::inRange(hsv_img, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), mask_img);

   cv::erode(mask_img, mask_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
   cv::dilate(mask_img, mask_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

   cv::dilate(mask_img, mask_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
   cv::erode(mask_img, mask_img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

   cv::imshow(OPENCV_WINDOW, mask_img);
   cv::waitKey(3);

   std::vector<std::vector<cv::Point> > contours;
   std::vector<Vec4i> hierarchy;
   cv::RNG rng(12345);
   cv::findContours(mask_img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  cv::Mat drawing = Mat::zeros(mask_img.size(), CV_8UC3);
   for( int i = 0; i< contours.size(); i++ ){
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
  }

  int largest_area = 40000; //Change value to adjust search size

  for(int i = 0; i< contours.size(); i++ ){
    double area = contourArea(contours[i],false);
    if(area>largest_area){
      //ROS_INFO("Mud detected!");
      anomaly = true;
    }
  }

   image_pub_.publish(cv_ptr->toImageMsg());
 }
};


ros::Subscriber odom_sub_;

void clbk_asd(const nav_msgs::Odometry::ConstPtr& asd){
  double x_pose = asd->pose.pose.position.x;
  double y_pose = asd->pose.pose.position.y;
  if(anomaly){
    ROS_INFO("Anomaly detected at: x: %f, y: %f",x_pose, y_pose);
    anomaly = false;
    return;
  }
}

int main(int argc, char** argv){

 cv::namedWindow("Control");
 cv::createTrackbar("LowerH:", "Control", &LowH, 179, NULL);
 cv::createTrackbar("LowerS:", "Control", &LowS, 255, NULL);
 cv::createTrackbar("LowerV:", "Control", &LowV, 255, NULL);
 cv::createTrackbar("UpperH:", "Control", &HighH, 179, NULL);
 cv::createTrackbar("UpperS:", "Control", &HighS, 255, NULL);
 cv::createTrackbar("UpperV:", "Control", &HighV, 255, NULL);

 ros::init(argc, argv, "color_detection");

 ros::NodeHandle n;
 ImageConverter ic;

 ros::spinOnce();

 odom_sub_ = n.subscribe("/odom", 1, clbk_asd);

 ros::spin();

 return 0;
}
