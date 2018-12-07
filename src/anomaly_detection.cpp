#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

static const std::string OPENCV_WINDOW = "Processed image";

int LowH = 0;
int LowS = 0;
int LowV = 0;
int HighH = 179;
int HighS = 255;
int HighV = 255;

class ImageConverter
{
 ros::NodeHandle nh_;
 image_transport::ImageTransport it_;
 image_transport::Subscriber image_sub_;
 image_transport::Publisher image_pub_;

public:
 ImageConverter()
   : it_(nh_)
 {
   image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
   image_pub_ = it_.advertise("/camera/image_processed", 1);

   cv::namedWindow(OPENCV_WINDOW);
 }

 ~ImageConverter()
 {
   cv::destroyWindow(OPENCV_WINDOW);
 }

 void imageCb(const sensor_msgs::ImageConstPtr& msg)
 {
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   cv::Mat hsv_img, mask_img;
   cvtColor(cv_ptr->image, hsv_img, COLOR_RGB2HSV);
   cv::inRange(hsv_img, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), mask_img);

   cv::imshow(OPENCV_WINDOW, mask_img);
   cv::waitKey(3);

   image_pub_.publish(cv_ptr->toImageMsg());
 }
};

int main(int argc, char** argv)
{
 cv::namedWindow("Control");
 cv::createTrackbar("LowerH:", "Control", &LowH, 179, NULL);
 cv::createTrackbar("LowerS:", "Control", &LowS, 179, NULL);
 cv::createTrackbar("LowerV:", "Control", &LowV, 179, NULL);
 cv::createTrackbar("UpperH:", "Control", &HighH, 179, NULL);
 cv::createTrackbar("UpperS:", "Control", &HighS, 179, NULL);
 cv::createTrackbar("UpperV:", "Control", &HighV, 179, NULL);

 ros::init(argc, argv, "anomaly_detection");
 ImageConverter ic;

 ros::spin();
 return 0;
}
