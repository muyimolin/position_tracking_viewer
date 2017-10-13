#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

// Global Variables
cv::Mat rgbImg, hsvImg;
cv::Mat mask;
float hsvMin[] = {180,255,255};
float hsvMax[] = {0,0,0};

int iLowH = 0;
int iHighH = 179;

int iLowS = 0; 
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

void ros2cv(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
{
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
	pCvImage->image.copyTo(image);
}

void applyHsvMask(int x, int y)
{
  // cv::Vec3b rtv = rgbImg.at<cv::Vec3b>(x,y);
  // cv::Mat rgbTest(1,1,CV_8UC3,cv::Scalar(rtv.val[0], rtv.val[1], rtv.val[2]));

  // cv::Mat hsvTest;
  // cv::cvtColor(rgbTest, hsvTest, CV_BGR2HSV);
  // cv::Vec3b hsvVec = hsvTest.at<cv::Vec3b>(0,0);
  
  cv::Vec3b hsvVec = hsvImg.at<cv::Vec3b>(x,y);

  for (int i = 0; i < 3; i++)
  {
    float value = (float)hsvVec.val[i];
    if (value < hsvMin[i])
    {
      hsvMin[i] = value;
    }
    if (value > hsvMax[i])
    {
      hsvMax[i] = value;
    }

    // cout << "HSV [" << i << "]" << (float)hsvVec.val[i] << endl;
  }

  cout << "HSV min: " << hsvMin[0] << " , " << hsvMin[1] << " , " << hsvMin[2] << endl;
  cout << "HSV max: " << hsvMax[0] << " , " << hsvMax[1] << " , " << hsvMax[2] << endl;
  cv::inRange(hsvImg, cv::Scalar(hsvMin[0], hsvMin[1], hsvMin[2]), cv::Scalar(hsvMax[0], hsvMax[1], hsvMax[2]), mask);

  cv::namedWindow( "HSV masked window", cv::WINDOW_AUTOSIZE);     
  cv::imshow( "HSV masked window", mask); 
  cv::waitKey(10);
}

void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == cv::EVENT_LBUTTONDOWN )
    {
      // cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
      applyHsvMask(x,y);  
    }
}

void trackbar()
{
  cv::Mat mask_rgb;
  cv::Mat hsv_masked;
  cv::inRange(hsvImg, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), mask);

  cvtColor(mask,mask_rgb,CV_GRAY2BGR);
  cv::bitwise_and(rgbImg, mask_rgb, hsv_masked);

  // cv::namedWindow( "HSV masked window", cv::WINDOW_AUTOSIZE);     
  cv::imshow( "HSV masked window", hsv_masked); 
  cv::waitKey(10);
}

void imageCallback(const sensor_msgs::Image::ConstPtr& imgMsg)
{
  // ROS_INFO("getting image");
  ros2cv(imgMsg,rgbImg);
  cv::resize(rgbImg, rgbImg, cv::Size(), 0.5, 0.5, CV_INTER_CUBIC);
  // rgbImg = cv::imread("/home/ulkesh/catkin_ws/src/kinect2_position_tracking/script/test01.JPG", CV_LOAD_IMAGE_COLOR);
  // cv::namedWindow( "rgb window", cv::WINDOW_AUTOSIZE);    
  // cv::imshow( "rgb window", rgbImg); 
  // cv::waitKey(10);

  // GaussianBlur(rgbImg, rgbImg, cv::Size(15,15), 0, 0);

  cvtColor(rgbImg, hsvImg, CV_BGR2HSV);
  
  // cv::namedWindow( "hsv window", cv::WINDOW_AUTOSIZE);    
  // cv::imshow( "hsv window", hsvImg); 
  // cv::waitKey(10);

  trackbar();

  // cv::setMouseCallback("hsv window", mouseCallBackFunc, NULL);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hsv_threshhold");
  
  ros::NodeHandle n;

  cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);
  
  ros::Subscriber sub = n.subscribe("/kinect2/hd/image_color", 10, imageCallback);

  ros::spin();

  return 0;
}