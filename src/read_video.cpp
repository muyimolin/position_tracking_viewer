#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "read_video");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("virtual_camera/ros_img", 1);
  // ros::Publisher chatter_pub = n.advertise<sensor_msgs::Image>("virtual_camera/ros_img", 1000);
  ros::Rate loop_rate(5);

  VideoCapture cap("/home/ulkesh/Downloads/test008.MOV"); // open the video file for reading

  if ( !cap.isOpened() )  // if not success, exit program
  {
   cout << "Cannot open the video file" << endl;
   return -1;
  }

  //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

  double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

   cout << "Frame per seconds : " << fps << endl;

  // namedWindow("MyVideo",WINDOW_AUTOSIZE); //create a window called "MyVideo"

  Mat frame;
  sensor_msgs::ImagePtr imgMsg;
  Size size(752,480);
  int i = 0;

  while (ros::ok())
  {

    bool bSuccess = cap.read(frame); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read the frame from video file" << endl;
      break;
    }

    //resize(frame, frame, size, INTER_CUBIC);
    cvtColor(frame,frame,CV_BGR2GRAY);
    // imwrite("/home/ulkesh/Downloads/my_table")
    // cout << i << endl;
    cout << "Width : " << frame.size().width << endl;
    cout << "Height: " << frame.size().height << endl;
    imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame).toImageMsg();
    // cout << "ros_img_size" << imgMsg->height << endl;
    // cout << "ros_img_size" << imgMsg->width << endl;

    pub.publish(imgMsg);
    cv::waitKey(1);

    i += 1;

    // imshow("MyVideo", frame); //show the frame in "MyVideo" window

    // if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
    // {
    //   cout << "esc key is pressed by user" << endl; 
    //   break; 
    // }


    // chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}