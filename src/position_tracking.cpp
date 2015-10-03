/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <typeinfo>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>
#include <tf/transform_broadcaster.h>

using namespace std;

double detected_pos[3] = {0.0, 0.0, 0.0};


visualization_msgs::Marker marker;
ros::Publisher marker_pub;

class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  // color segementaion variables

  double centroid_xyz[3];
  std::string sample_path;
  float color_idx[3];
  float color_MIN[3];
  float color_MAX[3];
  cv::Mat hist_2D;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(IMAGE)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

  void run(const Mode mode)
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
    this->mode = mode;
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }

    spinner.start();

    // initialize for color segmentation variables

    this->sample_path = "/home/motion/ros_ws/src/position_tracking/scripts/sample/sample001.jpg";
    
    cv::Mat img, hsv_image;
    img = cv::imread(this->sample_path, CV_LOAD_IMAGE_COLOR);
    cvtColor(img, hsv_image, CV_BGR2HSV);
    // cv::namedWindow( "hsv window", cv::WINDOW_AUTOSIZE);    
    // cv::imshow( "hsv window", hsv_image); 
    // cv::waitKey(10);


    int hsv_bin[] = {180, 255, 255};
    float hsv_range[] = {180, 255, 255};
    float hsv_threshold[] = {5, 75, 75};


    // generating 2D histograph

    int hbins = 180, sbins = 255;
    int histSize[] = {hbins, sbins};
    int channels[] = {0, 1};
    
    float hranges[] = { 0, 180};
    float sranges[] = { 0, 255};
    // float vranges[] = { 0, 255};

    const float* ranges[] = { hranges, sranges};
    cv::MatND hist2D; 

    cv::calcHist( &hsv_image, 1, channels, cv::Mat(), // do not use mask
             hist2D, 2, histSize, ranges,
             true, // the histogram is uniform
             false );

    cv::normalize(hist2D, hist2D, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());  // only works for 2D
    this->hist_2D = hist2D;

    // cout << "type of hist is " << typeid(hist).name() << endl;

    // cout << "dims of hist is " << hist.dims << endl;

    // generating 1D histograph for each dimension

    double min, max;
    cv::Point min_loc, max_loc;

    cv::MatND hist_temp;
    cv::Mat hist_temp_reduce;

    for(int i = 0; i < 3; i++)
    {
      
      int bin_temp[] = { hsv_bin[i] };
      int channel_temp[] = {i};
      float range_temp[] = {0, hsv_range[i]};
      const float* t_ranges[] = {range_temp};


      cv::calcHist( &hsv_image, 1, channel_temp, cv::Mat(), // do not use mask
             hist_temp, 1, bin_temp, t_ranges,
             true, // the histogram is uniform
             false);

      cv::reduce(hist_temp, hist_temp_reduce, i, CV_REDUCE_SUM);
      cv::minMaxLoc(hist_temp, &min, &max, &min_loc, &max_loc);
      cout << "Dim " << i << ": max value: "<< max << ", min value: " << min << ", max_ind: " << max_loc << ", min_ind: " << min_loc << endl;
      this->color_idx[i] = max_loc.y;
    }

    for(int i = 0 ; i<3; i++)
    {
      this->color_MAX[i] = this->color_idx[i] + hsv_threshold[i];
      this->color_MIN[i] = this->color_idx[i] - hsv_threshold[i];
      cout<< "color_idx["<< i << "] = " << color_idx[i] << endl;
      cout<< "color_range ["<< i << "] = [" << color_MIN[i] << ", " << color_MAX[i] << "]" << endl;

    }
    

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud)
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    switch(mode)
    {
    case CLOUD:
      cloudViewer();
      break;
    case IMAGE:
      imageViewer();
      break;
    case BOTH:
      imageViewerThread = std::thread(&Receiver::imageViewer, this);
      cloudViewer();
      break;
    }
  }

  void stop()
  {
    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }
  }

  void poseCallback(const visualization_msgs::Marker marker)
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z) );
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect2_link", "marker_pos"));
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
  }

  void imageViewer()
  {
    cv::Mat color, depth, depthDisp, combined;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::namedWindow("Image Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {
      if(updateImage)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateImage = false;
        lock.unlock();

        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if(elapsed >= 1.0)
        {
          fps = frameCount / elapsed;
          oss.str("");
          oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
          start = now;
          frameCount = 0;
        }

        dispDepth(depth, depthDisp, 12000.0f);
        combine(color, depthDisp, combined);
        //combined = color;

        cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
        cv::imshow("Image Viewer", combined);
      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        if(mode == IMAGE)
        {
          createCloud(depth, color, cloud);
          saveCloudAndImages(cloud, color, depth, depthDisp);
        }
        else
        {
          save = true;
        }
        break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }

  void cloudViewer()
  {
    cv::Mat color, depth;
    // pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);


    // visualizer->addPointCloud(cloud, cloudName);
    // visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    // visualizer->initCameraParameters();
    // visualizer->setBackgroundColor(0, 0, 0);
    // visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    // visualizer->setSize(color.cols, color.rows);
    // visualizer->setShowFPS(true);
    // visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    // visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        colorSegmentation(color, cloud);
        // update markers
        marker.header.stamp = ros::Time::now();
        // printf("sec = %3.2f", (double) (marker.header.stamp.sec + ((double) marker.header.stamp.nsec * 10e-9)));
        printf("sec = %d, nsec = %d,  ", marker.header.stamp.sec, marker.header.stamp.nsec);
        marker.pose.position.x = detected_pos[0];
        marker.pose.position.y = detected_pos[1];
        marker.pose.position.z = detected_pos[2];

        marker_pub.publish(marker);
        poseCallback(marker); 

        // visualizer->updatePointCloud(cloud, cloudName);
      }
      // if(save)
      // {
      //   save = false;
      //   cv::Mat depthDisp;
      //   dispDepth(depth, depthDisp, 12000.0f);
      //   saveCloudAndImages(cloud, color, depth, depthDisp);
      // }
      // visualizer->spinOnce(10);
    }
    // visualizer->close();
  }

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        save = true;
        break;
      }
    }
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }

  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
    out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

    #pragma omp parallel for
    for(int r = 0; r < inC.rows; ++r)
    {
      const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
       *itD = inD.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
        itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
        itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
      }
    }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(isnan(depthValue) || depthValue <= 0.001)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
  }

  void colorSegmentation(const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    // printf("this is for color segmentation \n");
    cv::Mat color_hsv, mask, mask_rgb, hsv_masked;
    cvtColor(color, color_hsv, CV_BGR2HSV);
    // cout << "creating mask ..." << endl;
    cv::inRange(color_hsv, cv::Scalar(this->color_MIN[0], this->color_MIN[1], this->color_MIN[2]), cv::Scalar(this->color_MAX[0], this->color_MAX[1], this->color_MAX[2]), mask);
    // cout << "doing bitwise and ..." << endl;

    cvtColor(mask,mask_rgb,CV_GRAY2BGR);
    cv::bitwise_and(color, mask_rgb, hsv_masked);

    cv::namedWindow( "HSV masked window", cv::WINDOW_AUTOSIZE);    
    cv::imshow( "HSV masked window", hsv_masked); 
    cv::waitKey(10);
    
    // int counter = 0;
    double centroid_xyz[] = {0.0, 0.0, 0.0};
    vector< double > detect_pos_arr[3];
    // vector< double > detect_pos_arr_y;
    // vector< double > detect_pos_arr_z;

    // cout << "type of mask = " << mask.type() <<endl;


    int mask_pix = 0;
    double conf = 1.0;
    double vect_min = 0.0;
    double vect_max = 0.0;
    for(int r = 0; r < mask.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * mask.cols];

      for(size_t c = 0; c < (size_t)mask.cols; ++c, ++itP)
      {
        mask_pix = (int) mask.at<uchar>(r, c);
        if(mask_pix == 255)
        {
          if(itP->x != itP->x || itP->y != itP->y || itP->z != itP->z)
          {

          }
          else
          {
            // centroid_xyz[0] = centroid_xyz[0] + itP->x;
            // centroid_xyz[1] = centroid_xyz[1] + itP->y;
            // centroid_xyz[2] = centroid_xyz[2] + itP->z;
            // counter = counter + 1.0;

            detect_pos_arr[0].push_back(itP->x);
            detect_pos_arr[1].push_back(itP->y);
            detect_pos_arr[2].push_back(itP->z);

            // printf("%f, %f, %f \n", itP->x, itP->y, itP->z);

          }
        }

      }

    }
    for(int i = 0; i<3; i++)
    {
      // centroid_xyz[i] = centroid_xyz[i]/counter;
      // detected_pos[i] = centroid_xyz[i];

      double sum = std::accumulate(detect_pos_arr[i].begin(), detect_pos_arr[i].end(), 0.0);
      double mean = sum / detect_pos_arr[i].size();

      double sq_sum = std::inner_product(detect_pos_arr[i].begin(), detect_pos_arr[i].end(), detect_pos_arr[i].begin(), 0.0);
      double stdev = std::sqrt(sq_sum / detect_pos_arr[i].size() - mean * mean);

      vect_min = mean - conf * stdev;
      vect_max = mean + conf * stdev;
      int counter = 0;

      for (vector<double>::iterator it = detect_pos_arr[i].begin() ; it != detect_pos_arr[i].end(); ++it)
      {

        if(*it >= vect_min && *it <= vect_max)
        {
          centroid_xyz[i] = centroid_xyz[i] + *it;
          counter = counter + 1;
        }
      }

      centroid_xyz[i] = centroid_xyz[i]/counter;
      detected_pos[i] = centroid_xyz[i];

    }



    printf("detected position = [%f, %f, %f] \n", centroid_xyz[0], centroid_xyz[1], centroid_xyz[2]);



  }

  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
    oss.str("");
    oss << "./" << std::setfill('0') << std::setw(4) << frame;
    const std::string baseName = oss.str();
    const std::string cloudName = baseName + "_cloud.pcd";
    const std::string colorName = baseName + "_color.jpg";
    const std::string depthName = baseName + "_depth.png";
    const std::string depthColoredName = baseName + "_depth_colored.png";

    std::cout << "saving cloud: " << cloudName << std::endl;
    writer.writeBinary(cloudName, *cloud);
    std::cout << "saving color: " << colorName << std::endl;
    cv::imwrite(colorName, color, params);
    std::cout << "saving depth: " << depthName << std::endl;
    cv::imwrite(depthName, depth, params);
    std::cout << "saving depth: " << depthColoredName << std::endl;
    cv::imwrite(depthColoredName, depthColored, params);
    std::cout << "saving complete!" << std::endl;
    ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }
};

void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  name: 'any string' equals to the kinect2_bridge topic base name" << std::endl
            << "  mode: 'qhd', 'hd', 'sd' or 'ir'" << std::endl
            << "  visualization: 'image', 'cloud' or 'both'" << std::endl
            << "  options:" << std::endl
            << "    'compressed' use compressed instead of raw topics" << std::endl
            << "    'approx' use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
  

  ros::init(argc, argv, "detect_position");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("Marker_glove", 10);

  marker.header.frame_id = "/kinect2_link";
  marker.header.stamp = ros::Time::now();

  marker.ns = "detect_position";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = detected_pos[0];
  marker.pose.position.y = detected_pos[1];
  marker.pose.position.z = detected_pos[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();


  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }


  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  bool useExact = true;
  bool useCompressed = false;
  Receiver::Mode mode = Receiver::CLOUD;

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else if(param == "qhd")
    {
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "approx")
    {
      useExact = false;
    }

    else if(param == "compressed")
    {
      useCompressed = true;
    }
    else if(param == "image")
    {
      mode = Receiver::IMAGE;
    }
    else if(param == "cloud")
    {
      mode = Receiver::CLOUD;
    }
    else if(param == "both")
    {
      mode = Receiver::BOTH;
    }
    else
    {
      ns = param;
    }
  }

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;



  Receiver receiver(topicColor, topicDepth, useExact, useCompressed);



  std::cout << "starting receiver..." << std::endl;
  receiver.run(mode);

  ros::shutdown();
  return 0;
}
