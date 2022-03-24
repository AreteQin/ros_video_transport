#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

cv::Mat guidedFilter(const cv::Mat &I, const cv::Mat &p, int r, float eps)
{
  int rows = I.rows;
  int cols = I.cols;
  cv::Mat mean_I, mean_p;
  boxFilter(I, mean_I, CV_8UC1, cv::Size(2 * r + 1, 2 * r + 1), cv::Point(-1, -1));
  boxFilter(p, mean_p, CV_8UC1, cv::Size(2 * r + 1, 2 * r + 1), cv::Point(-1, -1));
  cv::Mat Ip = I.mul(p);
  cv::Mat mean_Ip;
  boxFilter(Ip, mean_Ip, CV_8UC1, cv::Size(2 * r + 1, 2 * r + 1), cv::Point(-1, -1));
  cv::Mat cov_Ip = mean_Ip - mean_I.mul(mean_p);
  cv::Mat mean_II;
  boxFilter(I.mul(I), mean_II, CV_8UC1, cv::Size(2 * r + 1, 2 * r + 1), cv::Point(-1, -1));
  cv::Mat var_I = mean_II - mean_I.mul(mean_I);
  cv::Mat a = cov_Ip / (var_I + eps);
  cv::Mat b = mean_p - a.mul(mean_I);
  cv::Mat mean_a, mean_b;
  boxFilter(a, mean_a, CV_8UC1, cv::Size(2 * r + 1, 2 * r + 1), cv::Point(-1, -1));
  boxFilter(b, mean_b, CV_8UC1, cv::Size(2 * r + 1, 2 * r + 1), cv::Point(-1, -1));
  cv::Mat q = mean_a.mul(I) + mean_b;
  return q;
}

class ImageReceiver
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::string OPENCV_WINDOW;

public:
  ImageReceiver(const std::string &topic_name)
      : it_(nh_)
  {
    OPENCV_WINDOW = topic_name;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(OPENCV_WINDOW, 1,
                               &ImageReceiver::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageReceiver()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    if (OPENCV_WINDOW == "/camera/color/image_raw")
    {
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception &e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
    else
    {
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception &e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      cv::imshow("Original depth", cv_ptr->image);
      cv_ptr->image = guidedFilter(cv_ptr->image, cv_ptr->image, 10, 0.8f);
    }

    //cv_ptr->image = guidedFilter(cv_ptr->image, cv_ptr->image, 5, 0.5f);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgbd_receiver");
  ImageReceiver color("/camera/color/image_raw");
  ImageReceiver depth("/camera/depth/image_rect_raw");
  ros::spin();
  return 0;
}