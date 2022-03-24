#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_cam_node");
    ros::NodeHandle nh;

    cv::VideoCapture capture(0); //read the video from camera
    if (!capture.isOpened()) {
        ROS_ERROR_STREAM("Failed to open video device\n");
        ros::shutdown();
    }

    //image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_color = it.advertise("/camera/color/image_raw", 1);
    image_transport::Publisher pub_depth = it.advertise("/camera/depth/image_rect_raw", 1);

    cv::Mat image, depth;

    while (ros::ok()) {
        capture >> image; //load
        //printf("ros is ok\n");
        //cv::imshow("Color", image);
        //cv::waitKey(3);
        if (image.empty()) {
            ROS_ERROR_STREAM("Failed to capture image!");
            ros::shutdown();
        }
        //将图像从cv::Mat类型转化成sensor_msgs/Image类型并发布
        pub_color.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
        //cv::Mat depth = cv::Mat::zeros(image.size(),CV_32FC1);
        //convert color image to gray image, channel is changed from 3 to 1
        cv::cvtColor(image, depth, cv::COLOR_BGR2GRAY);
        //cv::imshow("Depth", depth);
        //depth.convertTo(depth, CV_8U, 255.0, 0);
        pub_depth.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", depth).toImageMsg());
        //将图像从sensor_msgs/Image类型转化成cv::Mat类型
        /*
        cv_bridge可以有选择的对颜色和深度信息进行转化。为了使用指定的特征编码，就有下面集中的编码形式：

        mono8:  CV_8UC1， 灰度图像
        mono16: CV_16UC1,16位灰度图像
        bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序
        rgb8: CV_8UC3,带有颜色信息并且颜色的顺序是RGB顺序
        bgra8: CV_8UC4, BGR的彩色图像，并且带alpha通道
        rgba8: CV_8UC4,CV，RGB彩色图像，并且带alpha通道
        */
    }
    
    ros::spin();
}
