#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat

        cv::imshow("below_image",image);
        cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'Mat'.", msg->format.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "below_image_subscriber");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/below_image/compressed", 1, imageCallback);
    
    ros::spin();
}
