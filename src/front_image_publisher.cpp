/*

本节点从图像设备读取数据,然后通过消息发送给决策节点

*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "front_image_publisher");

	if(argc != 2)
	{
        ROS_INFO("Usage: front_image_publisher <video number> ; eg: front_image_publisher 0");
        return 1;
	}

    int dev_number = atoi(argv[1]); //char* 转 int
    cv::VideoCapture cap(dev_number);
    if(!cap.isOpened())
    {
        ROS_INFO("打开摄像头失败 dev_number = %d\n",dev_number);
        return 1;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);


	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/front_image/", 1);

	ros::Rate loop_rate(50);
	while (nh.ok())
	{
        cv::Mat srcImage;
        cap >> srcImage;//读取图像

        if(!srcImage.empty())
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
            pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
}
