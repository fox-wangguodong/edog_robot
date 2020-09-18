/*

本节点为决策节点，接收图像数据和状态数据，生成决策指令，发送消息给命令执行机构

*/

#include <ros/ros.h>
#include "edog_robot/command.h"
#include "edog_robot/status.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include "command_builder.h"
#include "broadcast_types.h"
#include "ImageUtils.h"


ImageUtils imageUtils;
ros::Publisher serialport_publisher;//串口指令发布
int STATUS = StopSignal_TYPE;// 当前机器人所处的状态
void Status_ControlCallback(const edog_robot::status &msg);//状态改变回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void onStartStatus();
void onOutStatus();
void onInStatus();
void onStopStatus();
void construct_msg(Command &cmd);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "policy");

    std::string data_path;
	ros::param::get("ImageUtils", data_path);

    if( ! imageUtils.loadData(data_path) ){
        ROS_INFO("load data from %s failed!",data_path.c_str());
        return -1;
    };//加载物体识别参数

    ros::NodeHandle nh;
    ros::Subscriber status_sub = nh.subscribe("status_control", 3, Status_ControlCallback);//接收状态消息
    serialport_publisher = nh.advertise<edog_robot::command>("command", 3);//发布串口消息
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);//接收图像消息

    ros::spin();
}


//运动状态的定义
enum SportStatus{
    SportStatus_Stand,
    SportStatus_HitBall,
    SportStatus_FindBall
};

//当前的运动状态
SportStatus current_SportStatus = SportStatus_HitBall; // 默认是撞球状态


float neck = 30;//脖子俯仰的角度值
float angle = 0;//头部左右旋转的角度值
float circle_speed = 0;//自转的速度
float forword_speed = 0;//前进速度s
float side_speed = 0;//横移速度


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat srcImage = cv_bridge::toCvShare(msg, "bgr8")->image;//将jpg图像解压后转为Mat数据类型

/*此处因为需要调试，所以注释掉了，如果正常比赛，需要取消该部分注释*/
/*
        if(StartSignal_TYPE != STATUS){
            return;
        }
*/

        if(current_SportStatus == SportStatus_Stand)
        {
            Command cmd;
            MotorControlCommandBuilder(cmd,
                angle,
                -10,
                Stand,// 步态
                0,   // 前进速度
                0,   // 横移速度
                0, // 自转速度
                144, // 站立高度
                0,   // 躯干俯仰角
                0);  // 横滚角
            construct_msg(cmd);
        }
        if(current_SportStatus == SportStatus_HitBall)
        {
            cv::Rect current_ball;
            int ret = imageUtils.CheckRedBall(srcImage,current_ball);
            if(ret == 0)
            {
                // current_SportStatus = SportStatus_Stand;
            }
            else
            {
                int width = srcImage.cols; // 图像的宽
                int height = srcImage.rows; // 图像的高
                float x = current_ball.x+current_ball.width/2;//小球的x坐标
                float y = current_ball.y+current_ball.height/2;//小球的y坐标

                //小球距图像中心的像素距离(偏移量) * 比例系数(由偏移量计算出比例系数)
                angle -= (x - width/2) * ((abs(x - width/2) / 320.0) * 0.02);
                if(angle > 70) angle = 70;//防止输出值超过范围
                if(angle < -70) angle = -70;

                circle_speed = angle/70.0 * 0.6;
                forword_speed = (300 - y)/300 * 0.3;

                ROS_INFO("angle=%f,forword_speed=%f,circle_speed=%f",angle,forword_speed,circle_speed);

                Command cmd;
                MotorControlCommandBuilder(cmd,
                    angle,
                    -10,
                    FlyingTrot,// 步态
                    forword_speed,   // 前进速度
                    0,   // 横移速度
                    circle_speed, // 自转速度
                    144, // 站立高度
                    0,   // 躯干俯仰角
                    0);  // 横滚角
                construct_msg(cmd);

                if( fabs(angle) < 3 &&
                    fabs(forword_speed) <= 0.02 &&
                    fabs(circle_speed) < 0.02 ) // 满足条件,进入姿态矫正状态
                {
                    sleep(1);
                    Command cmd;
                    MotorControlCommandBuilder(cmd,
                        angle,
                        -10,
                        FlyingTrot,// 步态
                        0.3,   // 前进速度
                        0,   // 横移速度
                        0, // 自转速度
                        144, // 站立高度
                        0,   // 躯干俯仰角
                        0);  // 横滚角
                    construct_msg(cmd);
                    sleep(1);
                    MotorControlCommandBuilder(cmd,
                        angle,
                        -10,
                        FlyingTrot,// 步态
                        0,   // 前进速度
                        0,   // 横移速度
                        0, // 自转速度
                        144, // 站立高度
                        0,   // 躯干俯仰角
                        0);  // 横滚角
                    construct_msg(cmd);
                    sleep(1);
                    // current_SportStatus = SportStatus_Stand;
                }
            }
        }
        else if(current_SportStatus == SportStatus_FindBall)
        {
            cv::Rect current_ball;
            int ret = imageUtils.CheckRedBall(srcImage,current_ball);

            if(ret == 0) // 若没有找到球就自转
            {
                Command cmd;
                MotorControlCommandBuilder(cmd,
                    angle,
                    -10,
                    FlyingTrot,// 步态
                    0,   // 前进速度
                    0,   // 横移速度
                    0.2, // 自转速度
                    144, // 站立高度
                    0,   // 躯干俯仰角
                    0);  // 横滚角
                construct_msg(cmd);
            }
            else // 找到球之后开始对正球
            {
                int width = srcImage.cols; // 图像的宽
                int height = srcImage.rows; // 图像的高
                float x = current_ball.x+current_ball.width/2;//小球的x坐标
                float y = current_ball.y+current_ball.height/2;//小球的y坐标

                //小球距图像中心的像素距离(偏移量) * 比例系数(由偏移量计算出比例系数)
                angle -= (x - width/2) * ((abs(x - width/2) / 320.0) * 0.02);
                if(angle > 70) angle = 70;//防止输出值超过范围
                if(angle < -70) angle = -70;

                circle_speed = angle/70.0 * 0.6;

                ROS_INFO("angle=%f,circle_speed=%f",angle,circle_speed);

                Command cmd;
                MotorControlCommandBuilder(cmd,
                    angle,
                    -10,
                    FlyingTrot,// 步态
                    0,   // 前进速度
                    0,   // 横移速度
                    circle_speed, // 自转速度
                    144, // 站立高度
                    0,   // 躯干俯仰角
                    0);  // 横滚角
                construct_msg(cmd);
                
                if(circle_speed < 0.02 && abs(angle) < 3)
                {
                    current_SportStatus = SportStatus_Stand;
                }
            }
        }
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void construct_msg(Command &cmd)
{
    /*将cmd转换为msg*/
    edog_robot::command msg;
    for(int i=0;i<cmd.length;i++)
    {
        msg.data.push_back(cmd.data[i]);
    }
    msg.length = cmd.length;
    serialport_publisher.publish(msg);//发布消息
}

void Status_ControlCallback(const edog_robot::status &msg)
{
    switch(msg.status)
    {
    case StartSignal_TYPE:
        ROS_INFO("start");
        STATUS = StartSignal_TYPE;
        onStartStatus();
        break;
    case StopSignal_TYPE:
        ROS_INFO("stop");
        STATUS = StopSignal_TYPE;
        onStopStatus();
        break;
    case OutSignal_TYPE:
        ROS_INFO("out");
        STATUS = OutSignal_TYPE;
        onOutStatus();
        break;
    case InSignal_TYPE:
        ROS_INFO("in");
        STATUS = InSignal_TYPE;
        onInStatus();
        break;
    }
}

//当前处于开始比赛状态
void onStartStatus()
{
    Command cmd;
    MotorControlCommandBuilder(cmd,
                0,
                -10,
                Trot,// 步态
                0,   // 前进速度
                0,   // 横移速度
                0,   // 自转速度
                144, // 站立高度
                0,   // 躯干俯仰角
                0);  // 横滚角
    construct_msg(cmd);
}

//当前处于被罚出场地状态
void onOutStatus()
{
    Command cmd;
    MotorControlCommandBuilder(cmd,
                0,
                -10,
                Trot,// 步态
                -0.2,   // 前进速度
                0,   // 横移速度
                0,   // 自转速度
                144, // 站立高度
                0,   // 躯干俯仰角
                0);  // 横滚角
    construct_msg(cmd);
}

//当前处于重新进入场地状态
void onInStatus()
{
    Command cmd;
    MotorControlCommandBuilder(cmd,
                0,
                -10,
                Trot,// 步态
                0.2,   // 前进速度
                0,   // 横移速度
                0,   // 自转速度
                144, // 站立高度
                0,   // 躯干俯仰角
                0);  // 横滚角
    construct_msg(cmd);
}

//当前处于停止状态
void onStopStatus()
{
    Command cmd;
    MotorControlCommandBuilder(cmd,
                0,
                -10,
                Stand,// 步态
                0,   // 前进速度
                0,   // 横移速度
                0,   // 自转速度
                144, // 站立高度
                0,   // 躯干俯仰角
                0);  // 横滚角
    construct_msg(cmd);
}
