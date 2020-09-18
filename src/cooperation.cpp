/*

协作通信节点

*/


#include <ros/ros.h>
#include "command_builder.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "edog_robot/command.h"
#include <time.h>
#include <math.h>
#include "edog_robot/status.h"
double X=4430,Y=1640;//球门中心位置
int robotNo = 3;
int myid = 2;
char feasiblecompare[3];
char robothelp = -1;
bool coopera = false;
static ros::Publisher cooperation_pub;// 用来发布协作信息
static ros::Publisher chatter_pub;
void construct_msg(Command &cmd)
{
    /*将cmd转换为msg*/
    edog_robot::command msg;
    for(int i=0;i<cmd.length;i++)
    {
        msg.data.push_back(cmd.data[i]);
    }
    msg.length = cmd.length;
    cooperation_pub.publish(msg);//发布消息
}


//注意这里的回调函数参数类型与消息类型一致
void cooperationCallback(const edog_robot::command& msg)
{
    ROS_INFO("askingforhelp\n");
    // edog_robot::command t_msg;
    // cooperation_pub.publish(t_msg);

    if(msg.data[0] == 1)//asking for help
    {
        ROS_INFO("askingforhelp\n");
        robothelp = msg.data[1];
        if(msg.data[5] == 1)//射门
        {
            ROS_INFO("1shoot\n");
            if(msg.data[6] == 1)//求助缘由
            {
                ROS_INFO("1zhedang\n");
                // //协作完成可行度评估
                // double Xb=3000,Yb=1000;
                // double obstr2=10,obstr12=10;
                // double feasible;
                // double a,b,c,L;
                // a=atan2(Yb-Y,Xb-X)*180/3.1415926;
                // a=fabs(a)-90;
                // a=a*100.0/90.0;
                
                // L=(Yb-Y)*(Yb-Y)+(Xb-X)*(Xb-X);
                // L=sqrt(L);
                // if(L<2000.0)
                // {
                //     b=(2000.0-L)*100/2000.0;
                // }
                // else b=0;
                
                // c=100-obstr2;
                // feasible=0.1*a+0.1*b+0.4*c+0.4*(100-obstr12);
                int feasible=50;
                //发布可行度评估结果
                Command cmd;
                co_operationCommandBuilder(cmd,(char)0x02,(char)0x02,(char)0x02,1234.2,1234.2,1.23,(char)feasible,-1,-1);
                construct_msg(cmd);
            }
        }
    }
    if(msg.data[0] == 2)//收到可行度评估
    {
        ROS_INFO("receive feasible\n");
        bool all = true;
        feasiblecompare[(int)msg.data[1]] = msg.data[14];
        //检查是否所有机器人都上传了可行度数据
        for(int i = 2;i<robotNo;i++)
        {
            if(feasiblecompare[i] == 0)
            {
                all = false;
            }
        }
        if(all == true)//各机器人都上传了自己的评估结果
        {
            ROS_INFO("receive all\n");
            //求最优评估结果
            char maxfeasible = feasiblecompare[0];
            int max = 0;//最优的机器人ID
            for(int i = 1; i<robotNo;i++)
            {
                if(feasiblecompare[i]>maxfeasible) 
                {
                    maxfeasible = feasiblecompare[i];
                    max = i;
                }
            }
            if(max == myid)//若自己是最优的
            {
                ROS_INFO("it's me\n");
                coopera = true;
            }
        }
    }
    if(msg.data[0] == 3 && coopera == true)//收到战术策划
    {
        if(robothelp == msg.data[1])//是否来自求助机器人
        {
            ROS_INFO("receive plan\n");
            ROS_INFO("confirm plan\n");//确认计划
            
            Command cmd;
            cmd.data[0]=(unsigned char)4;
            cmd.data[1]=(unsigned char)1;//方案可行
            cmd.length=12;
            construct_msg(cmd);            
            // if()//是否是发给自己的
            // {

                // float targetx = msg.data[1];
                // float targety = msg.data[];
                bool shoot = msg.data[23];
            // }
        }
    }
    if(msg.data[0] == 4 && coopera == true)//收到接球命令
    {
        if(msg.data[1]== 2)
        {
            edog_robot::status status_msg;
            status_msg.status = 0;
            //发布消息
            chatter_pub.publish(status_msg);
        }
    }
    // /*将ROS消息转为要发送的命令包*/
    // Command cmd;
    // for(int i=0;i<msg.length;i++)
    // {
    //     cmd.data[i] = msg.data[i];
    // }
    // cmd.length = msg.length;

    // /*发送命令*/
    // SendData(serialport_fd,cmd.data,cmd.length);
    // usleep(50000);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "cooperation1");
    //节点句柄
    ros::NodeHandle cn;
    //订阅chatter话题，环形缓冲队列长度为3，回调函数
    ros::Subscriber sub = cn.subscribe("askforhelp", 26, cooperationCallback);
    cooperation_pub = cn.advertise<edog_robot::command>("askforhelp", 26);//发布命令消息

    ros::NodeHandle n;
    chatter_pub = n.advertise<edog_robot::status>("status_control", 10);
    ros::spin();
}