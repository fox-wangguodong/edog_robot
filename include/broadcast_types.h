#ifndef BROADCAST_TYPES
#define BROADCAST_TYPES


#define StartSignal_TYPE 0      //开始比赛信号
#define StopSignal_TYPE 1       //停止比赛信号
#define OutSignal_TYPE 2        //罚下比赛信号
#define InSignal_TYPE 3        //上场比赛信号


#define RobotPosition_TYPE 100  //机器人位置信息
struct RobotPosition
{
    int number;//狗编号
    int x;//位置x
    int y;//位置y
    int angle;//朝向angle
};

#define BallPosition_TYPE 101   //足球位置信息
struct BallPosition
{
    int x;//位置x
    int y;//位置y
};

#define RobotInfo 102           //机器人自身信息
struct RobotInformation
{
    int number;//机器人编号
    char robotTeam[128];//机器人所在队伍
};

#endif // UDP_H
