/*

指令生成类，通过相应的指令参数，生成下位机的指令数据

*/

#include "command_builder.h"

void disgussBuilder(Command &cmd,
                    unsigned char commondtype,
                    unsigned char robotid,
                    unsigned char corobotid,
					float locationx,
					float locationy,
					float oriatation,
					float targetx,
					float targety,
					unsigned char shoot)
{
    cmd.data[0]=commondtype;
    cmd.data[1]=robotid;
    cmd.data[2]=corobotid;

    char* pAngle;
    pAngle = (char*)&locationx;
    for(int i=0;i<4;i++)
    {
        cmd.data[6+i] = *pAngle;
        pAngle++;
    }

    pAngle = (char*)&locationy;
    for(int i=0;i<4;i++)
    {
        cmd.data[6+i] = *pAngle;
        pAngle++;
    }

    pAngle = (char*)&oriatation;
    for(int i=0;i<4;i++)
    {
        cmd.data[6+i] = *pAngle;
        pAngle++;
    }

    pAngle = (char*)&targetx;
    for(int i=0;i<4;i++)
    {
        cmd.data[6+i] = *pAngle;
        pAngle++;
    }

    pAngle = (char*)&targety;
    for(int i=0;i<4;i++)
    {
        cmd.data[6+i] = *pAngle;
        pAngle++;
    }
    cmd.data[23] = shoot;
    cmd.length = 24;
}

void co_operationCommandBuilder(Command &cmd,
                            unsigned char commondtype,//命令类型
							unsigned char robotid,//机器人编号
                            unsigned char corobotid,//机器人编号
							float locationx,
							float locationy,
							float oriatation,
							unsigned char parameter1,
							unsigned char parameter2,
							unsigned char parameter3)
{
    cmd.data[0] = commondtype;
    cmd.data[1] = robotid;

    char* plocationx = (char*)&locationx;
    for(int i=0;i<4;i++)
    {
        cmd.data[2+i] = *plocationx;
        plocationx++;
    }

    char* plocationy = (char*)&locationy;
    for(int i=0;i<4;i++)
    {
        cmd.data[6+i] = *plocationy;
        plocationy++;
    }

    char* poriatation = (char*)&oriatation;
    for(int i=0;i<4;i++)
    {
        cmd.data[10+i] = *poriatation;
        poriatation++;
    }
    
    cmd.data[14] = parameter1;
    cmd.data[15] = parameter2;
    cmd.data[16] = parameter3;
    cmd.length = 17;
}
void CallActionCommandBuilder(Command &cmd,enum Action action)
{
    cmd.data[0] = (char)0x8f;
    cmd.data[1] = (char)0x51;
    cmd.data[2] = (char)0x01;
    cmd.data[3] = (char)action;

    unsigned char checkSum = 0;
    for(int i=1;i<=3;i++)
    {
        checkSum += cmd.data[i];
    }
    cmd.data[4] = (char)checkSum;
    cmd.data[5] = (char)0xff;
    cmd.length = 6;
}


void MotorControlCommandBuilder(Command &cmd,
                            float headAngle,    // 头部转动的角度
                            float neckAngle,    // 颈部转动的角度
                            enum Gait gait,     // 步态
                            float forward,      // 前进速度
                            float sidespeed,    // 横移速度
                            float circlespeed,  // 自转速度
                            unsigned char standheight,  // 站立高度
                            float pitchangle,   // 躯干俯仰角
                            float rollangle)    // 躯干横滚角
{
    cmd.data[0] = (char)0x8f;
    cmd.data[1] = (char)0x29;
    cmd.data[2] = (char)0x1E;

    if(headAngle >=70) headAngle = 70;
    if(headAngle <=-70) headAngle = -70;
    if(headAngle>=0 && headAngle <=0.3) headAngle = 0.3;
    if(headAngle<=0 && headAngle >=-0.3) headAngle = -0.3;
    char* pHeadAngle = (char*)&headAngle;
    for(int i=0;i<4;i++)
    {
        cmd.data[3+i] = *pHeadAngle;
        pHeadAngle++;
    }

    if(neckAngle >=40) neckAngle = 40;
    if(neckAngle <=-10) neckAngle = -10;
    if(neckAngle>=0 && neckAngle <=0.3) neckAngle = 0.3;
    if(neckAngle<=0 && neckAngle >=-0.3) neckAngle = -0.3;
    char* pNeckAngle = (char*)&neckAngle;
    for(int i=0;i<4;i++)
    {
        cmd.data[7+i] = *pNeckAngle;
        pNeckAngle++;
    }

    cmd.data[11] = (char)gait;

    char* pForward = (char*)&forward;
    for(int i=0;i<4;i++)
    {
        cmd.data[12+i] = *pForward;
        pForward++;
    }

    char* pSideSpeed = (char*)&sidespeed;
    for(int i=0;i<4;i++)
    {
        cmd.data[16+i] = *pSideSpeed;
        pSideSpeed++;
    }

    char* pCircleSpeed = (char*)&circlespeed;
    for(int i=0;i<4;i++)
    {
        cmd.data[20+i] = *pCircleSpeed;
        pCircleSpeed++;
    }

    cmd.data[24] = (char)standheight;

    char* pPitchAngle = (char*)&pitchangle;
    for(int i=0;i<4;i++)
    {
        cmd.data[25+i] = *pPitchAngle;
        pPitchAngle++;
    }

    char* pRollAngle = (char*)&rollangle;
    for(int i=0;i<4;i++)
    {
        cmd.data[29+i] = *pRollAngle;
        pRollAngle++;
    }

    unsigned char checkSum = 0;
    for(int i=1;i<=32;i++)
    {
        checkSum += cmd.data[i];
    }
    cmd.data[33] = (char)checkSum;
    cmd.data[34] = (char)0xff;
    cmd.length = 35;
}

void SimpleJointCommandBuilder(Command &cmd,Joint joint,float angle)
{
    cmd.data[0] = (char)0x8f;
    cmd.data[1] = (char)0x10;
    cmd.data[2] = (char)0x05;
    cmd.data[3] = (char)joint;

    char* pAngle = (char*)&angle;
    for(int i=0;i<4;i++)
    {
        cmd.data[4+i] = *pAngle;
        pAngle++;
    }

    unsigned char checkSum = 0;
    for(int i=1;i<=7;i++)
    {
        checkSum += cmd.data[i];
    }

    cmd.data[8] = (char)checkSum;
    cmd.data[9] = (char)0xff;
    cmd.length = 10;
}

void HeadJointCommandBuilder(Command &cmd,float angle)
{
    cmd.data[0] = (char)0x8f;
    cmd.data[1] = (char)0x16;
    cmd.data[2] = (char)0x04;

    char* pAngle = (char*)&angle;
    for(int i=0;i<4;i++)
    {
        cmd.data[3+i] = *pAngle;
        pAngle++;
    }

    unsigned char checkSum = 0;
    for(int i=1;i<7;i++)
    {
        checkSum += cmd.data[i];
    }

    cmd.data[7] = (char)checkSum;
    cmd.data[8] = (char)0xff;
    cmd.length = 9;
}

void SimpleCommandBuilder(Command &cmd, SimpleCommand index)
{
    cmd.data[0] = (char)0x8F;
    cmd.data[1] = (char)0x00;
    switch (index) {
    case Standing:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x0D;
        break;
    case InPlace:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x00;
        break;
    case ForWord:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x01;
        break;
    case BackWord:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x11;
        break;
    case ToLeftSide:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x02;
        break;
    case ToRightSide:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x12;
        break;
    case LeftRotate:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x03;
        break;
    case RightRotate:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x13;
        break;
    case TrunLeft:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x04;
        break;
    case TrunRight:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x14;
        break;
    case MediumSpeedRun:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x0B;
        break;
    case HighSpeedRun:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x0C;
        break;
    case StandToSitdown:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x28;
        break;
    case SitdownAndForefoot:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x26;
        break;
    case SitdownAndForefootAndSitdown:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x27;
        break;
    case JumpInplace:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x09;
        break;
    case JumpForword:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x0A;
        break;
    case StandAndImpact:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x06;
        break;
    case ForwordAndImpact:
        cmd.data[2] = (char)0x01;
        cmd.data[3] = (char)0x08;
        break;
    default:

        break;
    }

    char checkSum = 0;
    for(int i=1;i<=3;i++){
        checkSum += cmd.data[i];
    }
    cmd.data[4] = checkSum;
    cmd.data[5] = (char)0xFF;
    cmd.length = 6;
}
