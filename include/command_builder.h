#ifndef COMMAND_BUILDER_H
#define COMMAND_BUILDER_H

//指令
struct Command{
	char data[128];
	unsigned int length;
};


enum Action{
    Raise_feet = 0,     //站立->抬脚
    Kick = 1,           //抬脚->踢球
    Reset = 2,          //踢球->站立
    KeepBall = 3,       //站立->守门
    KeepBall_Reset = 4, //守门->站立
    Kick_slow = 5       //抬脚->踢球(慢踢)
};


void CallActionCommandBuilder(Command &cmd,
                            enum Action action);

/*步态枚举*/
enum Gait{
	Stand = 0,     // 站立
	Crawl = 1,     // 爬行
	Trot = 2,      // 小跑
	Pace = 3,      // 快跑
	Bound = 4,     // 弹跳
	Jump = 5,      // 跳跃
	FlyingTrot = 6,// 飞跑
	GaitTypeNumber // 步态的种类数量
};

/**
 * @brief MotorControlCommandBuilder : 运动控制指令生成函数
 * @param cmd : 要生成的指令信息
 * @param headAngle : 头部角度,默认0
 * @param neckAngle : 颈部转动的角度
 * @param gait : 步态枚举量
 * @param forward : 前进速度,默认0
 * @param sidespeed : 横移速度,默认0
 * @param circlespeed : 自转速度,默认0
 * @param standheight : 站高,默认144
 * @param pitchangle : 躯干俯仰角,默认0
 * @param rollangle : 躯干横滚角,默认0
 */
void MotorControlCommandBuilder(Command &cmd,
                            float headAngle,    // 头部要转动的角度
                            float neckAngle,    // 颈部转动的角度
                            enum Gait gait,     // 步态
                            float forward,      // 前进速度
                            float sidespeed,    // 横移速度
                            float circlespeed,  // 自转速度
                            unsigned char standheight,  // 站立高度
                            float pitchangle,   // 躯干俯仰角
                            float rollangle);   // 躯干横滚角

void co_operationCommandBuilder(Command &cmd,
                            unsigned char commondtype,//命令类型
							unsigned char robotid,//机器人编号
                            unsigned char corobotid,//机器人编号
							float locationx,
							float locationy,
							float oriatation,
							unsigned char parameter1,
							unsigned char parameter2,
							unsigned char parameter3);

void disgussBuilder(Command &cmd,
					float locationx,
					float locationy,
					float oriatation,
					float targetx,
					float targety,
					unsigned char shoot);


//关节ID
enum Joint{
	LF_High = 0,
	LF_Mid,
	LF_Low,
	RF_High,
	RF_Mid,
	RF_Low,
	LH_High,
	LH_Mid,
	LH_Low,
	RH_High,
	RH_Mid,
	RH_Low,
	Head,
	JointCounts
};

/**
 * @brief SimpleJointCommandBuilder : 单关节控制指令
 * @param cmd : 要生成的指令信息
 * @param joint : 关节枚举
 * @param angle : 角度值
 */
void SimpleJointCommandBuilder(Command &cmd,Joint joint,float angle);



/**
 * @brief HeadJointCommandBuilder : 头部关节控制指令
 * @param cmd : 要生成的指令信息
 * @param angle : -0.6 ~ 0.6
 */
void HeadJointCommandBuilder(Command &cmd,float angle);



enum SimpleCommand{
	Standing = 0,	//恢复静止站立
	InPlace,		// 原地踏步
	ForWord, 		// 向前走
	BackWord, 		// 向后走
	ToLeftSide, 	// 向左横行
	ToRightSide, 	// 向右横行
	LeftRotate, 	// 左自转
	RightRotate, 	// 右自转
	TrunLeft, 		// 左转
	TrunRight, 		// 右转
	MediumSpeedRun, // 中速跑
	HighSpeedRun, 	// 高速跑
	StandToSitdown, // 站立到坐下
	SitdownAndForefoot, // 坐下伸左前脚
	SitdownAndForefootAndSitdown, // 坐下伸左前脚然后再坐下
	JumpInplace, 	// 原地四足跳跃
	JumpForword, 	// 向前四足跳跃
	StandAndImpact, // 站立抗冲击
	ForwordAndImpact// 前进抗冲击
};


/**
 * @brief SimpleCommandBuilder : 简单控制指令
 * @param cmd : 要生成的命令
 * @param index : 简单指令枚举量
 */
void SimpleCommandBuilder(Command &cmd, SimpleCommand index);

#endif
