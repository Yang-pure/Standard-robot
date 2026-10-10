#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "RC.h"
#include "xuc.h"

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			supply_motor[num4] = motor[i];
			supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			++num4;
		default:
			break;
		}
	}
	if (pantile_motor[PANTILE::TYPE::PITCH] != nullptr)
		pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;

}

void CONTROL::Control_Chassis(float speedx, float speedy, float speedz)

{
	
	const int32_t wheel_speed[4] = {
		static_cast<int32_t>(+speedx + speedy +  speedz), // ID1 后右  后右
		static_cast<int32_t>(+speedx - speedy +  speedz), // ID2 后左  前右
		static_cast<int32_t>(-speedx - speedy +  speedz), // ID3 前左  前左
		static_cast<int32_t>(-speedx + speedy +  speedz)  // ID4 前右  后左*/
	};

	for (int i = 0; i < 4; i++)
	{
		if (chassis_motor[i] == nullptr) continue;
		chassis_motor[i]->setspeed =
			std::max(-para.max_speed, std::min(para.max_speed, wheel_speed[i]));
	}
	return ;
}

void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	
}

void CONTROL::SHOOTER::RequestSingleShot(int16_t channel)
{
	openRub = (channel > 100);

	if (channel <= 100)
	{
		single_shot_ready = true;  // 松开后允许下一发
	}
	else if (channel >= 330 && single_shot_ready)
	{
		ctrl.supply_motor[0]->pd = true;
		single_shot_ready = false;
	}
}

void CONTROL::HoldPose()
{
	pantile_motor[0]->setangle = pantile_motor[0]->angle[now];
	DMmotor[0].setPos = DMmotor[0].pos;
	DMmotor[1].setPos = DMmotor[1].pos;
	rc.sumPos = (rc.previous.s[0] == 0 && rc.previous.s[1] == 0) ? 0.0f : DMmotor[2].pos;
	if (rc.sumPos > 1.0f)
	{
		rc.sumPos = 1.0f;
	}
	else if (rc.sumPos < -1.0f)
	{
		rc.sumPos = -1.0f;
	}
	DMmotor[2].setPos = rc.sumPos;
	DMmotor[0].setSpeed = 0;
	DMmotor[1].setSpeed = 0;
	DMmotor[2].setSpeed = 0;
}

void CONTROL::MoveYaw()
{
	if (mode == YAW_FOLLOW && (rc.input.ch[3] >= 100 || rc.input.ch[3] <= -100))
	{
		pantile_motor[0]->setangle += -rc.input.ch[3] * 30.f / 660.f;
	}
	else if (mode == PANTILE_CONTROL && (rc.input.ch[2] >= 100 || rc.input.ch[2] <= -100))
	{
		pantile_motor[0]->setangle += -rc.input.ch[2] * 30.f / 660.f;
	}
	else if (mode == SPINNING && (rc.input.ch[0] >= 100 || rc.input.ch[0] <= -100))
	{
		pantile_motor[0]->setangle += -rc.input.ch[0] * 30.f / 660.f;
	}
}

void CONTROL::MovePitch()
{
	if (mode == PANTILE_CONTROL || mode == RC_FOLLOW)
	{
		rc.sumPos += -rc.input.ch[3] * 0.01f / 660.f;
	}
	else if (mode == SPINNING)
	{
		rc.sumPos += -rc.input.ch[1] * 0.01f / 660.f;
	}
	if (rc.sumPos > 1.0f)
	{
		rc.sumPos = 1.0f;
	}
	else if (rc.sumPos < -1.0f)
	{
		rc.sumPos = -1.0f;
	}
	DMmotor[2].setPos = rc.sumPos;
	DMmotor[2].setSpeed = 2.0f;
}

void CONTROL::StopMotors()
{
	chassis.speedx = 0;
	chassis.speedy = 0;
	chassis.speedz = 0;
	can1_motor[0].setspeed = 0;
	can1_motor[1].setspeed = 0;
	can1_motor[2].setspeed = 0;
	can1_motor[3].setspeed = 0;
	can1_motor[4].setspeed = 0;
	can1_motor[5].setspeed = 0;
	can1_motor[6].setspeed = 0;
	can1_motor[7].setspeed = 0;
	can2_motor[0].setspeed = 0;
	can2_motor[1].setspeed = 0;
	DMmotor[0].setSpeed = 0;
	DMmotor[1].setSpeed = 0;
	DMmotor[2].setSpeed = 0;
}

void CONTROL::SeparateDrive()
{
	DMmotor[0].setPos = -rc.input.ch[3] * 1.5f / 660.f;
	DMmotor[0].setSpeed = 1.5f;
	DMmotor[1].setPos = rc.input.ch[3] * 1.5f / 660.f;
	DMmotor[1].setSpeed = 1.5f;
	can1_motor[5].setspeed = -3000;
	can1_motor[6].setspeed = 3000;
	chassis.speedx = rc.input.ch[0] * para.max_speed / 660.f;
	chassis.speedy = rc.input.ch[1] * para.max_speed / 660.f;
	if (rc.input.ch[2] > 100 || rc.input.ch[2] < -100)
	{
		chassis.speedz = rc.input.ch[2] * para.max_speed / 660.f;
	}
	else
	{
		chassis.speedz = 0;
	}
}

void CONTROL::SHOOTER::ManualFire()
{
	if (rc.input.ch[3] > 500)
	{
		ctrl.supply_motor[0]->spinning = true;
		ctrl.supply_motor[0]->pd = false;
		ctrl.supply_motor[0]->need_curcircle = 0;
		ctrl.supply_motor[0]->setspeed = rc.input.ch[3] * 4000.f / 660.f;
		single_shot_ready = false;
		openRub = true;
	}
	else
	{
		ctrl.supply_motor[0]->spinning = false;
		RequestSingleShot(rc.input.ch[2] > 500 ? rc.input.ch[2] : 0);
		openRub = rc.input.ch[2] > 500 || ctrl.supply_motor[0]->pd || ctrl.supply_motor[0]->need_curcircle != 0;
		if (!ctrl.supply_motor[0]->pd && ctrl.supply_motor[0]->need_curcircle == 0)
		{
			ctrl.supply_motor[0]->setspeed = 0;
		}
	}
}

void CONTROL::StopVision()
{
	rc.visionShot.Reset();
	supply_motor[0]->pd = false;
	supply_motor[0]->need_curcircle = 0;
	supply_motor[0]->spinning = false;
	supply_motor[0]->setspeed = 0;
	shooter.openRub = false;
	pantile_motor[0]->setangle = pantile_motor[0]->angle[now];
	rc.sumPos = DMmotor[2].pos;
	if (rc.sumPos > 1.0f)
	{
		rc.sumPos = 1.0f;
	}
	else if (rc.sumPos < -1.0f)
	{
		rc.sumPos = -1.0f;
	}
	DMmotor[2].setPos = rc.sumPos;
}

void CONTROL::VisionFire() // FIRE 模式每轮调用：读取视觉目标，控制云台位置和射击输出。
{
	shooter.supply_bullet = true; // 标记进入视觉射击流程；实际拨弹仍由下方 feeder 指令决定。
	if (!rc.visionActive) // 只在首次进入 FIRE 时清理上一次视觉会话。
	{
		rc.visionActive = true; // 记录 FIRE 会话已启动，避免后续循环重复初始化。
		rc.visionShot.Reset(); // 清除上一会话的连发、单发及超时状态。
		xuc.InvalidateVisionCommand(); // 丢弃切入 FIRE 前收到的视觉帧，等待新命令。
		pantile_motor[0]->setangle = pantile_motor[0]->angle[now]; // Yaw 先锁定当前编码器位置，避免沿用旧目标。
		DMmotor[2].setPos = rc.sumPos; // Pitch 先沿用切模式时同步好的位置目标。
		DMmotor[2].setSpeed = 2.0f; // Pitch 位置速度模式的最大运动速度设为 2 rad/s。
	}
	const TickType_t nowTick = xTaskGetTickCount(); // 读取当前系统节拍，用于视觉帧时效和射击状态计时。
	RxPacket_TJ command{}; // 准备接收本轮视觉命令，默认字段全部清零。
	const bool valid = xuc.GetVisionCommand(command, nowTick) && imu_pantile.Fresh(); // 视觉命令与 IMU 反馈都有效才允许本轮瞄准和开火。
	Motor* feeder = supply_motor[0]; // 指向拨弹电机，后续集中写入单发或连发指令。
	const int32_t readyRpm = (shooter.shoot_speed < shooter_motor[0]->maxspeed ? shooter.shoot_speed : shooter_motor[0]->maxspeed) * 4 / 5; // 就绪阈值取目标与电机上限较小值的 80%。
	const bool frictionReady = std::abs(shooter_motor[0]->curspeed) >= readyRpm && std::abs(shooter_motor[1]->curspeed) >= readyRpm; // 两只摩擦轮都达到阈值才允许供弹。
	const bool singleBusy = feeder->pd || feeder->need_curcircle != 0; // 已请求单发或单发行程尚未结束时，不重置拨弹速度。
	const VisionShotAction action = rc.visionShot.Update(valid ? command.shoot_TJ : 0, valid, frictionReady, singleBusy, nowTick * portTICK_PERIOD_MS); // 将 shoot=0/1/2 和电机状态转换为本轮摩擦轮、连发、单发动作。

	if (!valid) // 视觉帧或 IMU 失效时停止供弹；Yaw、Pitch 保持最后的位置目标。
	{
		feeder->pd = false; // 撤销尚未执行的单发请求。
		feeder->need_curcircle = 0; // 清除正在记录的单发行程状态。
		feeder->spinning = false; // 关闭连续拨弹模式。
		feeder->setspeed = 0; // 将拨弹速度目标清零。
	}
	else // 视觉和 IMU 均有效时，才根据新目标更新云台与拨弹。
	{
		float yawTarget = pantile_motor[0]->setangle, pitchTarget = DMmotor[2].setPos; // 以上轮位置目标为起点，避免每轮从反馈值重新计算。
		if (VisionAim(command.yaw_TJ, command.pitch_TJ, imu_pantile.GetAngleYaw(), imu_pantile.GetAnglePitch(), pantile_motor[0]->angle[now], DMmotor[2].pos, yawTarget, pitchTarget)) // Yaw 和 Pitch 均直接跟随视觉目标，计算限幅位置目标。
		{
			pantile_motor[0]->setangle = yawTarget; // 下发 Yaw 编码器位置目标；0.25° 视觉死区内保持原值。
			DMmotor[2].setPos = pitchTarget; // 下发 Pitch DM 位置目标；VisionAim 已限制在 [-1,1]。
			DMmotor[2].setSpeed = 2.0f; // 保持 Pitch 最大运动速度为 2 rad/s。
		}
		feeder->spinning = action.feed; // 连发动作使拨弹电机进入连续转动分支。
		if (action.feed) // shoot=1 且摩擦轮就绪、单发未占用时连续拨弹。
		{
			feeder->setspeed = para.ace_speed; // 连发拨弹速度沿用工程参数。
		}
		else if (!singleBusy) // 无连发且单发行程未占用时，停止拨弹速度指令。
		{
			feeder->setspeed = 0; // 防止上一次连发速度残留。
		}
		if (action.single) // shoot=2 满足再装填与摩擦轮就绪条件时触发一次单发。
		{
			feeder->pd = true; // 把单发请求交给拨弹电机的位置行程逻辑。
		}
		if (action.abortSingle) // 单发执行超时后撤销请求，等待视觉 shoot 先回到 0。
		{
			feeder->pd = false; // 清除单发触发标志。
			feeder->need_curcircle = 0; // 清除未完成的单发行程标志。
			feeder->setspeed = 0; // 停止拨弹电机速度指令。
		}
	}
	shooter.openRub = true; // 进入 FIRE 后摩擦轮持续运行，不受视觉指令或视觉帧有效性影响。
}

void CONTROL::CHASSIS::Keep_Direction()
{
	Motor* yaw_motor = ctrl.pantile_motor[PANTILE::YAW];
	if (yaw_motor == nullptr)
	{
		return;
	}

	// 以发射口正对底盘前方时的编码器值为零点，求云台相对底盘的最短偏角。
	float yaw_delta = yaw_motor->angle[now] - para.initial_yaw;
	while (yaw_delta > 4096.0f)
	{
		yaw_delta -= 8192.0f;
	}
	while (yaw_delta < -4096.0f)
	{
		yaw_delta += 8192.0f;
	}

	const float yaw_rad = yaw_delta * 2.0f * PI / 8192.0f;
	const float command_x_gimbal = static_cast<float>(speedx);
	const float command_y_gimbal = static_cast<float>(speedy);

	// 将以云台发射口为参考的平移指令转换到底盘自身坐标系。
	speedx = static_cast<int32_t>(command_x_gimbal * cosf(yaw_rad) - command_y_gimbal * sinf(yaw_rad));
	speedy = static_cast<int32_t>(command_x_gimbal * sinf(yaw_rad) + command_y_gimbal * cosf(yaw_rad));

}

void CONTROL::CHASSIS::Update()
{
	if (ctrl.mode == CONTROL::YAW_FOLLOW)
	{
		Keep_Direction();
	}

	ctrl.Control_Chassis(speedx, speedy, speedz);
}

void CONTROL::PANTILE::Update()
{
	if (ctrl.pantile_motor[0]->setangle > 8192.0)
	{
		ctrl.pantile_motor[0]->setangle -= 8192.0;
	}
	if (ctrl.pantile_motor[0]->setangle < 0.0)
	{
		ctrl.pantile_motor[0]->setangle += 8192.0;
	}
}

void CONTROL::SHOOTER::Update()
{
	if (ctrl.mode == RESET)
	{
		openRub = false;
		supply_bullet = false;
		auto_shoot = false;
	}
	const int16_t speed = ((ctrl.mode == SPINNING || ctrl.mode == FIRE) && openRub) ? shoot_speed : 0;
	ctrl.shooter_motor[0]->setspeed = speed;
	ctrl.shooter_motor[1]->setspeed = -speed;
	if (ctrl.supply_motor[0]->setangle > 8192.0)
	{
		ctrl.supply_motor[0]->setangle -= 8192.0;
	}
	if (ctrl.supply_motor[0]->setangle < 0.0)
	{
		ctrl.supply_motor[0]->setangle += 8192.0;
	}
}


float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

