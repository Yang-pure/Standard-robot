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

void CONTROL::VisionFire()
{
	shooter.supply_bullet = true;
	if (!rc.visionActive)
	{
		rc.visionActive = true;
		rc.visionShot.Reset();
		xuc.InvalidateVisionCommand();
		pantile_motor[0]->setangle = pantile_motor[0]->angle[now];
		DMmotor[2].setPos = rc.sumPos;
		DMmotor[2].setSpeed = 8.0f;
	}
	const TickType_t nowTick = xTaskGetTickCount();
	RxPacket_TJ command{};
	const bool valid = xuc.GetVisionCommand(command, nowTick);
	Motor* feeder = supply_motor[0];
	const int32_t readyRpm = (shooter.shoot_speed < shooter_motor[0]->maxspeed ? shooter.shoot_speed : shooter_motor[0]->maxspeed) * 4 / 5;
	const bool frictionReady = std::abs(shooter_motor[0]->curspeed) >= readyRpm && std::abs(shooter_motor[1]->curspeed) >= readyRpm;
	const bool singleBusy = feeder->pd || feeder->need_curcircle != 0;
	const VisionShotAction action = rc.visionShot.Update(valid ? command.shoot_TJ : 0, valid, frictionReady, singleBusy, nowTick * portTICK_PERIOD_MS);

	if (!valid)
	{
		feeder->pd = false;
		feeder->need_curcircle = 0;
		feeder->spinning = false;
		feeder->setspeed = 0;
	}
	else
	{
		float yawTarget = 0.0f, pitchTarget = 0.0f;
		if (VisionAim(command.yaw_TJ, command.pitch_TJ, imu_pantile.GetAngleYaw(), imu_pantile.GetAnglePitch(), pantile_motor[0]->angle[now], DMmotor[2].pos, yawTarget, pitchTarget))
		{
			pantile_motor[0]->setangle = yawTarget;
			DMmotor[2].setPos = pitchTarget;
			DMmotor[2].setSpeed = 8.0f;
		}
		feeder->spinning = action.feed;
		if (action.feed)
		{
			feeder->setspeed = para.ace_speed;
		}
		else if (!singleBusy)
		{
			feeder->setspeed = 0;
		}
		if (action.single)
		{
			feeder->pd = true;
		}
		if (action.abortSingle)
		{
			feeder->pd = false;
			feeder->need_curcircle = 0;
			feeder->setspeed = 0;
		}
	}
	shooter.openRub = action.flywheel;
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

