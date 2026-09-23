#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"

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
			supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
	}
	if (pantile_motor[PANTILE::TYPE::PITCH] != nullptr)
		pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	if (pantile_motor[PANTILE::TYPE::YAW] != nullptr)
		pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;
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

void CONTROL::CHASSIS::Keep_Direction()
{


}

void CONTROL::CHASSIS::Update()
{
	ctrl.Control_Chassis(speedx, speedy, speedz);
}

void CONTROL::PANTILE::Update()
{
	
}

void CONTROL::SHOOTER::Update()
{
	
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

