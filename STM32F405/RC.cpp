#include "label.h"
#include "RC.h"
#include "control.h"
#include "HTmotor.h"

void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void RC::OnRC()
{
	if (!m_hasValidFrame || (xTaskGetTickCount() - m_lastValidFrameTick) > pdMS_TO_TICKS(100))
	{
		if (ctrl.mode != CONTROL::RESET)
		{
			ctrl.HoldPose();
			ctrl.shooter.single_shot_ready = false;
		}
		ctrl.mode = CONTROL::RESET;
	}
	else
	{
		if (Shift_mode() || (ctrl.mode == CONTROL::RESET && input.s[0] >= UP && input.s[0] <= MID && input.s[1] >= UP && input.s[1] <= MID && RC_STATE(input.s[0], input.s[1]) != RC_STATE(MID, MID)))
		{
			ctrl.HoldPose();
			ctrl.shooter.single_shot_ready = false;
			previous.s[0] = input.s[0];
			previous.s[1] = input.s[1];
		}
		RC_CheckState();
	}
	RC_Control();
}

void RC::OnPC()
{
	;
}

void RC::Update()
{
	OnRC();
	OnPC();
}

void RC::RC_CheckState() {

	switch (RC_STATE(input.s[0], input.s[1]))
	{
	case RC_STATE(UP, UP):
		ctrl.mode = CONTROL::YAW_FOLLOW;
		break;

	case RC_STATE(UP, MID):
		ctrl.mode = CONTROL::PANTILE_CONTROL;
		break;

	case RC_STATE(UP, DOWN):
		ctrl.mode = CONTROL::SPINNING;
		break;

	case RC_STATE(MID, UP):
		ctrl.mode = CONTROL::RC_FOLLOW;
		break;

	case RC_STATE(MID, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(MID, DOWN):
		ctrl.mode = CONTROL::SEPARATE;
		break;

	case RC_STATE(DOWN, UP):
		ctrl.mode = CONTROL::FIRE;
		break;

	case RC_STATE(DOWN, MID):
		ctrl.mode = CONTROL::FIRE;
		break;

	case RC_STATE(DOWN, DOWN):
		ctrl.mode = CONTROL::STOP;
		break;

	default:
		ctrl.mode = CONTROL::RESET;
		break;
	}

}

void RC::RC_Control() {
	if (ctrl.mode != CONTROL::YAW_FOLLOW && ctrl.mode != CONTROL::RC_FOLLOW && ctrl.mode != CONTROL::PANTILE_CONTROL && ctrl.mode != CONTROL::SEPARATE)
	{
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;
		ctrl.Control_Chassis(0, 0, 0);
	}
	if (ctrl.mode != CONTROL::SEPARATE)
	{
		can1_motor[5].setspeed = 0;
		can1_motor[6].setspeed = 0;
	}
	if (ctrl.mode != CONTROL::FIRE && visionActive)
	{
		ctrl.StopVision();
		visionActive = false;
	}
	if (ctrl.mode != CONTROL::FIRE && ctrl.mode != CONTROL::SPINNING)
	{
		ctrl.supply_motor[0]->pd = false;
		ctrl.supply_motor[0]->need_curcircle = 0;
		ctrl.shooter.single_shot_ready = false;
	}
	if (ctrl.mode != CONTROL::SPINNING && ctrl.mode != CONTROL::FIRE)
	{
		ctrl.supply_motor[0]->setspeed = 0;
		ctrl.supply_motor[0]->spinning = false;
		ctrl.shooter.openRub = false;
	}
	if (ctrl.mode != CONTROL::RESET)
	{
		switch (ctrl.mode)
		{
		case CONTROL::YAW_FOLLOW:
			ctrl.chassis.speedx = input.ch[0] * para.max_speed / 660.f;
			ctrl.chassis.speedy = input.ch[1] * para.max_speed / 660.f;
			ctrl.chassis.speedz = input.ch[2] * para.max_speed / 660.f;
			ctrl.MoveYaw();
			break;

		case CONTROL::PANTILE_CONTROL:
			ctrl.chassis.speedx = input.ch[0] * para.max_speed / 660.f;
			ctrl.chassis.speedy = input.ch[1] * para.max_speed / 660.f;
			ctrl.chassis.speedz = 0;
			ctrl.MoveYaw();
			ctrl.MovePitch();
			break;

		case CONTROL::RC_FOLLOW:
			ctrl.chassis.speedx = input.ch[0] * para.max_speed / 660.f;
			ctrl.chassis.speedy = input.ch[1] * para.max_speed / 660.f;
			ctrl.chassis.speedz = input.ch[2] * para.max_speed / 660.f;
			ctrl.MovePitch();
			break;

		case CONTROL::SEPARATE:
			ctrl.SeparateDrive();
			break;

		case CONTROL::AUTOAIM:
			break;

		case CONTROL::FIRE:
			ctrl.VisionFire();
			break;

		case CONTROL::STOP:
			ctrl.StopMotors();
			break;

		case CONTROL::SPINNING:
			ctrl.shooter.ManualFire();
			ctrl.MoveYaw();
			ctrl.MovePitch();
			break;

		default:
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			break;
		}
	}
	else
	{
		ctrl.StopMotors();
	}
}

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 或者报错
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, 0);
	}
	if (pd_Rx != pdTRUE) return;

	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;
	m_lastValidFrameTick = xTaskGetTickCount();
	m_hasValidFrame = true;

	input.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	input.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	input.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	input.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (input.ch[0] <= 8 && input.ch[0] >= -8)
	{
		input.ch[0] = 0;
	}
	if (input.ch[1] <= 8 && input.ch[1] >= -8)
	{
		input.ch[1] = 0;
	}
	if (input.ch[2] <= 8 && input.ch[2] >= -8)
	{
		input.ch[2] = 0;
	}
	if (input.ch[3] <= 8 && input.ch[3] >= -8)
	{
		input.ch[3] = 0;
	}

	input.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	input.s[1] = ((m_frame[5] >> 4) & 0x03);

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E

}

bool RC::Shift_mode()
{
	if (input.s[0] != previous.s[0] || input.s[1] != previous.s[1])
	{
		return true;
	}
	return false;
}
