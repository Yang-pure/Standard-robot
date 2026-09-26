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
	RC_CheckState();
	if (!m_hasValidFrame ||
		(xTaskGetTickCount() - m_lastValidFrameTick) > pdMS_TO_TICKS(100))
		ctrl.mode = CONTROL::RESET;
	RC_Control();

	if (Shift_mode())
	{

	}

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

	switch (RC_STATE(rc.s[0], rc.s[1]))
	{
	case RC_STATE(UP, UP):
		ctrl.mode = CONTROL::ROTATION;
		break;

	case RC_STATE(UP, MID):
		ctrl.mode = CONTROL::ROTATION;
		break;

	case RC_STATE(UP, DOWN):
		ctrl.mode = CONTROL::SPINNING;
		break;

	case RC_STATE(MID, UP):
		ctrl.mode = CONTROL::FOLLOW;
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
	if (ctrl.mode != CONTROL::FIRE && ctrl.mode != CONTROL::SPINNING)
	{
		ctrl.supply_motor[0]->pd = false;
		ctrl.supply_motor[0]->need_curcircle = 0;
	}
	if (ctrl.mode != CONTROL::SPINNING)
	{
		ctrl.supply_motor[0]->setspeed = 0;
		ctrl.supply_motor[0]->spinning = false;
		ctrl.shooter.openRub = false;
	}
	if (ctrl.mode != CONTROL::RESET)
	{

		/*ctrl.chassis.speedx = rc.ch[3] * 4000.f / 660.f;
		ctrl.chassis.speedy = -1 * rc.ch[2] * 4000.f / 660.f;
		ctrl.chassis.speedz = 0;*/

		//ctrl.chassis.Keep_Direction();

		switch (ctrl.mode)
		{
		case CONTROL::ROTATION:
		{
			ctrl.chassis.speedx = rc.ch[0] * para.max_speed / 660.f;
			ctrl.chassis.speedy = rc.ch[1] * para.max_speed / 660.f;
			ctrl.chassis.speedz = rc.ch[2] * para.max_speed / 660.f;
			DMmotor[2].setPos = -rc.ch[3] * 1.0 / 660.f;
			DMmotor[2].setSpeed = 2.0f;
		}

			break;

		case CONTROL::FOLLOW:

			break;

		case CONTROL::SEPARATE:

			break;

		case CONTROL::AUTOAIM:
			break;

		case CONTROL::FIRE:
			ctrl.shooter.supply_bullet = true;
			/*ctrl.chassis.speedx = rc.ch[0] * para.max_speed / 660.f;
			ctrl.chassis.speedy = rc.ch[1] * para.max_speed / 660.f;*/
			if (rc.ch[0] >= 200 || rc.ch[0] <= -200)
			{
				ctrl.pantile_motor[0]->setangle += -rc.ch[0] * 30.f / 660.f;
			}
			DMmotor[2].setPos = -rc.ch[1] * 1.0f / 660.f;
			DMmotor[2].setSpeed = 1.0f;
			ctrl.shooter.RequestSingleShot(rc.ch[3]);
			break;

		case CONTROL::STOP:

			break;

		case CONTROL::SPINNING:
			ctrl.supply_motor[0]->spinning = true;
			ctrl.shooter.openRub = (rc.ch[0] > 100);
			can1_motor[7].setspeed = rc.ch[0]* 4000.f / 660.f;
			break;

		default:
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			break;
		}
	}
	else {
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

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);

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
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}
