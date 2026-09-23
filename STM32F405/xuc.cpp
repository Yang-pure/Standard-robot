#include "xuc.h"
#include "label.h"
#include "imu.h"
#include "CRC.h"
#include "math.h"
#include "RC.h"
#include <cmath>
#include <cstring>

namespace
{
constexpr uint8_t kHead0 = 'S';
constexpr uint8_t kHead1 = 'P';
}

void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	m_uart = huart;
	queue_handler = nullptr;
	if (m_uart != nullptr)
	{
		m_uart->Init(Instance, BaudRate).DMARxInit(nullptr).DMATxInit();
		frame = m_uart->m_uartrx;
		queue_handler = &m_uart->UartQueueHandler;
	}

	autoaim_controller[0].m_Kp = 0.005f;
	autoaim_controller[0].m_Td = 0.004f;
	autoaim_controller[1].m_Kp = 0.0014f;
	autoaim_controller[1].m_Td = 0.001f;
}

void XUC::Decode()
{
	if (queue_handler == nullptr || *queue_handler == nullptr)
		return;

	pd_Rx = xQueueReceive(*queue_handler, m_frame, 0);
	if (pd_Rx == pdPASS)
	{
		constexpr uint32_t packetLen = sizeof(RxPacket_TJ);
		for (uint32_t i = 0; i + packetLen <= UART_MAX_LEN; ++i)
		{
			if (m_frame[i] != kHead0 || m_frame[i + 1] != kHead1)
				continue;

			uint8_t packet[packetLen]{};
			memcpy(packet, m_frame + i, packetLen);
			if (!VerifyCRC16CheckSum(packet, packetLen))
				continue;

			memcpy(&Rx_TJ, packet, packetLen);
			if (Rx_TJ.control_TJ == 0)
			{
				yaw = pitch = 0.0f;
				fireadvice = false;
			}
			else
			{
				yaw = Rx_TJ.yaw_TJ;
				pitch = Rx_TJ.pitch_TJ;
				fireadvice = (Rx_TJ.shoot_TJ != 0);
			}
			return;
		}
	}
}

void XUC::Encode()
{
	if (m_uart == nullptr)
		return;

	static uint16_t frameCounter = 0;
	Tx_TJ = {};
	Tx_TJ.head[0] = kHead0;
	Tx_TJ.head[1] = kHead1;
	Tx_TJ.mode_TJ = 0;
	Tx_TJ.robot_id = judgement.data.robot_status_t.robot_id;

	const float bulletSpeed = judgement.data.shoot_data_t.bullet_speed;
	Tx_TJ.bullet_speed_TJ = std::isfinite(bulletSpeed) ? bulletSpeed : 0.0f;
	Tx_TJ.bullet_count_TJ = frameCounter++;
	Tx_TJ.imu_pitch_TJ = imu_pantile.GetAnglePitch();
	Tx_TJ.imu_yaw_TJ = imu_pantile.GetAngleYaw();
	Tx_TJ.imu_roll_TJ = imu_pantile.GetAngleRoll();

	memcpy(tx_data, &Tx_TJ, sizeof(Tx_TJ));
	AppendCRC16CheckSum(tx_data, sizeof(Tx_TJ));
	m_uart->UARTTransmit(tx_data, sizeof(Tx_TJ));
}
