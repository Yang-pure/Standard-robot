#pragma once

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "judgement.h"
#include "pid.h"
#include <string.h>

#pragma pack(push, 1)
struct TxPacket_TJ
{
	uint8_t head[2] = {'S', 'P'};
	uint8_t mode_TJ = 0;
	uint8_t robot_id = 0;
	float bullet_speed_TJ = 0.0f;
	uint16_t bullet_count_TJ = 0;
	float imu_pitch_TJ = 0.0f;
	float imu_yaw_TJ = 0.0f;
	float imu_roll_TJ = 0.0f;
	uint16_t crc16_TJ = 0;
};

struct RxPacket_TJ
{
	uint8_t head[2] = {'S', 'P'};
	uint8_t control_TJ = 0;
	uint8_t shoot_TJ = 0;
	float yaw_TJ = 0.0f;
	float pitch_TJ = 0.0f;
	uint16_t crc16_TJ = 0;
};
#pragma pack(pop)

static_assert(sizeof(TxPacket_TJ) == 24, "TxPacket_TJ size error");
static_assert(sizeof(RxPacket_TJ) == 14, "RxPacket_TJ size error");

class XUC
{
public:
	TxPacket_TJ Tx_TJ{};
	RxPacket_TJ Rx_TJ{};

	// 保留原有公开状态，避免影响工程其他模块。
	float feedforward = 1.2f;
	float yaw = 0.0f, pitch = 0.0f;
	float yaw_diff = 0.0f, pitch_diff = 0.0f;
	float distance = 0.0f, v_y = 0.0f;
	bool fireadvice = false;
	float speed_x = 0.0f, speed_y = 0.0f, prespeedx = 0.0f, prespeedy = 0.0f;
	float yaw_bias = -2.7f;
	float x = 0.0f, y = 0.0f, z = 0.0f;
	float vx = 0.0f, vy = 0.0f, vz = 0.0f;
	bool track_flag = false;
	bool test_navigation = false;
	float rad_pitch = 0.0f, rad_yaw = 0.0f;
	float aim_x = 0.0f, aim_y = 0.0f, aim_z = 0.0f;
	struct { uint8_t game_type = 0; uint8_t game_progress = 4; uint16_t stage_remain_time = 420; } test_game_status;
	struct {
		uint16_t red_1_robot_HP = 0, red_2_robot_HP = 0, red_3_robot_HP = 0, red_4_robot_HP = 0;
		uint16_t red_5_robot_HP = 0, red_7_robot_HP = 0, red_outpost_HP = 1400, red_base_HP = 0;
		uint16_t blue_1_robot_HP = 0, blue_2_robot_HP = 0, blue_3_robot_HP = 0, blue_4_robot_HP = 0;
		uint16_t blue_5_robot_HP = 0, blue_7_robot_HP = 0, blue_outpost_HP = 1400, blue_base_HP = 0;
	} test_game_robot_HP;
	enum ARMOR_COLOR { RED = 0, BLUE };
	ARMOR_COLOR own_color = RED;
	uint32_t count = 0, navi_count = 0, aim_count = 0;
	PID autoaim_controller[2];
	bool stm32_fireadvice = false;
	float yaw_pre = 0.0f, yaw_spd = 0.0f, k_feedforward = 0.5f;

	void Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate);
	void Decode();
	void Encode();

private:
	QueueHandle_t* queue_handler = NULL;
	BaseType_t pd_Rx = pdFALSE, pd_Tx = pdFALSE;
	UART* m_uart = nullptr;
	uint8_t* frame = nullptr;
	uint8_t m_frame[UART_MAX_LEN]{};
	uint8_t tx_data[sizeof(TxPacket_TJ)]{};
};

extern XUC xuc;
