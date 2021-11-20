#include "motors_dshot.h"
#include "tim.h"
#include "config.h"
#include "cmsis_os2.h"
#include "cfassert.h"
#include "debug.h"
#include "motors.h"
#include "dma.h"

struct {
	TIM_HandleTypeDef* tim;
	uint32_t channel;
	uint8_t TIM_DMA_ID_CC;
	uint32_t TIM_DMA_CC;
} static MotorTim[4];

static bool isInit = false;

static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
}

#define DSHOT150_PRESCALER 27
#define DSHOT300_PRESCALER 13
#define DSHOT600_PRESCALER 6
#define DSHOT_DMA_BUFFER_SIZE 20
#define MOTOR_BIT0_PULSE 7
#define MOTOR_BIT1_PULSE 14

void motorsDshotInit() {
	if (isInit)
		return;
	MotorTim[0].tim = &MOTOR1_TIM;
	MotorTim[0].channel = MOTOR1_CHANNEL;
	MotorTim[0].TIM_DMA_CC = MOTOR1_TIM_DMA_CC;
	MotorTim[1].tim = &MOTOR2_TIM;
	MotorTim[1].channel = MOTOR2_CHANNEL;
	MotorTim[1].TIM_DMA_CC = MOTOR2_TIM_DMA_CC;
	MotorTim[2].tim = &MOTOR3_TIM;
	MotorTim[2].channel = MOTOR3_CHANNEL;
	MotorTim[2].TIM_DMA_CC = MOTOR3_TIM_DMA_CC;
	MotorTim[3].tim = &MOTOR4_TIM;
	MotorTim[3].channel = MOTOR4_CHANNEL;
	MotorTim[3].TIM_DMA_CC = MOTOR4_TIM_DMA_CC;
	
	for (int i = 0; i < NBR_OF_MOTORS; i++) {
		MotorTim[i].TIM_DMA_ID_CC = MotorTim[i].channel / 4 + 1;
		__HAL_TIM_SET_PRESCALER(MotorTim[i].tim, DSHOT300_PRESCALER);
		__HAL_TIM_SET_AUTORELOAD(MotorTim[i].tim, DSHOT_DMA_BUFFER_SIZE - 1);
		MotorTim[i].tim->hdma[MotorTim[i].TIM_DMA_ID_CC]->XferCpltCallback = dshot_dma_tc_callback;
		HAL_TIM_PWM_Start(MotorTim[i].tim, MotorTim[i].channel);
	}
	isInit = true;
}

bool motorsDshotTest() {
	return isInit;
}

static uint32_t motorDmaBuffer[NBR_OF_MOTORS][DSHOT_DMA_BUFFER_SIZE];

static uint16_t dshot_prepare_packet(uint16_t value) {
	uint16_t packet;
	bool dshot_telemetry = false;

	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for (int i = 0; i < 3; i++) {
		csum ^=  csum_data; // xor data by nibbles
		csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value) {
	uint16_t packet;
	packet = dshot_prepare_packet(value);

	for (int i = 0; i < 16; i++) {
		motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT1_PULSE : MOTOR_BIT0_PULSE;
		packet <<= 1;
	}

	motor_dmabuffer[16] = 0;
	motor_dmabuffer[17] = 0;
	motor_dmabuffer[18] = 0;
	motor_dmabuffer[19] = 0;
}

void motorsDshotSetRatio(uint8_t id, uint16_t thrust) {
	if (!isInit)
		return;

	uint32_t value = thrust * 2048 / 65536;
	thrust = value & 0xFFFF;
	dshot_prepare_dmabuffer(motorDmaBuffer[id], thrust);

	switch (MotorTim[id].TIM_DMA_ID_CC) {
		case 1:
			HAL_DMA_Start_IT(MotorTim[id].tim->hdma[TIM_DMA_ID_CC1], (uint32_t)motorDmaBuffer[id],
											 (uint32_t)&MotorTim[id].tim->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
			__HAL_TIM_ENABLE_DMA(MotorTim[id].tim, TIM_DMA_CC1);
			break;
		case 2:
			HAL_DMA_Start_IT(MotorTim[id].tim->hdma[TIM_DMA_ID_CC2], (uint32_t)motorDmaBuffer[id],
											 (uint32_t)&MotorTim[id].tim->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
			__HAL_TIM_ENABLE_DMA(MotorTim[id].tim, TIM_DMA_CC2);
			break;
		case 3:
			HAL_DMA_Start_IT(MotorTim[id].tim->hdma[TIM_DMA_ID_CC3], (uint32_t)motorDmaBuffer[id],
											 (uint32_t)&MotorTim[id].tim->Instance->CCR3, DSHOT_DMA_BUFFER_SIZE);
			__HAL_TIM_ENABLE_DMA(MotorTim[id].tim, TIM_DMA_CC3);
			break;
		case 4:
			HAL_DMA_Start_IT(MotorTim[id].tim->hdma[TIM_DMA_ID_CC4], (uint32_t)motorDmaBuffer[id],
											 (uint32_t)&MotorTim[id].tim->Instance->CCR4, DSHOT_DMA_BUFFER_SIZE);
			__HAL_TIM_ENABLE_DMA(MotorTim[id].tim, TIM_DMA_CC4);
			break;
		default:
			break;
	}
}