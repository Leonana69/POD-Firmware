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
} static MotorTim[4];

static bool isInit = false;


static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	}
}

void motorsDshotInit() {
	if (isInit)
		return;

	// MotorTim[0].tim = &MOTOR1_TIM;
	// MotorTim[0].channel = MOTOR1_CHANNEL;
	__HAL_TIM_SET_PRESCALER(&MOTOR1_TIM, 13);
	__HAL_TIM_SET_AUTORELOAD(&MOTOR1_TIM, 20);

	MOTOR1_TIM.hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;
	HAL_TIM_PWM_Start(&MOTOR1_TIM, MOTOR1_CHANNEL);

	isInit = true;
}

bool motorsDshotTest() {
	return isInit;
}
#define DSHOT_DMA_BUFFER_SIZE 20
#define MOTOR_BIT_0            	7
#define MOTOR_BIT_1            	14
static uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

static uint16_t dshot_prepare_packet(uint16_t value)
{
	uint16_t packet;
	bool dshot_telemetry = false;

	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0; i < 3; i++)
	{
        csum ^=  csum_data; // xor data by nibbles
        csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value)
{
	uint16_t packet;
	packet = dshot_prepare_packet(value);

	for(int i = 0; i < 16; i++)
	{
		motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	motor_dmabuffer[16] = 0;
	motor_dmabuffer[17] = 0;
}

void motorsDshotSetRatio(uint8_t id, uint16_t thrust) {
	if (!isInit)
		return;
	dshot_prepare_dmabuffer(motor1_dmabuffer, thrust);
	HAL_DMA_Start_IT(MOTOR1_TIM.hdma[TIM_DMA_ID_CC4], (uint32_t)motor1_dmabuffer, (uint32_t)&MOTOR1_TIM.Instance->CCR4, DSHOT_DMA_BUFFER_SIZE);
	__HAL_TIM_ENABLE_DMA(&MOTOR1_TIM, TIM_DMA_CC4);
}