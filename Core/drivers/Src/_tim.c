#include "_tim.h"
#include "config.h"

void _TIM_Init(void) {
    // set USEC_TIM TIM_IT_UPDATE
    HAL_TIM_Base_Start_IT(&USEC_TIM);
}
