#include "_tim.h"
#include "config.h"

void _TIM_Init(void) {
    // set usecTim TIM_IT_UPDATE
    HAL_TIM_Base_Start_IT(&usecTim);
}
