# POD-Firmware

This project is based on the crazyflie-firmware. It uses STM32CUBEMX to generate the orignal structure and uses HAL library instead of STM32 Standard Peripheral Libraries (SPL), which is deprecated.

## STM32CubeMX Setup

1. Active TIM7 as usecTim
    - Prescaler: ABP1 timer clock / 1000000 (84)
    - Counter Mode: up
    - Counter Period: 0xFFFF (65535)
    - auto-reload preload: Disable
    - Tigger Event Selection: Reset
    - Enable NVIC with Preemption Pri = 4

2. NVIC
    - DMA

3. SYS
    - Timebase Source: TIM6

3. Enable FreeRTOS
    - CMSIS V2

4. Enable I2C1 and I2C3
    - I2C Speed Mode: Fast Mode
    - I2C Clock Speed: 400000
    - Fast Mode Duty Cycle: 2
    - Enable Rx DMA
        - Mode: Normal
        - Increment Address: enable Memory Inc
        - Data Width: Byte, Byte
    - Enable I2C1 event interrupt: 7
    - Enable I2C1 error interrupt: 7

5. Enable USART6
    - Baud Rate: 1000000
    - Word Length: 8 Bits
    - Parity: None
    - Stop Bits: 1
    - Data Direction: Receive and Transmit
    - Over Sampling: 16 Samples
    - Enable TX DMA: DMA2 Stream 7
    - Pull up GPIOs
    - Enable DMA2 strem7 global interrupt and USART6 global interrupt

6. Enable PA4 (GPIO_EXTI4)
    - GPIO Mode: External Interrupt Mode with Rising/Falling edge trigger detection
    - GPIO Pull-up/Pull-down: Pull-up
    - NVIC: Enable EXTI line4 interrupt: 5, 0

7. LED GPIOs
    - PD2, PC0, PC1, PC2, PC3

8. 

9. Enable PC14 (GPIO_EXTI14)
    - GPIO Mode: External Interrupt Mode with Rising edge trigger detection
    - GPIO Pull-up/Pull-down: No
    - NVIC: Enable EXTI line4 interrupt: 5, 0
    <!-- Enable IWDG -->
    <!-- - IWDG counter clock prescaler: 32 -->
    <!-- - IWDG down-counter reload value: 188 -->

10. Motor GPIOs
    - Enable TIM2: Prescaler(0), Mode(Up), Period(255), No Division, Auto-reload(Enable), Trig M/S Mode(Disable), Trig Event Selection(Reset)
        - Channel1: PWM Generation CH1 on PA15
            - Mode: PWM mode 1
            - Pulse: 0
            - Output compare preload: Enable
            - Fast Mode: Disable
            - CH: High
        - Channel2: PWM Generation CH2 on PA1
        - Channel4: PWM Generation CH4 on PB11
    - Enable TIM4: Prescaler(0), Mode(Up), Period(255), No Division, Auto-reload(Enable), Trig M/S Mode(Disable), Trig Event Selection(Reset)
        - Channel4: PWM Generation CH4 on PB9
    - The HAL_TIM_PWM_START needs to be called to enable the output.

11. Comment the #define __FPU_PRESENT in Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f405xx.h
    - This line will cause redefinition of __FPU_PRESENT, I guess it's a mismatch between CMSIS and STM32CubeMX.
## Modifications to Auto-generated Files

### STM32F405RGTx_FLASH.ld

Change the flash origin address to 0x8004000:

```FLASH (rx)      : ORIGIN = 0x8004000, LENGTH = 1008K```

Add _param, _log, _deckDriver, _eventtrigger to the ld file.
### stm32f4xx_it.c

