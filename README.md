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

2. NVIC Code generation
    - Disable IRQ handler generation for System service call via SWI instruction
    - Disable IRQ handler generation for Pendable request for system service

3. Enable IWDG
    - IWDG counter clock prescaler: 32
    - IWDG down-counter reload value: 188

3. Enable FreeRTOS
    - CMSIS V2
    - 

4. Enable I2C1
    - I2C Speed Mode: Fast Mode
    - I2C Clock Speed: 400000
    - Fast Mode Duty Cycle: 2
    - Enable Rx DMA
        - Mode: Normal
        - Increment Address: enable Memory Inc
        - Data Width: Byte, Byte
    - Enable I2C1 event interrupt: 7
    - Enable I2C1 error interrupt: 7
## Modifications to Auto-generated Files

### STM32F405RGTx_FLASH.ld

Change the flash origin address to 0x8004000:

```FLASH (rx)      : ORIGIN = 0x8004000, LENGTH = 1024K```

Add _param, _log, _deckDriver, _eventtrigger to the ld file.
### stm32f4xx_it.c

