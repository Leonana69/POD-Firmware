##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.14.1] date: [Thu Nov 18 11:56:26 EST 2021] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = POD-Firmware

######################################
# guojun's define
######################################
POD_BASE ?= ./
OPENOCD				?= openocd
OPENOCD_INTERFACE	?= interface/stlink-v2.cfg
OPENOCD_TARGET    	?= target/stm32f4x.cfg
OPENOCD_CMDS		?=
LOAD_ADDRESS		?= 0x8004000
PROG				?= $(BUILD_DIR)/$(TARGET)
CPU 				?= stm32f4

FREERTOS = $(POD_BASE)/Drivers/FreeRTOS

# ifeq ($(CPU), stm32f4)
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
# endif

vpath += $(PORT)

######################################
# building variables
######################################
# debug build?
DEBUG = 0
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = bin

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/stm32f4xx_it.c \
Core/Src/gpio.c \
Core/Src/stm32f4xx_hal_msp.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Core/Src/usart.c \
Core/Src/tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Core/Src/freertos.c \
Core/Src/stm32f4xx_hal_timebase_tim.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Core/Src/dma.c \
Core/Src/i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
Core/Src/spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c

######################################
# guojun's C source
######################################
# utils
VPATH += Core/utils/Src
C_SOURCES += usec_timer.c eprintf.c cfassert.c static_mem.c configblock.c cal.c filter.c pid.c kalman_filter.c \
	kalman_filter_update.c

# drivers
VPATH += Core/drivers/Src
C_SOURCES += _usart.c _tim.c _i2c.c led.c eeprom.c syslink.c radiolink.c pm.c sensors_bmi088_bmp388.c \
	motors.c tof.c _spi.c flow.c sensors_bmi270_bmp384.c

VPATH += Core/drivers/Bosch/Src
C_SOURCES += bmi08a.c bmi08g.c bmp3.c bmi2.c bmi270.c

VPATH += Core/drivers/Vl53l1/Src
C_SOURCES += vl53l1_api_calibration.c vl53l1_api_core.c vl53l1_api_debug.c vl53l1_api_preset_modes.c vl53l1_api_strings.c \
	vl53l1_api.c vl53l1_core_support.c vl53l1_core.c vl53l1_error_strings.c vl53l1_register_funcs.c vl53l1_silicon_core.c \
	vl53l1_wait.c

# modules
VPATH += Core/modules/Src
C_SOURCES += worker.c system.c mem.c crtp.c console.c comm.c ledseq.c crtp_link.c log.c crtp_platform.c param.c \
	sysload.c commander.c crtp_commander.c crtp_commander_rpyt.c crtp_commander_generic.c stabilizer.c controller.c controller_pid_attitude.c \
	controller_pid.c controller_pid_position.c estimator.c supervisor.c estimator_kalman.c sensors.c power_distribution.c self_test.c

# CMSIS DSP
VPATH += Drivers/CMSIS/DSP/Source/FastMathFunctions
C_SOURCES += arm_cos_f32.c arm_sin_f32.c
VPATH += Drivers/CMSIS/DSP/Source/MatrixFunctions
C_SOURCES += arm_mat_mult_f32.c arm_mat_trans_f32.c
VPATH += Drivers/CMSIS/DSP/Source/CommonTables
C_SOURCES += arm_common_tables.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f405xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32F405xx \
-DUSE_HAL_DRIVER


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IDrivers/CMSIS/DSP/Include \
-ICore/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include

######################################
# guojun's C includes
######################################
C_INCLUDES += \
-ICore/drivers/Inc \
-ICore/utils/Inc \
-ICore/modules/Inc \
-ICore/drivers/Bosch/Inc \
-ICore/drivers/Vl53l1/Inc \
-ICore/drivers/Vl53l1_Platform/Inc


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

######################################
# guojun's CFLAGS
######################################
CFLAGS += -DSTM32F4XX -DSTM32F405xx
CFLAGS += -DSTM32F40_41xxx -DARM_MATH_CM4
CFLAGS += -D__FPU_PRESENT=1
# CFLAGS += -Wno-unused
# CFLAGS += -DSENSOR_INCLUDED_BMI088_BMP388

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F405RGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: bin/ build

bin/:
	mkdir -p bin

build:
	@$(MAKE) --no-print-directory compile

compile: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))


$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "  CC    $@"
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "  CC    $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "  COPY $@"
	@$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "  COPY $@"
	@$(BIN) $< $@	
	
$(BUILD_DIR):
	@mkdir $@

# remote load, require cflib
cload:
	python3 ./cfloader.py -w radio://0/80/2M/E7E7E7E7E7 flash $(PROG).bin stm32-fw

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown
#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
.PHONY: build
