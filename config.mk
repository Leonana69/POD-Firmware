######################################
# guojun's define
######################################

DEBUG = 0
BUILD_DIR = bin
OPENOCD				?= openocd
OPENOCD_INTERFACE	?= interface/stlink-v2.cfg
OPENOCD_TARGET    	?= target/stm32f4x.cfg
OPENOCD_CMDS		?=
LOAD_ADDRESS		?= 0x8004000
PROG				?= $(BUILD_DIR)/$(TARGET)
PLATFORM 			?= POD
######################################
# guojun's C source
######################################
# VPATH += Core/Src
# C_SOURCES += dma.c freertos.c gpio.c i2c.c main.c spi.c stm32f4xx_hal_msp.c stm32f4xx_hal_timebase_tim.c \
# 	stm32f4xx_it.c system_stm32f4xx.c tim.c usart.c

# utils
VPATH += Core/utils/Src
C_SOURCES += usec_timer.c eprintf.c cfassert.c static_mem.c configblock.c cal.c filter.c pid.c kalman_filter.c \
	kalman_filter_update.c

# drivers
VPATH += Core/drivers/Src
C_SOURCES += _usart.c _tim.c _i2c.c led.c eeprom.c syslink.c radiolink.c pm.c sensors_bmi088_bmp388.c \
	tof.c _spi.c flow.c sensors_bmi270_bmp384.c motors_pwm.c usblink.c front_dis.c

ifeq ($(PLATFORM), POD)
C_SOURCES += motors_dshot.c
CFLAGS += -DPOD
endif

VPATH += Core/drivers/Bosch/Src
C_SOURCES += bmi08a.c bmi08g.c bmp3.c bmi2.c bmi270.c

VPATH += Core/drivers/Vl53l1/Src
C_SOURCES += vl53l1_api_calibration.c vl53l1_api_core.c vl53l1_api_debug.c vl53l1_api_preset_modes.c vl53l1_api_strings.c \
	vl53l1_api.c vl53l1_core_support.c vl53l1_core.c vl53l1_error_strings.c vl53l1_register_funcs.c vl53l1_silicon_core.c \
	vl53l1_wait.c

VPATH += Core/drivers/Opt3101
C_SOURCES += opt3101.c

# modules
VPATH += Core/modules/Src
C_SOURCES += worker.c system.c mem.c crtp.c console.c comm.c ledseq.c crtp_link.c log.c param.c \
	sysload.c commander.c crtp_commander.c crtp_commander_rpyt.c crtp_commander_generic.c stabilizer.c controller.c controller_pid_attitude.c \
	controller_pid.c controller_pid_position.c estimator.c supervisor.c estimator_kalman.c sensors.c power_distribution.c self_test.c \
	motors.c crtp_platform.c

# CMSIS DSP
VPATH += Drivers/CMSIS/DSP/Source/FastMathFunctions
C_SOURCES += arm_cos_f32.c arm_sin_f32.c
VPATH += Drivers/CMSIS/DSP/Source/MatrixFunctions
C_SOURCES += arm_mat_mult_f32.c arm_mat_trans_f32.c
VPATH += Drivers/CMSIS/DSP/Source/CommonTables
C_SOURCES += arm_common_tables.c

######################################
# guojun's C includes
######################################
C_INCLUDES += \
-IDrivers/CMSIS/DSP/Include \
-ICore/drivers/Inc \
-ICore/utils/Inc \
-ICore/modules/Inc \
-ICore/drivers/Bosch/Inc \
-ICore/drivers/Vl53l1/Inc \
-ICore/drivers/Opt3101 \
-ICore/drivers/Vl53l1_Platform/Inc

######################################
# guojun's CFLAGS
######################################
CFLAGS += -DSTM32F4XX -DSTM32F40_41xxx
CFLAGS += -DARM_MATH_CM4
CFLAGS += -D__FPU_PRESENT=1U
# CFLAGS += -DDEBUG_PRINT_ON_UART
# CFLAGS += -Wno-unused

# default action: build all
all: bin/ build

# remote load, require cflib
cload:
	python3 ./cfloader.py -w radio://0/80/2M/E7E7E7E7E7 flash $(PROG).bin stm32-fw

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown