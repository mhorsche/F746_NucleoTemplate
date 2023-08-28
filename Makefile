################################################################################
# @file    Makefile
# @author  Max Horsche
#
# @brief   Compiler settings for stm32f746-nucleo board.
#
# @changelog	2015-07-22 - first version
#							2017-02-10 - Several enhancements + project update mode
#							2023-08-11 - STM32F7 HAL update + FreeRTOS update
################################################################################

######################################
# firmware version
######################################
FW_VERSION = v0.0.1


######################################
# target
######################################
TARGET = firmware


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
# https://www.keil.com/support/man/docs/armclang_ref/armclang_ref_chr1383664854780.htm
ifeq ($(DEBUG), 1)
	OPT = -Og -gdwarf-4
else
#	OPT = -Os
#	OPT = -O2
	OPT = -O3
#	OPT = -Og
endif


#######################################
# paths
#######################################
# Build path
BUILD_DIR = .build

######################################
# link script
######################################
LDSCRIPT = STM32F746ZGTx_FLASH.ld


######################################
# source
######################################
# ASM sources
ASM_SOURCES =  \
	Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s

# User Source Code
C_SOURCES =  \
	Src/main.c \
	Src/freertos.c \

# Board Support Package
C_SOURCES +=  \
	Drivers/BSP/stm32f7xx_common.c \
	Drivers/BSP/Components/stlink/stlink.c \

# FreeRTOS User Tasks
C_SOURCES +=  \
	Src/freertos_task_logging.c \
	Src/freertos_task_led.c \
	Src/freertos_task_iperf.c \
	# Src/freertos_mqtt_task.c \
	# Src/freertos_modbus_task.c \


# STM HAL sources
C_SOURCES +=  \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_eth.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rng.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c \
	Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c \
	Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c 
# Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c \
# Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c \
# Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c \
# Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c \
# Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c \
# Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c \
# Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c \
# Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c \


# FreeRTOS sources
C_SOURCES +=  \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/croutine.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/event_groups.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/list.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/queue.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/stream_buffer.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/tasks.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/timers.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/portable/MemMang/heap_5.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1/port.c 	
# Middlewares/Third_Party/FreeRTOS/CMSIS_RTOS_V2/cmsis_os2.c \

# FreeRTOS+Utilities sources
C_SOURCES +=  \
	Middlewares/Third_Party/FreeRTOS/logging/printf.c 
# Middlewares/Third_Party/FreeRTOS/logging/printf_stdarg.c 

# FreeRTOS+TCP sources
C_SOURCES +=  \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_ARP.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_BitConfig.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_DHCP.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_DHCPv6.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_DNS.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_DNS_Cache.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_DNS_Callback.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_DNS_Networking.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_DNS_Parser.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_ICMP.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IP.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IP_Timers.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IP_Utils.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IPv4.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IPv4_Sockets.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IPv4_Utils.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IPv6.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IPv6_Sockets.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_IPv6_Utils.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_ND.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_RA.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_Routing.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_Sockets.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_Stream_Buffer.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_IP.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_IP_IPv4.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_IP_IPv6.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Reception.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_State_Handling.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_State_Handling_IPv4.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_State_Handling_IPv6.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Transmission.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Transmission_IPv4.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Transmission_IPv6.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Utils.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Utils_IPv4.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_Utils_IPv6.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_TCP_WIN.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_Tiny_TCP.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_UDP_IP.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_UDP_IPv4.c \
  Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/FreeRTOS_UDP_IPv6.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/portable/BufferManagement/BufferAllocation_2.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/portable/NetworkInterface/Common/phyHandling.c \
	Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/portable/NetworkInterface/STM32/NetworkInterface.c
# Middlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/portable/NetworkInterface/STM32Fxx/stm32fxx_hal_eth.c 

# FreeRTOS+coreMQTT sources
C_SOURCES +=  \
	Middlewares/Third_Party/FreeRTOS/backoffAlgorithm/source/backoff_algorithm.c \
	Middlewares/Third_Party/FreeRTOS/network_transport/freertos_plus_tcp/sockets_wrapper.c \
	Middlewares/Third_Party/FreeRTOS/network_transport/freertos_plus_tcp/using_plaintext/using_plaintext.c \
	Middlewares/Third_Party/FreeRTOS/coreMQTT/source/core_mqtt_serializer.c \
	Middlewares/Third_Party/FreeRTOS/coreMQTT/source/core_mqtt_state.c \
	Middlewares/Third_Party/FreeRTOS/coreMQTT/source/core_mqtt.c 

# FreeRTOS+coreMQTT-Agent sources
C_SOURCES +=  \
	Middlewares/Third_Party/FreeRTOS/coreMQTT-Agent/source/core_mqtt_agent.c \
	Middlewares/Third_Party/FreeRTOS/coreMQTT-Agent/source/core_mqtt_agent_command_functions.c \
	Middlewares/Third_Party/FreeRTOS/mqtt-agent-interface/freertos_agent_message.c \
	Middlewares/Third_Party/FreeRTOS/mqtt-agent-interface/freertos_command_pool.c

# FreeRTOS+coreJSON sources
C_SOURCES +=  \
	Middlewares/Third_Party/FreeRTOS/coreJSON/source/core_json.c


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
CPU = -mcpu=cortex-m7

# fpu: fpv5-sp-d16 (single precision) / fpv5-d16 (double precision)
FPU = -mfpu=fpv5-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
	-DSTM32F7xx \
	-DSTM32F746xx \
	-DUSE_STM32F7XX_NUCLEO_144 \
	-DUSE_HAL_DRIVER \
	-DDUSE_FULL_LL_DRIVER \
	-DFW_VERSION=\"$(FW_VERSION)\" \

ifeq ($(DEBUG), 1)
	C_DEFS += -DDEBUG
endif

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
	-IInc \
	-IDrivers/BSP \
	-IDrivers/STM32F7xx_HAL_Driver/Inc \
	-IDrivers/CMSIS/Include \
	-IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
	-IMiddlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/include \
	-IMiddlewares/Third_Party/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM7/r0p1 \
	-IMiddlewares/Third_Party/FreeRTOS/logging \
	-IMiddlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/include \
	-IMiddlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/portable/Compiler/GCC \
	-IMiddlewares/Third_Party/FreeRTOS/FreeRTOS-Plus-TCP/source/portable/NetworkInterface/include \
	-IMiddlewares/Third_Party/FreeRTOS/coreJSON/source/include \
	-IMiddlewares/Third_Party/FreeRTOS/coreMQTT-Agent/source/include \
	-IMiddlewares/Third_Party/FreeRTOS/mqtt-agent-interface/include \
	-IMiddlewares/Third_Party/FreeRTOS/backoffAlgorithm/source/include \
	-IMiddlewares/Third_Party/FreeRTOS/coreMQTT/source/include \
	-IMiddlewares/Third_Party/FreeRTOS/coreMQTT/source/interface \
	-IMiddlewares/Third_Party/FreeRTOS/network_transport/freertos_plus_tcp \
	-IMiddlewares/Third_Party/FreeRTOS/network_transport/freertos_plus_tcp/using_plaintext \
# -IMiddlewares/Third_Party/FreeRTOS/CMSIS_RTOS_V2 \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
	CFLAGS += -g -gdwarf-4
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


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
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		


#######################################
# clean up
#######################################
clean:
	-rm -fr $(BUILD_DIR)/*
  

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***