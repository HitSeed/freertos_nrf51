FREERTOS_SOURCE_DIR=../../../../FreeRTOSV8.2.1/FreeRTOS/Source/
SDK_PATH = ../../../../nRF51_SDK_8/components/

PROJECT_NAME = FreeRTOS-Demo
DEVICE_VARIANT := xxac
USE_SOFTDEVICE := S110

OUTPUT_FILENAME := $(PROJECT_NAME)_$(USE_SOFTDEVICE)_$(DEVICE_VARIANT)

# with this the SDK loads custom_boad.h for board definitions
BOARD := BOARD_CUSTOM
TARGET_CHIP := NRF51822_QFAC_CA

LINKER_SCRIPT=$(SDK_PATH)softdevice/$(USE_SOFTDEVICE)/toolchain/armgcc/armgcc_$(USE_SOFTDEVICE)_nrf51422_$(DEVICE_VARIANT).ld



ASM_SOURCE_PATHS = $(SDK_PATH)toolchain/gcc/


CFLAGS := -DBLE_STACK_SUPPORT_REQD -DSPI_MASTER_0_ENABLE -DNEED_PSTORAGE -D${USE_SOFTDEVICE} -DUSE_LFCLK_RC
CFLAGS += -Os -g -Wall
LDFLAGS := -Wl,--gc-sections
#LDFLAGS += --specs=nano.specs
#ASMFLAGS := -D__HEAP_SIZE=0

LIBRARY_PATHS += .

INCLUDEPATHS += -I.
INCLUDEPATHS += -I${FREERTOS_SOURCE_DIR}/include
INCLUDEPATHS += -I${FREERTOS_SOURCE_DIR}/portable/GCC/nRF51_SD
INCLUDEPATHS += -I$(SDK_PATH)device
INCLUDEPATHS += -I$(SDK_PATH)drivers_nrf/hal
INCLUDEPATHS += -I$(SDK_PATH)libraries/util
INCLUDEPATHS += -I$(SDK_PATH)softdevice/common/softdevice_handler
INCLUDEPATHS += -I$(SDK_PATH)softdevice/${USE_SOFTDEVICE}/headers/
INCLUDEPATHS += -I$(SDK_PATH)toolchain/gcc/
INCLUDEPATHS += -I$(SDK_PATH)toolchain/
INCLUDEPATHS += -I$(SDK_PATH)libraries/timer/

C_SOURCE_PATHS += ${FREERTOS_SOURCE_DIR}
C_SOURCE_PATHS += ${FREERTOS_SOURCE_DIR}/portable/MemMang
C_SOURCE_PATHS += ${FREERTOS_SOURCE_DIR}/portable/GCC/nRF51_SD
C_SOURCE_PATHS += $(SDK_PATH)drivers_nrf/hal
C_SOURCE_PATHS += $(SDK_PATH)softdevice/common/softdevice_handler/
C_SOURCE_PATHS += $(SDK_PATH)toolchain/
C_SOURCE_PATHS += $(SDK_PATH)libraries/timer

# List all source files the application uses.
C_SOURCE_FILES = main.c
C_SOURCE_FILES += nrf51_freertos.c
C_SOURCE_FILES += softdevice_handler.c
C_SOURCE_FILES += nrf_delay.c
C_SOURCE_FILES += app_timer.c

#FreeRTOS core files
FREERTOS_SOURCE_FILES += list.c
FREERTOS_SOURCE_FILES += queue.c
FREERTOS_SOURCE_FILES += tasks.c
FREERTOS_SOURCE_FILES += port.c
#See the FreeRTOS documentation to choose a suitable memory allocation scheme for your app
FREERTOS_SOURCE_FILES += heap_4.c
#FreeRTOS optional files
FREERTOS_SOURCE_FILES += timers.c
FREERTOS_SOURCE_FILES += croutine.c
FREERTOS_SOURCE_FILES += event_groups.c
C_SOURCE_FILES += $(FREERTOS_SOURCE_FILES)



MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO :=
else
NO_ECHO := @
endif

# Toolchain commands
CC                      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS                      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR                      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD                      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM                      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP                 := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY                 := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE                    := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

TARFILES := FreeRTOSV8.2.1/FreeRTOS/Source/portable/GCC/nRF51_SD/port.c\
FreeRTOSV8.2.1/FreeRTOS/Source/portable/GCC/nRF51_SD/portmacro.h\
FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC/FreeRTOSConfig.h\
FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC/main.c\
FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC/nrf51_freertos.c\
FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC/Makefile\
FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC/Makefile.common\
FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC/README.md\
FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC/CORTEX_M0_nRF51_GCC.cbp\


tar:
	cd ../../../..; tar czvf freertos_nrf51.tgz $(TARFILES)

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))


#include $(SDK_PATH)/toolchain/gcc/Makefile.common
include ./Makefile.common
