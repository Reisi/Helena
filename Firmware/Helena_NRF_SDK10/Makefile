# The PROJECT_NAME can be changed arbitrarily (it does not depend on anything).
PROJECT_NAME := Helena_app

# The LINKER_SCRIPT is usually in the same directory as the Makefile.
LINKER_SCRIPT := Helena_app_nrf51.ld

# Directories for debug, release and dependency files
DBG_BUILD_DIR := ./bin/debug

# include definitions of toolchain files and sdk path
include ../Makefile.conf

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

SHAREDFLAGS += \
	-DNRF51 \
	-DS130 \
	-DSOFTDEVICE_PRESENT \
	-DBLE_STACK_SUPPORT_REQD \
	-DMPU6050 \
	-D__STACK_SIZE=0x600 \
	-D__HEAP_SIZE=0 \
	-DARM_MATH_CM0 \
	-DDEBUG \
	-DHELENA_DEBUG_SWD \
	-DHELENA_DEBUG_RTT \
	-DHELENA_DEBUG_FIELD_TESTING

CFLAGS += \
	-mcpu=cortex-m0 \
	-mthumb \
	-mabi=aapcs \
	--std=gnu99 \
	-Wall \
	-O3 \
	-g3 \
	-mfloat-abi=soft \
	-ffunction-sections \
	-fdata-sections \
	-fno-strict-aliasing \
	-fno-builtin \
	--short-enums \
	$(SHAREDFLAGS)

ASMFLAGS += \
	-x assembler-with-cpp \
	-mcpu=cortex-m0 \
	-mthumb \
	$(SHAREDFLAGS)

LDFLAGS += \
	-Xlinker \
	-Map=$(OBJ_DIR)/$(PROJECT_NAME).map \
	-mthumb \
	-mabi=aapcs \
	-T$(LINKER_SCRIPT) \
	-mcpu=cortex-m0 \
	-mfloat-abi=soft \
	-Wl,--gc-sections \
	--specs=nano.specs \
    -lc \
    -lnosys

#source common to all targets
SRC_FILES += \
	./src/ble_cgw.c \
	./src/ble_hids_c.c \
	./src/ble_lcs.c \
	./src/ble_lcs_c.c \
	./src/ble_lcs_ctrlpt.c \
	./src/ble_scanning.h \
	./src/btle.c \
	./src/comreloaded.c \
	./src/com_msg_handling.c \
	./src/custom_board.h \
	./src/debug.c \
	./src/fastmath.c \
	./src/hmi.c \
	./src/i2c.c \
	./src/light.c \
	./src/main.c \
	./src/motion_sensor.c \
	./src/power.c \
	./mod/nrf_drv_twi.c \
	./mod/ble_db_discovery.c \
	./mod/peer_manager/gatt_cache_manager.c \
	./mod/peer_manager/gattc_cache_manager.c \
	./mod/peer_manager/gatts_cache_manager.c \
	./mod/peer_manager/peer_data.c \
	./mod/peer_manager/peer_data_storage.c \
	./mod/peer_manager/peer_database.c \
	./mod/peer_manager/peer_id.c \
	./mod/peer_manager/peer_manager.c \
	./mod/peer_manager/pm_buffer.c \
	./mod/peer_manager/pm_mutex.c \
	./mod/peer_manager/id_manager.c \
	./mod/peer_manager/security_dispatcher.c \
	./mod/peer_manager/security_manager.c \
	./ext/Invensense/eMPL/inv_mpu.c \
	./ext/Invensense/eMPL/inv_mpu_dmp_motion_driver.c \
	./ext/segger/RTT/SEGGER_RTT.c \
	./ext/segger/RTT/SEGGER_RTT_printf.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/ble_advertising/ble_advertising.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dis/ble_dis.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_nus/ble_nus.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_advdata.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_conn_params.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_conn_state.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_srv_common.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/delay/nrf_delay.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/ppi/nrf_drv_ppi.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/pstorage/pstorage.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/timer/nrf_drv_timer.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/wdt/nrf_drv_wdt.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/hal/nrf_adc.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/crc16/crc16.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/fds/fds.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/fstorage/fstorage.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/timer/app_timer.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util/app_util_platform.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util/sdk_mapped_flags.c \
	$(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
	$(NRF_SDK_INSTALL_ROOT)/components/toolchain/system_nrf51.c \

#assembly files common to all targets
ASM_FILES  += \
    ./asm/gcc_startup_nrf51.S \

#library files common to all targets
LIB_FILES  += \
	$(CMSIS_INSTALL_ROOT)/CMSIS/Lib/GCC/libarm_cortexM0l_math.a

#includes common to all targets
INC_DIRS += \
    ./inc \
    ./ext/Invensense/eMPL \
    ./ext/segger/RTT \
    ./mod \
    ./mod/peer_manager \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_advertising \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dis \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_nus \
    $(NRF_SDK_INSTALL_ROOT)/components/device \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/delay \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/common \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/hal \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/pstorage \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/ppi \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/gpiote \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/timer \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/twi_master \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/wdt \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain/CMSIS/Include \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain/gcc \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/crc16 \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/experimental_section_vars \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/fds \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/fstorage \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/scheduler \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/util \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/timer \
    $(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler \
    $(NRF_SDK_INSTALL_ROOT)/components/softdevice/s130/headers \
    $(CMSIS_INSTALL_ROOT)/CMSIS/DSP/Include \
    $(CMSIS_INSTALL_ROOT)/CMSIS/Core/Include

# Convert to absolute paths and sort to remove duplicates.
SRC_FILES := $(sort $(foreach f,$(SRC_FILES),$(abspath $(f))))
INC_DIRS := $(sort $(foreach d,$(INC_DIRS),$(abspath $(d))))

# This directory needs to be added after the main list is sorted because it
# contains its own copy of 'nrf_drv_config.h' that will conflict with the
# project's local version if it's found first.
INC_DIRS += \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/config \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/common \


SRC_DIRS := $(dir $(SRC_FILES) $(ASM_FILES))

SRC_FILE_NAMES := $(notdir $(SRC_FILES) $(ASM_FILES))

# Convert each source file name into the form '$(OBJ_DIR)/$(SRC_FILE).o'.
OBJ := $(patsubst %,%.o,$(basename $(SRC_FILE_NAMES)))
DBG_OBJ := $(addprefix $(DBG_BUILD_DIR)/,$(OBJ))

.PHONY: debug
debug: OBJ_DIR := $(DBG_BUILD_DIR)
debug: $(DBG_BUILD_DIR)/$(PROJECT_NAME).hex
	$(call gendfupackage, $(DBG_BUILD_DIR)/$(PROJECT_NAME))
	$(call echosize, $(DBG_BUILD_DIR)/$(PROJECT_NAME).elf)

.PHONY: all
all: debug

define gendfupackage
	@$(NRFUTIL) dfu genpkg --application $1.hex --sd-req 0x0067 $1.zip
endef

INCLUDES := $(patsubst %,-I%,$(INC_DIRS))

vpath %.c $(SRC_DIRS)
vpath %.S $(SRC_DIRS)

define echosize
	@'$(SIZE)' $1
endef

$(DBG_BUILD_DIR)/%.o: %.c | $(DBG_BUILD_DIR)
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(DBG_BUILD_DIR)/%.o: %.S | $(DBG_BUILD_DIR)
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(ASMFLAGS) $(INCLUDES) -c -o $@ $<

$(DBG_BUILD_DIR)/$(PROJECT_NAME).hex: $(DBG_BUILD_DIR)/$(PROJECT_NAME).elf
	@echo Creating hex file: $(notdir $@)
	@'$(OBJCOPY)' -O ihex $(DBG_BUILD_DIR)/$(PROJECT_NAME).elf $@

$(DBG_BUILD_DIR)/$(PROJECT_NAME).bin: $(DBG_BUILD_DIR)/$(PROJECT_NAME).elf
	@echo Creating bin file: $(notdir $@)
	@'$(OBJCOPY)' -O binary $(DBG_BUILD_DIR)/$(PROJECT_NAME).elf $@

$(DBG_BUILD_DIR)/$(PROJECT_NAME).elf: $(DBG_OBJ)
	@echo Linking ELF file: $(notdir $@)
	@'$(CC)' $(LDFLAGS) $(DBG_OBJ) $(LIB_FILES) -lm -o $@

.PHONY: clean
clean:
	@rm -rf $(DBG_BUILD_DIR)

.PHONY: help
help:
	@echo "The following targets are available:"
	@echo "    (debug)                - Compile with debug flags"
	@echo "    all                    - Compile debug target"
