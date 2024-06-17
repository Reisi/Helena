# The PROJECT_NAME can be changed arbitrarily (it does not depend on anything).
PROJECT_NAME := Helena

# release is the standard output, to compile debug configuration override variable in command line
TYPE = RELEASE

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

#source common to all targets
SRC_FILES_SIZE += \
	./ble/btle.c \
	./ble/link_management.c \
	./ble/ble_scanning.c \
	./ble/ble_hps.c \
	./ble/ble_hids_c.c \
	./ble/ble_hps_c.c \
	./ble/ble_gatt_queue.c \
	./ble/ble_hps_discovery.c \
	./base/main.c \
	./base/mode_management.c \
	./com/comreloaded.c \
	./com/com_message_handling.c \
	./debug/debug.c \
	./debug/log.c \
	./debug/watchdog.c \
	./hmi/hmi.c \
	./util/data_storage.c \

SRC_FILES_SPEED += \
    ./mod/dfu_app_handler_pm.c \
	./mod/peer_manager/gatts_cache_manager.c \
	./mod/peer_manager/peer_database.c \
	./mod/peer_manager/id_manager.c \
	./mod/peer_manager/gatt_cache_manager.c \
	./mod/peer_manager/gattc_cache_manager.c \
	./mod/peer_manager/peer_data.c \
	./mod/peer_manager/peer_data_storage.c \
	./mod/peer_manager/peer_id.c \
	./mod/peer_manager/peer_manager.c \
	./mod/peer_manager/pm_buffer.c \
	./mod/peer_manager/pm_mutex.c \
	./mod/peer_manager/security_dispatcher.c \
	./mod/peer_manager/security_manager.c \
	./mod/ble_advertising.c \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu.c \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dis/ble_dis.c \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_nus/ble_nus.c \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_advdata.c \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_conn_params.c \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_conn_state.c \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_srv_common.c \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/delay/nrf_delay.c \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/timer/nrf_drv_timer.c \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/wdt/nrf_drv_wdt.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/bootloader_util.c \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/fds/fds.c \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/fstorage/fstorage.c \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/timer/app_timer.c \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/util/app_error.c \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/util/app_util_platform.c \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/util/sdk_mapped_flags.c \
    $(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
	$(NRF_SDK_INSTALL_ROOT)/components/toolchain/system_nrf51.c \

#assembly files common to all targets
ASM_FILES  += \
	./mod/gcc_startup_nrf51.S \
	#$(NRF_SDK_INSTALL_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.s \

#library files common to all targets
LIB_FILES  += \
#	$(CMSIS_INSTALL_ROOT)/CMSIS/Lib/GCC/libarm_cortexM0l_math.a

#includes common to all targets
INC_DIRS += \
    ./base \
    ./ble \
    ./mod \
    ./com \
    ./debug \
    ./remotes \
    ./hmi \
    ./boards \
    ./util \
    ./mod/peer_manager \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dfu \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dis \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_nus \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/common \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/common \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/delay \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/hal \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/wdt \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/timer \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/gpiote \
    $(NRF_SDK_INSTALL_ROOT)/components/device \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/experimental_section_vars \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/fds \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/fstorage \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/timer \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/util \
    $(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain/CMSIS/Include \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain/gcc \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain \

#debug variant
ifeq ($(TYPE),DEBUG)
OUTPUT_DIRECTORY := bin_debug

SRC_FILES_SIZE += \
	./debug/dbg_cmd_log.c \
	./debug/dbg_cmd_mem.c \
	./debug/dbg_cmd_reset.c \
	./debug/dbg_cmd_wdt.c \
	./ext/segger/RTT/SEGGER_RTT.c \
	./ext/segger/RTT/SEGGER_RTT_printf.c \

SRC_FILES_SPEED += \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/crc16/crc16.c \

INC_DIRS += \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/crc16 \
    ./ext/segger/RTT \

CFLAGS += \
	-DDEBUG_EXT \
	-DDEBUG \

ASMFLAGS += \
	-DDEBUG_EXT \
	-DDEBUG

OPT = -g3

# release variant
else
OUTPUT_DIRECTORY := bin

OPT = -g3
endif

# C flags common to all targets
CFLAGS += \
	$(OPT) \
	-mcpu=cortex-m0 \
	-mthumb \
	-mabi=aapcs \
	--std=gnu99 \
	-Wall \
	-mfloat-abi=soft \
	-ffunction-sections \
	-fdata-sections \
	-fno-strict-aliasing \
	-fno-builtin \
	--short-enums \
	-DNRF51 \
	-DSOFTDEVICE_PRESENT \
	-DBLE_STACK_SUPPORT_REQD \

# Assembler flags common to all targets
ASMFLAGS += \
	$(OPT) \
	-x assembler-with-cpp \
	-mcpu=cortex-m0 \
	-mthumb \
	-DNRF51 \
	-DSOFTDEVICE_PRESENT \
	-DBLE_STACK_SUPPORT_REQD \

# Linker flags common to all targets
LDFLAGS += \
	-Xlinker \
	-Map=$(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).map \
	-mthumb \
	-mabi=aapcs \
	-T$(LINKER_SCRIPT) \
	-mcpu=cortex-m0 \
	-mfloat-abi=soft \
	-Wl,--gc-sections \
	--specs=nano.specs \
    -lc \
    -lnosys \

# add board specific makefile
include boards/$(MAKECMDGOALS)/makefile_$(MAKECMDGOALS)

# Convert to absolute paths and sort to remove duplicates.
SRC_FILES_SPEED := $(sort $(foreach f,$(SRC_FILES_SPEED),$(abspath $(f))))
SRC_FILES_SIZE := $(sort $(foreach f,$(SRC_FILES_SIZE),$(abspath $(f))))
INC_DIRS := $(sort $(foreach d,$(INC_DIRS),$(abspath $(d))))
SRC_DIRS := $(dir $(SRC_FILES_SPEED) $(SRC_FILES_SIZE) $(ASM_FILES))
SRC_FILE_NAMES_SPEED := $(notdir $(SRC_FILES_SPEED) $(ASM_FILES))
SRC_FILE_NAMES_SIZE := $(notdir $(SRC_FILES_SIZE))

# This directory needs to be added after the main list is sorted because it
# contains its own copy of 'nrf_drv_config.h' that will conflict with the
# project's local version if it's found first.
INC_DIRS += \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/config \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu \

# Convert each source file name into the form '$(OBJ_DIR)/$(SRC_FILE).o'.
OBJ_SPEED := $(patsubst %,%.o,$(basename $(SRC_FILE_NAMES_SPEED)))
OBJ_SIZE := $(patsubst %,%.o,$(basename $(SRC_FILE_NAMES_SIZE)))
OBJ := $(addprefix $(OUTPUT_DIRECTORY)/$(MAKECMDGOALS)/,$(OBJ_SPEED))
OBJ += $(addprefix $(OUTPUT_DIRECTORY)/$(MAKECMDGOALS)/,$(OBJ_SIZE))

.PHONY: $(MAKECMDGOALS)
$(MAKECMDGOALS): $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).hex
	$(call gendfupackage, $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS))
	$(call echosize, $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf)

INCLUDES := $(patsubst %,-I%,$(INC_DIRS))

vpath %.c $(SRC_DIRS)
vpath %.S $(SRC_DIRS)

define echosize
	@'$(SIZE)' $1
endef

$(OUTPUT_DIRECTORY)/$(MAKECMDGOALS)/%.o: %.c | $(OUTPUT_DIRECTORY)/$(MAKECMDGOALS)/
	@echo Compiling file for $(if $(findstring $(basename $(notdir $<)),$(SRC_FILE_NAMES_SIZE)),size,speed): $(notdir $<)
	@'$(CC)' $(CFLAGS) $(if $(findstring $(basename $(notdir $<)),$(SRC_FILE_NAMES_SIZE)),-Os,-O3) $(INCLUDES) -c -o $@ $<

$(OUTPUT_DIRECTORY)/$(MAKECMDGOALS)/%.o: %.S | $(OUTPUT_DIRECTORY)/$(MAKECMDGOALS)/
	@echo Assembling file: $(notdir $<)
	@'$(CC)' $(ASMFLAGS) -O3 $(INCLUDES) -c -o $@ $<

$(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).hex: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf
	@echo Creating hex file: $(notdir $@)
	@'$(OBJCOPY)' -O ihex $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf $@

$(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).bin: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf
	@echo Creating bin file: $(notdir $@)
	@'$(OBJCOPY)' -O binary $(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf $@

$(OUTPUT_DIRECTORY)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf: $(OBJ)
	@echo Linking ELF file: $(notdir $@)
	@'$(CC)' $(LDFLAGS) $(OBJ) $(LIB_FILES) -lm -o $@

$(OUTPUT_DIRECTORY)/$(MAKECMDGOALS)/:
	mkdir -p $@
