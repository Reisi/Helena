# The PROJECT_NAME can be changed arbitrarily (it does not depend on anything).
PROJECT_NAME := Helena_btldr

# The LINKER_SCRIPT is usually in the same directory as the Makefile.
LINKER_SCRIPT := Helena_btldr_nrf51.ld

# Directories for debug, release and dependency files
OUTPUT_DIR := ./bin

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
	-DSWI_DISABLE0 \
	-D__STACK_SIZE=0x600 \
	-D__HEAP_SIZE=0 \

CFLAGS += \
	-mcpu=cortex-m0 \
	-mthumb \
	-mabi=aapcs \
	--std=gnu99 \
	-Wall \
	-Os \
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
	-Map=$(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).map \
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
	./src/dfu_transport_ble.c \
	./src/dfu_ble_svc.c \
	./src/main.c \
	./boards/$(MAKECMDGOALS)/hmi.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util/app_error.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util/app_error_weak.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/scheduler/app_scheduler.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/timer/app_timer.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/timer/app_timer_appsh.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util/app_util_platform.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/bootloader.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/bootloader_settings.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/bootloader_util.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/crc16/crc16.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/dfu_single_bank.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/dfu_init_template.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/hci/hci_mem_pool.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util/nrf_assert.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/delay/nrf_delay.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/hal/nrf_nvmc.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/pstorage/pstorage_raw.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_advdata.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_conn_params.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_srv_common.c \
	$(NRF_SDK_INSTALL_ROOT)/components/toolchain/system_nrf51.c \
	$(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
	$(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c \
	#./src/s130_2.0.1.c \

#assembly files common to all targets
ASM_FILES  += \
    ./asm/gcc_startup_nrf51.S \

#library files common to all targets
#LIB_FILES  += \
#	$(CMSIS_INSTALL_ROOT)/CMSIS/Lib/GCC/libarm_cortexM0l_math.a

#includes common to all targets
INC_DIRS += \
    ./inc \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dfu \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common \
	$(NRF_SDK_INSTALL_ROOT)/components/device \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/common \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/config \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/delay \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/hal \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/pstorage \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/ble_transport \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/crc16 \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/hci \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/scheduler \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/timer \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util \
	$(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler \
	$(NRF_SDK_INSTALL_ROOT)/components/softdevice/s130/headers \
	$(NRF_SDK_INSTALL_ROOT)/components/softdevice/s130/headers/nrf51 \
	$(NRF_SDK_INSTALL_ROOT)/components/toolchain \
	$(NRF_SDK_INSTALL_ROOT)/components/toolchain/CMSIS/Include \
	$(NRF_SDK_INSTALL_ROOT)/components/toolchain/gcc \

# Convert to absolute paths and sort to remove duplicates.
#SRC_FILES := $(sort $(foreach f,$(SRC_FILES),$(abspath $(f))))
SRC_FILES := $(foreach f,$(SRC_FILES),$(abspath $(f)))
INC_DIRS := $(sort $(foreach d,$(INC_DIRS),$(abspath $(d))))

# This directory needs to be added after the main list is sorted because it
# contains its own copy of 'nrf_drv_config.h' that will conflict with the
# project's local version if it's found first.
INC_DIRS += \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu \

SRC_DIRS := $(dir $(SRC_FILES) $(ASM_FILES))

SRC_FILE_NAMES := $(notdir $(SRC_FILES) $(ASM_FILES))

# Convert each source file name into the form '$(OBJ_DIR)/$(SRC_FILE).o'.
OBJ := $(patsubst %,%.o,$(basename $(SRC_FILE_NAMES)))
RLS_OBJ := $(addprefix $(OUTPUT_DIR)/$(MAKECMDGOALS)/,$(OBJ))

.PHONY: $(MAKECMDGOALS)
$(MAKECMDGOALS): $(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).hex
	$(call gendfupackage, $(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS))
	$(call echosize, $(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf)

INCLUDES := $(patsubst %,-I%,$(INC_DIRS))

vpath %.c $(SRC_DIRS)
vpath %.S $(SRC_DIRS)

define echosize
	@'$(SIZE)' $1
endef

$(OUTPUT_DIR)/$(MAKECMDGOALS)/%.o: %.c | $(OUTPUT_DIR)/$(MAKECMDGOALS)/
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(OUTPUT_DIR)/$(MAKECMDGOALS)/%.o: %.S | $(OUTPUT_DIR)/$(MAKECMDGOALS)/
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(ASMFLAGS) $(INCLUDES) -c -o $@ $<

$(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).hex: $(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf
	@echo Creating hex file: $(notdir $@)
	@'$(OBJCOPY)' -O ihex $(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf $@

$(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).bin: $(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf
	@echo Creating bin file: $(notdir $@)
	@'$(OBJCOPY)' -O binary $(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf $@

$(OUTPUT_DIR)/$(PROJECT_NAME)_$(MAKECMDGOALS).elf: $(RLS_OBJ)
	@echo Linking ELF file: $(notdir $@)
	@'$(CC)' $(LDFLAGS) $(RLS_OBJ) $(LIB_FILES) -lm -o $@

$(OUTPUT_DIR)/$(MAKECMDGOALS)/:
	mkdir -p $@

define gendfupackage
	@$(NRFUTIL) dfu genpkg --bootloader $1.hex \
	                       --softdevice $(NRF_SDK_INSTALL_ROOT)/components/softdevice/s130/hex/s130_nrf51_2.0.1_softdevice.hex \
	                       $1.zip
endef
