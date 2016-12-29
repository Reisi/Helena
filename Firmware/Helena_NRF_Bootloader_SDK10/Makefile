# The PROJECT_NAME can be changed arbitrarily (it does not depend on anything).
PROJECT_NAME := Helena_bootloader

# The LINKER_SCRIPT is usually in the same directory as the Makefile.
LINKER_SCRIPT := Helena_bootloader_nrf51.ld

# Directories for debug, release and dependency files
DBG_BUILD_DIR := ./bin/debug
DEPS_DIR := ./bin/deps

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
	-DSWI_DISABLE0 \
	-DS130 \
	-DSOFTDEVICE_PRESENT \
	-DBLE_STACK_SUPPORT_REQD \
	-DBSP_DEFINES_ONLY \
	-D__STACK_SIZE=0x600 \
	-D__HEAP_SIZE=0 \
	-DBOARD_CUSTOM \
	-DHELENA_REV21 \

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
	./src/main.c \
	./src/dfu_ble_svc.c \
	./src/s130_1_0_0.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_advdata.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_conn_params.c \
	$(NRF_SDK_INSTALL_ROOT)/components/ble/common/ble_srv_common.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/delay/nrf_delay.c \
	$(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/pstorage/pstorage_raw.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/bootloader.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/bootloader_settings.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/bootloader_util.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/dfu_single_bank.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/dfu_init_template.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/dfu_transport_ble.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/crc16/crc16.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/hci/hci_mem_pool.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/scheduler/app_scheduler.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/timer/app_timer.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/timer/app_timer_appsh.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util/app_error.c \
	$(NRF_SDK_INSTALL_ROOT)/components/libraries/util/nrf_assert.c \
	$(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
	$(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c \
	$(NRF_SDK_INSTALL_ROOT)/components/toolchain/system_nrf51.c \

#assembly files common to all targets
ASM_FILES  += \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain/gcc/gcc_startup_nrf51.s \

#library files common to all targets
LIB_FILES  += \
	$(CMSIS_INSTALL_ROOT)/CMSIS/Lib/GCC/libarm_cortexM0l_math.a

#includes common to all targets
INC_DIRS += \
    ./inc \
    ./mod \
    $(NRF_SDK_INSTALL_ROOT)/examples/bsp \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/common \
    $(NRF_SDK_INSTALL_ROOT)/components/ble/ble_services/ble_dfu \
    $(NRF_SDK_INSTALL_ROOT)/components/device \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/delay \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/hal \
    $(NRF_SDK_INSTALL_ROOT)/components/drivers_nrf/pstorage \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain/gcc \
    $(NRF_SDK_INSTALL_ROOT)/components/toolchain \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/bootloader_dfu/ble_transport \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/crc16 \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/hci \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/scheduler \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/util \
    $(NRF_SDK_INSTALL_ROOT)/components/libraries/timer \
    $(NRF_SDK_INSTALL_ROOT)/components/softdevice/common/softdevice_handler \
    $(NRF_SDK_INSTALL_ROOT)/components/softdevice/s130/headers

# Convert to absolute paths and sort to remove duplicates.
SRC_FILES := $(sort $(foreach f,$(SRC_FILES),$(abspath $(f))))
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
DBG_OBJ := $(addprefix $(DBG_BUILD_DIR)/,$(OBJ))
REL_OBJ := $(addprefix $(REL_BUILD_DIR)/,$(OBJ))

.PHONY: debug
debug: OBJ_DIR := $(DBG_BUILD_DIR)
debug: $(DBG_BUILD_DIR)/$(PROJECT_NAME).hex
	$(call echosize,$(DBG_BUILD_DIR)/$(PROJECT_NAME).elf)

.PHONY: release
release: OBJ_DIR := $(REL_BUILD_DIR)
release: $(REL_BUILD_DIR)/$(PROJECT_NAME).bin $(REL_BUILD_DIR)/$(PROJECT_NAME).hex
	$(call echosize,$(REL_BUILD_DIR)/$(PROJECT_NAME).elf)

.PHONY: all
all: debug release

#DBG_DEP := $(addprefix $(DEPS_DIR)/,$(OBJ:.o=.d))
#-include $(DBG_DEP)

INCLUDES := $(patsubst %,-I%,$(INC_DIRS))

vpath %.c $(SRC_DIRS)
vpath %.s $(SRC_DIRS)

define echosize
	@'$(SIZE)' $1
endef

#$(call gendfupackage,$(DBG_BUILD_DIR)/$(PROJECT_NAME).hex)

define gendfupackage
	@$(NRFUTIL) pkg generate --hw-version 52 --sd-req 0x8C --application-version 1 --application $1 --key-file ../Pussycat.pem $(DBG_BUILD_DIR)/$(PROJECT_NAME).zip
endef

$(DBG_BUILD_DIR)/%.o: %.c | $(DBG_BUILD_DIR)
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(DBG_BUILD_DIR)/%.o: %.s | $(DBG_BUILD_DIR)
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

$(REL_BUILD_DIR)/%.o: %.c | $(REL_BUILD_DIR)
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(REL_BUILD_DIR)/%.o: %.s | $(REL_BUILD_DIR)
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(ASMFLAGS) $(INCLUDES) -c -o $@ $<

$(REL_BUILD_DIR)/$(PROJECT_NAME).hex: $(REL_BUILD_DIR)/$(PROJECT_NAME).elf
	@echo Creating hex file: $(notdir $@)
	@'$(OBJCOPY)' -O ihex $(REL_BUILD_DIR)/$(PROJECT_NAME).elf $@

$(REL_BUILD_DIR)/$(PROJECT_NAME).bin: $(REL_BUILD_DIR)/$(PROJECT_NAME).elf
	@echo Creating bin file: $(notdir $@)
	@'$(OBJCOPY)' -O binary $(REL_BUILD_DIR)/$(PROJECT_NAME).elf $@

$(REL_BUILD_DIR)/$(PROJECT_NAME).elf: $(REL_OBJ)
	@echo Linking ELF file: $(notdir $@)
	@'$(CC)' $(LDFLAGS) $(REL_OBJ) $(LIB_FILES) -lm -o $@

$(DEPS_DIR)/%.d: %.c | $(DEPS_DIR)
	@echo Adding dependency for file: $(notdir $<)
	@'$(CC)' $(CFLAGS) $(INCLUDES) -MM -MF $(patsubst %.o,%.d,$@) -o $@ $<

$(DEPS_DIR)/%.d: %.s | $(DEPS_DIR)
	@echo Adding dependency for file: $(notdir $<)
	@'$(CC)' $(ASMFLAGS) $(INCLUDES) -MM -MF $(patsubst %.o,%.d,$@) -o $@ $<

$(DEPS_DIR) $(DBG_BUILD_DIR) $(REL_BUILD_DIR):; @mkdir $@

.PHONY: clean
clean:
	@rm -rf $(DBG_BUILD_DIR)
	@rm -rf $(REL_BUILD_DIR)
	@rm -rf $(DEPS_DIR)

.PHONY: help
help:
	@echo "The following targets are available:"
	@echo "    (debug)                - Compile with debug flags"
	@echo "    release                - Compile with release flags"
	@echo "    all                    - Compile debug and release targets"
	@echo "    clean                  - Remove object and dependency directories"

# This is a special target that tells make to delete a file if an error occurs
# while the file is being generated.
.DELETE_ON_ERROR:
