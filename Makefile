PROJECT_NAME     := smartCard
TARGETS          := smardCard_nrf52840
OUTPUT_DIRECTORY := target

$(OUTPUT_DIRECTORY)/smardCard_nrf52840.out: \
  LINKER_SCRIPT  := ./tool/smartCard_gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  ./src/Application/main.c \
  ./src/Application/board_spi.c \
  ./src/Application/GUI_Paint.c \
  ./src/Application/multi_button.c \
  ./src/Board_Definition/boards.c \
  ./src/Board_Support/bsp.c \
  ./src/Device/gcc_startup_nrf52840.S \
  ./src/Device/system_nrf52840.c \
  ./src/nRF_BLE/ble_advdata.c \
  ./src/nRF_BLE/ble_conn_params.c \
  ./src/nRF_BLE/ble_conn_state.c \
  ./src/nRF_BLE/ble_srv_common.c \
  ./src/nRF_BLE/nrf_ble_gatt.c \
  ./src/nRF_BLE/nrf_ble_qwr.c \
  ./src/nRF_BLE_Services/ble_lbs.c \
  ./src/nRF_Drivers/app_fifo.c \
  ./src/nRF_Drivers/app_uart_fifo.c \
  ./src/nRF_Drivers/nrf_drv_clock.c \
  ./src/nRF_Drivers/nrf_drv_spi.c \
  ./src/nRF_Drivers/nrf_drv_uart.c \
  ./src/nRF_Drivers/nrf_nvic.c \
  ./src/nRF_Drivers/nrf_soc.c \
  ./src/nRF_Drivers/nrfx_atomic.c \
  ./src/nRF_Drivers/nrfx_clock.c \
  ./src/nRF_Drivers/nrfx_gpiote.c \
  ./src/nRF_Drivers/nrfx_nfct.c \
  ./src/nRF_Drivers/nrfx_prs.c \
  ./src/nRF_Drivers/nrfx_spi.c \
  ./src/nRF_Drivers/nrfx_spim.c \
  ./src/nRF_Drivers/nrfx_timer.c \
  ./src/nRF_Drivers/nrfx_uart.c \
  ./src/nRF_Drivers/nrfx_uarte.c \
  ./src/nRF_Drivers/retarget.c \
  ./src/nRF_Libraries/app_button.c \
  ./src/nRF_Libraries/app_error.c \
  ./src/nRF_Libraries/app_error_handler_gcc.c \
  ./src/nRF_Libraries/app_error_weak.c \
  ./src/nRF_Libraries/app_fifo.c \
  ./src/nRF_Libraries/app_scheduler.c \
  ./src/nRF_Libraries/app_timer2.c \
  ./src/nRF_Libraries/app_uart_fifo.c \
  ./src/nRF_Libraries/app_util_platform.c \
  ./src/nRF_Libraries/drv_rtc.c \
  ./src/nRF_Libraries/hardfault_handler_gcc.c \
  ./src/nRF_Libraries/hardfault_implementation.c \
  ./src/nRF_Libraries/nrf_assert.c \
  ./src/nRF_Libraries/nrf_atfifo.c \
  ./src/nRF_Libraries/nrf_atflags.c \
  ./src/nRF_Libraries/nrf_atomic.c \
  ./src/nRF_Libraries/nrf_balloc.c \
  ./src/nRF_Libraries/nrf_fprintf.c \
  ./src/nRF_Libraries/nrf_fprintf_format.c \
  ./src/nRF_Libraries/nrf_memobj.c \
  ./src/nRF_Libraries/nrf_pwr_mgmt.c \
  ./src/nRF_Libraries/nrf_ringbuf.c \
  ./src/nRF_Libraries/nrf_section_iter.c \
  ./src/nRF_Libraries/nrf_sortlist.c \
  ./src/nRF_Libraries/nrf_strerror.c \
  ./src/nRF_Log/nrf_log_backend_rtt.c \
  ./src/nRF_Log/nrf_log_backend_serial.c \
  ./src/nRF_Log/nrf_log_backend_uart.c \
  ./src/nRF_Log/nrf_log_default_backends.c \
  ./src/nRF_Log/nrf_log_frontend.c \
  ./src/nRF_Log/nrf_log_str_formatter.c \
  ./src/nRF_NFC/nfc_ndef_msg.c \
  ./src/nRF_NFC/nfc_ndef_record.c \
  ./src/nRF_NFC/nfc_platform.c \
  ./src/nRF_NFC/nfc_text_rec.c \
  ./src/nRF_Segger_RTT/SEGGER_RTT.c \
  ./src/nRF_Segger_RTT/SEGGER_RTT_Syscalls_GCC.c \
  ./src/nRF_Segger_RTT/SEGGER_RTT_printf.c \
  ./src/nRF_SoftDevice/nrf_sdh.c \
  ./src/nRF_SoftDevice/nrf_sdh_ble.c \
  ./src/nRF_SoftDevice/nrf_sdh_soc.c \
  ./src/nRF_Util/font8.c \
  ./src/nRF_Util/font12.c \
  ./src/nRF_Util/font12CN.c \
  ./src/nRF_Util/font16.c \
  ./src/nRF_Util/font20.c \
  ./src/nRF_Util/font24.c \
  ./src/nRF_Util/font24CN.c \
  ./src/UTF8-UTF16-converter/utf.c \

# Include folders common to all targets
INC_FOLDERS += ./inc

# Libraries common to all targets
LIB_FILES += \
  ./lib/nfc_t2t_lib_gcc.a \

# Optimization flags
OPT = -O3 -g3

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DAPP_TIMER_V2
CFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_SD_BLE_API_VERSION=7
CFLAGS += -DS140
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += -O3 
CXXDLAGS += -g3

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DAPP_TIMER_V2
ASMFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=7
ASMFLAGS += -DS140
ASMFLAGS += -DSOFTDEVICE_PRESENT

# Linker flags
LDFLAGS += -O3 -g3
LDFLAGS += -mthumb -mabi=aapcs -L./src/modules/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs

smardCard_nrf52840: CFLAGS += -D__HEAP_SIZE=8192
smardCard_nrf52840: CFLAGS += -D__STACK_SIZE=8192
smardCard_nrf52840: ASMFLAGS += -D__HEAP_SIZE=8192
smardCard_nrf52840: ASMFLAGS += -D__STACK_SIZE=8192

LIB_FILES += -lc -lnosys -lm

.PHONY: default help

# Default target - first one defined
default: smardCard_nrf52840

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		smardCard_nrf52840
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := ./tool

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase recover

# Flash the program
flash: default
	nrfjprog -f nrf52 --eraseall
	nrfjprog -f nrf52 --program softdevice/s140_nrf52_7.2.0_softdevice.hex --chiperase --verify
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/smardCard_nrf52840.hex --verify
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

recover:
	nrfjprog --recover