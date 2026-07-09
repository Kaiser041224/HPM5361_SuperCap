SHELL := /bin/bash

# ============================================================================
# Paths
# ============================================================================
ROOT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
APP_DIR := $(ROOT_DIR)
BOARDS_DIR := $(ROOT_DIR)/Board
BUILD_DIR := $(ROOT_DIR)/build
OUTPUT_DIR := $(ROOT_DIR)/output
PROJECT_NAME ?= $(notdir $(ROOT_DIR))

# ============================================================================
# Host Path Configuration (for Ozone debug outside container)
# ============================================================================
HOST_WORKSPACE_DIR ?= D:/Codes/HPM_dev/Alliance-HPM-Dev
SDK_DIR := $(if $(HPM_SDK_BASE),$(HPM_SDK_BASE),$(ROOT_DIR)/../hpm_sdk)

# Debug source path mode:
#   host      - default, remap debug info to HOST_WORKSPACE_DIR for host-side debuggers
#   container - keep debug info as container paths for VS Code Remote/Dev Container F5 debug
DEBUG_PATH_MODE ?= host

# ============================================================================
# Board Selection
# ============================================================================
# Default board (can be overridden: make BOARD=user_board)
BOARD ?= HPM5361_SuperCap_board

# Derive board path
BOARD_DIR := $(BOARDS_DIR)/$(BOARD)
BOARD_SEARCH_PATH := $(BOARDS_DIR)

# ============================================================================
# Toolchain & Build Options
# ============================================================================
RV_ARCH ?= rv32imafdc
RV_ABI ?= ilp32d
CMAKE_BUILD_TYPE ?= Debug
HPM_BUILD_TYPE ?= flash_xip
GENERATOR ?= Ninja
LAST_BUILD_LOG := $(BUILD_DIR)/last_build.log

# Optimization level (separate Debug/Release)
# Usage: make build OPT_LEVEL_DBG=-Og OPT_LEVEL_REL=-Ofast
# Debug:  -O0 (default, best debugging experience)
# Release: -O3 (default, maximum speed)
OPT_LEVEL_DBG ?= -O3
OPT_LEVEL_REL ?= -O3
# Extra flags
DBG_EXTRA_FLAGS ?= -g3 -gdwarf-4 -fno-omit-frame-pointer -fno-optimize-sibling-calls
REL_EXTRA_FLAGS ?= -DNDEBUG

ifneq ($(origin OPT_LEVEL), undefined)
ifneq ($(origin OPT_LEVEL_DBG), command line)
OPT_LEVEL_DBG := $(OPT_LEVEL)
endif
ifneq ($(origin OPT_LEVEL_REL), command line)
OPT_LEVEL_REL := $(OPT_LEVEL)
endif
endif

# ============================================================================
# Flash Configuration
# ============================================================================
FLASH_TOOL ?= openocd
OPENOCD_BIN ?= $(if $(HPM_OPENOCD_PREFIX),$(HPM_OPENOCD_PREFIX)/bin/openocd,openocd)
OCD_SCRIPTS ?= $(if $(HPM_OCD_SCRIPTS),$(HPM_OCD_SCRIPTS),$(ROOT_DIR)/../hpm_sdk/boards/openocd)
PROBE_CFG ?= probes/cmsis_dap.cfg
SOC_CFG ?= soc/hpm5300.cfg
BOARD_CFG ?= $(ROOT_DIR)/Board/HPM5361_SuperCap_board/HPM5361_SuperCap_board.cfg
OPENOCD_SPEED ?= 1000
OPENOCD_RISCV_TIMEOUT ?= 60
JLINK_DEVICE ?= HPM5361xEGx
JLINK_IF ?= JTAG
JLINK_SPEED ?= 1000

# ============================================================================
# Scripts (relative paths)
# ============================================================================
BUILD_UI_SCRIPT := $(ROOT_DIR)/../tools/scripts/build_ui.sh
FLASH_SCRIPT := $(ROOT_DIR)/../tools/scripts/flash_target.sh

# ============================================================================
# CMake Arguments
# ============================================================================
ifeq ($(DEBUG_PATH_MODE),container)
  DEBUG_PREFIX_MAP := -fdebug-prefix-map=$(ROOT_DIR)=$(ROOT_DIR) -fdebug-prefix-map=$(SDK_DIR)=$(SDK_DIR)
else
  DEBUG_PREFIX_MAP := -fdebug-prefix-map=$(ROOT_DIR)=$(HOST_WORKSPACE_DIR)/$(PROJECT_NAME) -fdebug-prefix-map=$(SDK_DIR)=$(HOST_WORKSPACE_DIR)/hpm_sdk
endif

# Build extra C flags with optional optimization override
EXTRA_C_FLAGS := $(DEBUG_PREFIX_MAP)

# Optimization level flags for Debug/Release builds
OPT_CMAKE_ARGS := -DCMAKE_C_FLAGS_DEBUG="$(OPT_LEVEL_DBG) -g" -DCMAKE_C_FLAGS_RELEASE="$(OPT_LEVEL_REL) -DNDEBUG"

CMAKE_ARGS := \
	-G$(GENERATOR) \
	-DBOARD=$(BOARD) \
	-DBOARD_SEARCH_PATH=$(BOARD_SEARCH_PATH) \
	-DRV_ARCH=$(RV_ARCH) \
	-DRV_ABI=$(RV_ABI) \
	-DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
	-DHPM_BUILD_TYPE=$(HPM_BUILD_TYPE) \
	-DCMAKE_C_FLAGS="$(EXTRA_C_FLAGS)" \
	$(OPT_CMAKE_ARGS)

# ============================================================================
# Phony Targets
# ============================================================================
.PHONY: all configure build build-core artifacts artifacts-core flash flash-openocd debug-openocd flash-jlink clean distclean rebuild help list-boards banner

all: artifacts

# ============================================================================
# Build Targets
# ============================================================================
configure:
	@if [ ! -d "$(BOARD_DIR)" ]; then \
		echo "Error: board directory not found: $(BOARD_DIR)"; \
		echo "Available boards:"; \
		ls $(BOARDS_DIR) 2>/dev/null || echo "  (none)"; \
		echo "Use: make BOARD=<board_name>"; \
		exit 1; \
	fi
	cmake -S $(APP_DIR) -B $(BUILD_DIR) $(CMAKE_ARGS)

build-core: configure
	@echo ""
	@echo "============================================================"
	@echo "[BUILD] Start compiling target"
	@echo "============================================================"
	cmake --build $(BUILD_DIR) -j

build:
	@mkdir -p $(BUILD_DIR); \
	status=0; \
	set -o pipefail; \
	$(MAKE) --no-print-directory build-core 2>&1 | tee $(LAST_BUILD_LOG); \
	status=$$?; \
	$(MAKE) --no-print-directory banner BUILD_STATUS=$$status BUILD_ACTION=build BUILD_LOG="$(LAST_BUILD_LOG)"; \
	exit $$status

artifacts-core: build-core
	@echo ""
	@echo "============================================================"
	@echo "[ARTIFACTS] Copying to output/"
	@echo "============================================================"
	@mkdir -p $(OUTPUT_DIR)
	@for f in $(BUILD_DIR)/output/demo.*; do \
		if [ -f "$$f" ]; then \
			ext=$${f##*.}; \
			cp "$$f" "$(OUTPUT_DIR)/$(PROJECT_NAME).$$ext"; \
			echo "  -> $(OUTPUT_DIR)/$(PROJECT_NAME).$$ext"; \
		fi; \
	done

artifacts:
	@mkdir -p $(BUILD_DIR); \
	status=0; \
	set -o pipefail; \
	$(MAKE) --no-print-directory artifacts-core 2>&1 | tee $(LAST_BUILD_LOG); \
	status=$$?; \
	$(MAKE) --no-print-directory banner BUILD_STATUS=$$status BUILD_ACTION=artifacts BUILD_LOG="$(LAST_BUILD_LOG)"; \
	exit $$status

# ============================================================================
# Flash Targets
# ============================================================================
flash: flash-$(FLASH_TOOL)

flash-openocd:
	@echo "[FLASH] Using OpenOCD + CMSIS-DAP/DAPLink..."
	$(OPENOCD_BIN) \
		-s $(OCD_SCRIPTS) \
		-f $(PROBE_CFG) \
		-f $(SOC_CFG) \
		-f $(BOARD_CFG) \
		-c "adapter speed $(OPENOCD_SPEED)" \
		-c "riscv set_command_timeout_sec $(OPENOCD_RISCV_TIMEOUT)" \
		-c "reset_config srst_only srst_nogate connect_deassert_srst" \
		-c "init; reset halt; program $(OUTPUT_DIR)/$(PROJECT_NAME).elf verify reset exit"

debug-openocd:
	@echo "[DEBUG] Starting OpenOCD GDB server with CMSIS-DAP/DAPLink..."
	$(OPENOCD_BIN) \
		-s $(OCD_SCRIPTS) \
		-f $(PROBE_CFG) \
		-f $(SOC_CFG) \
		-f $(BOARD_CFG) \
		-c "adapter speed $(OPENOCD_SPEED)" \
		-c "riscv set_command_timeout_sec $(OPENOCD_RISCV_TIMEOUT)" \
		-c "reset_config srst_only srst_nogate connect_deassert_srst"

flash-jlink:
	@echo "[FLASH] Using J-Link..."
	@echo "device $(JLINK_DEVICE)" > /tmp/jlink_flash.jlink
	@echo "if $(JLINK_IF)" >> /tmp/jlink_flash.jlink
	@echo "speed $(JLINK_SPEED)" >> /tmp/jlink_flash.jlink
	@echo "loadfile $(OUTPUT_DIR)/$(PROJECT_NAME).bin 0x8003000" >> /tmp/jlink_flash.jlink
	@echo "r" >> /tmp/jlink_flash.jlink
	@echo "q" >> /tmp/jlink_flash.jlink
	JLinkExe /tmp/jlink_flash.jlink

# ============================================================================
# Clean Targets
# ============================================================================
clean:
	rm -rf $(BUILD_DIR) $(OUTPUT_DIR) $(ROOT_DIR)/.cache

distclean: clean
	@echo "Deep clean complete."

rebuild: clean build

# ============================================================================
# Info Targets
# ============================================================================
list-boards:
	@echo "Available boards in $(BOARDS_DIR):"
	@echo ""
	@for d in $(BOARDS_DIR)/*/; do \
		if [ -d "$$d" ]; then \
			bname=$$(basename "$$d"); \
			echo "  - $$bname"; \
		fi; \
	done
	@echo ""
	@echo "Current board: $(BOARD)"
	@echo "Use: make BOARD=<board_name> build"

help:
	@echo "HPM User Template - Build System"
	@echo ""
	@echo "Usage: make [TARGET] [OPTIONS]"
	@echo ""
	@echo "Targets:"
	@echo "  build         Configure and compile (default)"
	@echo "  artifacts     Build and copy outputs to output/"
	@echo "  flash         Flash firmware (default: openocd)"
	@echo "  flash-openocd Flash using OpenOCD"
	@echo "  debug-openocd Start OpenOCD GDB server using DAPLink"
	@echo "  flash-jlink   Flash using J-Link"
	@echo "  clean         Remove build and output directories"
	@echo "  distclean     Deep clean"
	@echo "  rebuild       Clean and rebuild"
	@echo "  list-boards   Show available boards"
	@echo "  help          Show this help"
	@echo ""
	@echo "Options:"
	@echo "  BOARD=<name>             Board name (default: HPM5361_SuperCap_board)"
	@echo "  CMAKE_BUILD_TYPE=<type>  Debug or Release (default: Debug)"
	@echo "  HPM_BUILD_TYPE=<type>    flash_xip, flash_sdram_xip, etc."
	@echo "  OPT_LEVEL_DBG=<level>    Debug optimization: -O0 (default), -Og, -O1..."
	@echo "  OPT_LEVEL_REL=<level>    Release optimization: -O3 (default), -O2, -Os, -Ofast"
	@echo "  FLASH_TOOL=<tool>        openocd or jlink (default: openocd)"
	@echo "  OPENOCD_SPEED=<kHz>      OpenOCD adapter speed (default: 1000)"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make BOARD=user_board build"
	@echo "  make CMAKE_BUILD_TYPE=Release artifacts"
	@echo "  make OPT_LEVEL_DBG=-Og build"
	@echo "  make FLASH_TOOL=jlink flash"

# ============================================================================
# Banner
# ============================================================================
banner:
	@if [ -x "$(BUILD_UI_SCRIPT)" ]; then \
		PROJECT_NAME="$(PROJECT_NAME)" \
		BUILD_ACTION="$(BUILD_ACTION)" \
		BUILD_STATUS="$(BUILD_STATUS)" \
		OUTPUT_DIR="$(OUTPUT_DIR)" \
		BUILD_LOG="$(LAST_BUILD_LOG)" \
		APP_DIR="$(APP_DIR)" \
		BUILD_DIR="$(BUILD_DIR)" \
		BOARD="$(BOARD)" \
		BOARD_SEARCH_PATH="$(BOARD_SEARCH_PATH)" \
		RV_ARCH="$(RV_ARCH)" \
		RV_ABI="$(RV_ABI)" \
		CMAKE_BUILD_TYPE="$(CMAKE_BUILD_TYPE)" \
		HPM_BUILD_TYPE="$(HPM_BUILD_TYPE)" \
		bash $(BUILD_UI_SCRIPT) --action $(BUILD_ACTION) --status $(BUILD_STATUS); \
	else \
		if [ "$(BUILD_STATUS)" -eq 0 ]; then \
			echo ""; \
			echo "============================================================"; \
			echo "  BUILD SUCCEEDED"; \
			echo "============================================================"; \
		else \
			echo ""; \
			echo "============================================================"; \
			echo "  BUILD FAILED (exit code: $(BUILD_STATUS))"; \
			echo "============================================================"; \
		fi; \
	fi
