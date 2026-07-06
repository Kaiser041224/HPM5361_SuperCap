# CMakeLists.txt
# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

# Linker scripts - using SDK default
# Uncomment to use custom linker scripts:
# set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/gcc/user_linker.ld)
# set(CUSTOM_SES_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/segger/user_linker.icf)
# set(CUSTOM_IAR_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/iar/user_linker.icf)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(user_app)

# SEGGER RTT (manually included without syscalls to avoid _write conflict)
# UART printf remains on DAPLink; use SEGGER_RTT_printf for RTT output on J-Link
set(RTT_DIR $ENV{HPM_SDK_BASE}/middleware/segger_rtt)
sdk_app_inc(${RTT_DIR}/Config)
sdk_app_inc(${RTT_DIR}/RTT)
sdk_app_src(${RTT_DIR}/RTT/SEGGER_RTT.c)
sdk_app_src(${RTT_DIR}/RTT/SEGGER_RTT_printf.c)

# ============================================================================
# Include Paths
# ============================================================================

# Interface headers (hardware-independent)
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/Interface)

# App headers
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/App/Application/Inc)
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/App/Control/Inc)
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/App/Algorithm/Inc)
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/App/Platform/Inc)
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/App/Debug/Inc)

# Driver headers
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/Driver/hpm_impl)
sdk_app_inc(${CMAKE_CURRENT_SOURCE_DIR}/Driver/IrqProfiler)

# ============================================================================
# Source Files
# ============================================================================

# Driver implementations
sdk_app_src(Driver/hpm_impl/drv_clock.c)
sdk_app_src(Driver/hpm_impl/drv_gpio.c)
sdk_app_src(Driver/hpm_impl/drv_hrpwm.c)
sdk_app_src(Driver/hpm_impl/drv_gptmr.c)
sdk_app_src(Driver/hpm_impl/drv_adc.c)
sdk_app_src(Driver/hpm_impl/drv_trgm.c)
sdk_app_src(Driver/hpm_impl/drv_synt.c)
sdk_app_src(Driver/hpm_impl/drv_mcan.c)
sdk_app_src(Driver/IrqProfiler/irq_profiler.c)

# Interface default stubs
sdk_app_src(Interface/intf_default.c)

# App logic — Algorithm library
sdk_app_src(App/Algorithm/Src/algo_pid.c)
sdk_app_src(App/Algorithm/Src/algo_filter.c)
sdk_app_src(App/Algorithm/Src/algo_rms.c)
sdk_app_src(App/Algorithm/Src/algo_ramp.c)
sdk_app_src(App/Algorithm/Src/algo_ffd.c)
sdk_app_src(App/Algorithm/Src/algo_hyst.c)
sdk_app_src(App/Algorithm/Src/algo_pll.c)

# App logic — Platform
sdk_app_src(App/Platform/Src/app_gpio.c)
sdk_app_src(App/Platform/Src/app_buzzer.c)
sdk_app_src(App/Platform/Src/app_hrpwm.c)
sdk_app_src(App/Platform/Src/app_gptmr.c)
sdk_app_src(App/Platform/Src/app_adc.c)
sdk_app_src(App/Platform/Src/app_analog_signal.c)
sdk_app_src(App/Platform/Src/app_can.c)
sdk_app_src(App/Debug/Src/app_debug.c)
sdk_app_src(App/Debug/Src/app_debug_adc.c)
sdk_app_src(App/Debug/Src/app_debug_can.c)
sdk_app_src(App/Debug/Src/app_debug_hrpwm.c)
sdk_app_src(App/Debug/Src/app_debug_profiler.c)
sdk_app_src(App/Debug/Src/app_debug_rtt.c)
sdk_app_src(App/Debug/Src/app_debug_trap.c)

# App logic — Application (skeleton)
sdk_app_src(App/Application/Src/app_entry.c)
sdk_app_src(App/Application/Src/app_control.c)
sdk_app_src(App/Application/Src/app_comm.c)

# App logic — Control (skeleton)
sdk_app_src(App/Control/Src/ctrl_buckboost.c)
sdk_app_src(App/Control/Src/ctrl_fault.c)

# Main entry
sdk_app_src(App/main.c)

# ============================================================================
# Generate IDE Projects
# ============================================================================

# generate_ide_projects()
