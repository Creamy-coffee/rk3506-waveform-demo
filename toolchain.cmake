# CMake 交叉编译工具链：RK3506（Cortex-A7，ARMv7-A hard-float）
set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TC_ROOT "/home/lqnm/rk3506_linux6.1_sdk_v1.2.0_iot_evb/prebuilts/gcc/linux-x86/arm/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf")

set(CMAKE_C_COMPILER   "${TC_ROOT}/bin/arm-none-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${TC_ROOT}/bin/arm-none-linux-gnueabihf-g++")

# Arm 官方工具链自带的 sysroot
set(CMAKE_SYSROOT "${TC_ROOT}/arm-none-linux-gnueabihf/libc")

set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# RK3506 是 Cortex-A7 + NEON：ARMv7-A / hard-float
set(ARCH_FLAGS "-march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT   "${ARCH_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT "${ARCH_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${ARCH_FLAGS} -static -Wl,--gc-sections")
