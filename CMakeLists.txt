cmake_minimum_required(VERSION 3.10)
project(waveform_demo C)
include(third_party/liquid.cmake)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# LVGL 源码目录（默认取同级的 lvgl-release-v9.1，可在命令行覆盖）
set(LVGL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lvgl-release-v9.1"
    CACHE PATH "LVGL source directory")

if(NOT EXISTS "${LVGL_DIR}/lvgl.h")
    message(FATAL_ERROR "找不到 LVGL 源码：${LVGL_DIR}")
endif()
message(STATUS "LVGL source : ${LVGL_DIR}")

# 让 LVGL 用我们的 lv_conf.h
set(LV_CONF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h"
    CACHE STRING "" FORCE)

# 不编示例和官方 demo，省编译时间
# 注意：LVGL 9.1 真正认的是下面这两个名字（LV_BUILD_EXAMPLES 是无效的）
set(LV_CONF_BUILD_DISABLE_EXAMPLES ON CACHE BOOL "" FORCE)
set(LV_CONF_BUILD_DISABLE_DEMOS    ON CACHE BOOL "" FORCE)

add_subdirectory("${LVGL_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/lvgl")

# ---------------- 应用 ----------------
# 字体是 tools/gen_v32_fonts.py 从参考稿 HTML 扒字符集后生成的位图子集
# （DejaVuSans + seguisym + simhei 三合一，覆盖 246 个字符，无缺字）。
# 改动界面文案后要重跑那个脚本，否则新字会显示成空白。
add_executable(waveform_demo
    src/main.c
    src/ui.c
    src/bpsk_decoder.c
    src/plot.c
    src/port.c
    src/drm_disp.c
    src/demo_signal.c
    src/wave_source.c
    src/wave_source_v32.c
    src/v32cap3.c
    src/fft.c
    src/bl_interp.c
    src/fonts/lv_font_v32_12.c
    src/fonts/lv_font_v32_16.c
    src/fonts/lv_font_v32_18.c
    src/fonts/lv_font_v32_20.c
    src/fonts/lv_font_v32_12_bold.c
    src/fonts/lv_font_v32_16_bold.c
    src/fonts/lv_font_v32_18_bold.c
    src/fonts/lv_font_v32_20_bold.c
    # 16px 粗体现在用上了：scope_proto 化之后顶部信息行（A1=… Time/Div=…）
    # 和右栏读数都走 lv_font_v32_16_bold（V32_FONT_INFO / V32_FONT_ROWVAL）。
    # （CMake 注释只有 #，写 /* */ 会被当成文件名，报 Cannot find source file: "//*"）
)

target_include_directories(waveform_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(waveform_demo PRIVATE lvgl liquid m pthread)
add_executable(bpsk_capture_decode tools/decode_bpsk_capture.c src/bpsk_decoder.c)
target_include_directories(bpsk_capture_decode PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(bpsk_capture_decode PRIVATE liquid m)
add_executable(test_bpsk_decoder tools/test_bpsk_decoder.c src/bpsk_decoder.c)
add_executable(test_raw_capture tools/test_raw_capture.c)
target_include_directories(test_raw_capture PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(test_raw_capture PRIVATE pthread)
target_include_directories(test_bpsk_decoder PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(test_bpsk_decoder PRIVATE liquid m)
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(test_bpsk_decoder PRIVATE -UNDEBUG)
    target_compile_options(test_raw_capture PRIVATE -UNDEBUG)
endif()
add_executable(ook_capture_decode tools/decode_ook_capture.c)
target_include_directories(ook_capture_decode PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(ook_capture_decode PRIVATE m)
add_executable(fsk_capture_decode tools/decode_ook_capture.c)
target_compile_definitions(fsk_capture_decode PRIVATE FSK_CAPTURE=1)
target_include_directories(fsk_capture_decode PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(fsk_capture_decode PRIVATE m)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(waveform_demo PRIVATE -O2 -Wall -Wextra)
endif()

# Release 下 strip 符号，体积从 ~3.3MB 降到 ~1MB
if(CMAKE_BUILD_TYPE STREQUAL "Release" AND CMAKE_STRIP)
    add_custom_command(TARGET waveform_demo POST_BUILD
        COMMAND "${CMAKE_STRIP}" "$<TARGET_FILE:waveform_demo>"
        COMMENT "Stripping waveform_demo")
endif()

message(STATUS "toolchain   : ${CMAKE_C_COMPILER}")
message(STATUS "sysroot     : ${CMAKE_SYSROOT}")
