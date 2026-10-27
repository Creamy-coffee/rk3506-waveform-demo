# Offline, pinned dependency; no configure-time network or target execution.
set(V32_LIQUID_ROOT "${CMAKE_CURRENT_BINARY_DIR}/third_party")
file(SHA256 "${CMAKE_CURRENT_LIST_DIR}/liquid-dsp-1.7.0.tar.gz" liquid_archive_hash)
if(NOT liquid_archive_hash STREQUAL "33c42ebc2e6088570421e282c6332e899705d42b4f73ebd1212e6a11da714dd4")
    message(FATAL_ERROR "Pinned liquid-dsp archive checksum mismatch")
endif()
set(V32_LIQUID_DIR "${V32_LIQUID_ROOT}/liquid-dsp-1.7.0")
file(MAKE_DIRECTORY "${V32_LIQUID_ROOT}")
if(NOT EXISTS "${V32_LIQUID_DIR}/include/liquid.h")
    execute_process(COMMAND "${CMAKE_COMMAND}" -E tar xzf
        "${CMAKE_CURRENT_LIST_DIR}/liquid-dsp-1.7.0.tar.gz"
        WORKING_DIRECTORY "${V32_LIQUID_ROOT}" RESULT_VARIABLE unpack_result)
    if(NOT unpack_result EQUAL 0)
        message(FATAL_ERROR "Cannot unpack pinned liquid-dsp")
    endif()
endif()
# Upstream's C-only library is shared and unconditionally runs SIMD probes.
# Patch only the BUILD TREE: static linking, C language, skip unused SIMD runs.
file(READ "${V32_LIQUID_DIR}/CMakeLists.txt" liquid_cmake)
string(REPLACE "LANGUAGES C CXX" "LANGUAGES C" liquid_cmake "${liquid_cmake}")
string(REPLACE "include(cmake/FindSIMD.cmake)" "# Generic C kernels: no cross-target SIMD probes" liquid_cmake "${liquid_cmake}")
string(REPLACE "add_library(\${LIBNAME} SHARED" "add_library(\${LIBNAME} STATIC" liquid_cmake "${liquid_cmake}")
file(WRITE "${V32_LIQUID_DIR}/CMakeLists.txt" "${liquid_cmake}")
# The upstream summary assumes CXX flags exist. With a fresh C-only toolchain,
# unquoted empty arguments make string(STRIP) fail on some CMake versions.
file(READ "${V32_LIQUID_DIR}/cmake/summary.cmake" liquid_summary)
string(REPLACE "string(STRIP \${__result} __result)" "string(STRIP \"\${__result}\" __result)" liquid_summary "${liquid_summary}")
string(REPLACE "string(STRIP \${__flag} __flag)" "string(STRIP \"\${__flag}\" __flag)" liquid_summary "${liquid_summary}")
file(WRITE "${V32_LIQUID_DIR}/cmake/summary.cmake" "${liquid_summary}")
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_AUTOTESTS OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_SANDBOX OFF CACHE BOOL "" FORCE)
set(BUILD_DOC OFF CACHE BOOL "" FORCE)
set(ENABLE_SIMD OFF CACHE BOOL "" FORCE)
add_subdirectory("${V32_LIQUID_DIR}" "${V32_LIQUID_ROOT}/build" EXCLUDE_FROM_ALL)
target_include_directories(liquid PUBLIC "${V32_LIQUID_DIR}/include")
