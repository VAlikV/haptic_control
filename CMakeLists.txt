cmake_minimum_required(VERSION 3.5.0)

set(PROJECT HapticControl)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(GCC_COVERAGE_COMPILE_FLAGS "-W -fexceptions -O2 -DNDEBUG -Dlinux")

find_library(HDU_LIB HDU NAMES libHDU.a -libHDU HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(HD_LIB HD NAMES libHD.so.3.4.0 -libHD HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)

# set(USABLE_FUNCTIONS ${CMAKE_CURRENT_SOURCE_DIR}/usable_functions)

add_executable(${PROJECT}
    haptic_handling.cpp
)

target_link_libraries(${PROJECT}
    ${HDU_LIB}
    ${HD_LIB}
    rt
    stdc++
    m
    ncurses
)

target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
