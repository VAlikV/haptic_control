cmake_minimum_required(VERSION 3.5.0)

set(PROJECT HapticControl)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(GCC_COVERAGE_COMPILE_FLAGS "-W -fexceptions -O2 -DNDEBUG -Dlinux")

# find_library(Lapack_LIB liblapack NAMES liblapack lapack HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(drake CONFIG REQUIRED)

find_library(HDU_LIB HDU NAMES libHDU.a -libHDU HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(HD_LIB HD NAMES libHD.so.3.4.0 -libHD HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(HLU_LIB HDU NAMES libHLU.a -libHLU HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(HL_LIB HD NAMES libHL.so.3.4.0 -libHL HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)

find_package(nlohmann_json 3.11.3 REQUIRED)

add_executable(${PROJECT}
    main.cpp
    tele_params/tele_params.cpp         tele_params/tele_params.hpp
    ik/drake_kinematic.cpp              ik/drake_kinematic.hpp
    udp/udp_server.cpp                  udp/udp_server.hpp
    lockfree/lockfree.hpp
)

target_link_libraries(${PROJECT}
    ${HDU_LIB}
    ${HD_LIB}
    ${HLU_LIB}
    ${HL_LIB}
    rt
    stdc++
    m
    ncurses
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    drake::drake
)

target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)