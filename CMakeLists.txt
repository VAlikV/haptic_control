cmake_minimum_required(VERSION 3.5.0)

set(PROJECT HapticControl)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(GCC_COVERAGE_COMPILE_FLAGS "-W -fexceptions -O2 -DNDEBUG -Dlinux")

# find_library(Lapack_LIB liblapack NAMES liblapack lapack HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_library(HDU_LIB HDU NAMES libHDU.a -libHDU HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(HD_LIB HD NAMES libHD.so.3.4.0 -libHD HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(GLUT_LIB GLUT NAMES libglut.so.3.9.0 -libglut HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(GL_LIB GL NAMES libGL.so.1.7.0 -libGL HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_library(GLU_LIB GLU NAMES libGLU.so.1.3.1 -libGL HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)

# set(USABLE_FUNCTIONS ${CMAKE_CURRENT_SOURCE_DIR}/usable_functions)

add_executable(${PROJECT}
    main.cpp
    # main_haptic/haptic_handling.cpp     main_haptic/haptic_handling.hpp
    main_haptic/haptic_graphics.cpp     main_haptic/haptic_graphics.hpp
    helper/helper.cpp                   helper/helper.hpp
)

add_subdirectory(lib/my_lib_for_ik)

target_link_libraries(${PROJECT}
    # ${Lapack_LIB}
    # Eigen3::Eigen
    ${HDU_LIB}
    ${HD_LIB}
    ${GLUT_LIB}
    ${GL_LIB}
    ${GLU_LIB}
    rt
    stdc++
    m
    ncurses
    My_Ctrl
)

target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)