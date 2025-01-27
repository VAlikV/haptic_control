cmake_minimum_required(VERSION 3.5.0)

set(PROJECT My_Ctrl)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_library(KDL liborocos-kdl.so.1.5.1 NAMES liborocos-kdl liborocos kdl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(drake CONFIG REQUIRED)


# message(${FORTRUN})

set(SNS ${CMAKE_CURRENT_SOURCE_DIR}/lib/sns)
set(SNS_LIB
    ${SNS}/fosns_velocity_ik.cpp            ${SNS}/fosns_velocity_ik.hpp
    ${SNS}/fsns_velocity_ik.cpp             ${SNS}/fsns_velocity_ik.hpp
    ${SNS}/osns_sm_velocity_ik.cpp          ${SNS}/osns_sm_velocity_ik.hpp
    ${SNS}/osns_velocity_ik.cpp             ${SNS}/osns_velocity_ik.hpp
    ${SNS}/sns_acc_ik_base.cpp              ${SNS}/sns_acc_ik_base.hpp
    ${SNS}/sns_ik_base.cpp                  ${SNS}/sns_ik_base.hpp
    ${SNS}/sns_ik_math_utils.cpp            ${SNS}/sns_ik_math_utils.hpp
    ${SNS}/sns_ik.cpp                       ${SNS}/sns_ik.hpp
    ${SNS}/sns_linear_solver.cpp            ${SNS}/sns_linear_solver.hpp
    ${SNS}/sns_position_ik.cpp              ${SNS}/sns_position_ik.hpp
    ${SNS}/sns_vel_ik_base_interface.cpp    ${SNS}/sns_vel_ik_base_interface.hpp
    ${SNS}/sns_vel_ik_base.cpp              ${SNS}/sns_vel_ik_base.hpp
    ${SNS}/sns_velocity_ik.cpp              ${SNS}/sns_velocity_ik.hpp
)

set(MAIN_KINEMATIC ${CMAKE_CURRENT_SOURCE_DIR}/facade)
set(KINEMATIC_CLASS ${CMAKE_CURRENT_SOURCE_DIR}/kinematic_class)
set(KDL_KINEMATIC_CLASS ${KINEMATIC_CLASS}/KDL)
set(SNS_KINEMATIC_CLASS ${KINEMATIC_CLASS}/SNS)
set(PIN_KINEMATIC_CLASS ${KINEMATIC_CLASS}/Pinocchio)
set(DRAKE_KINEMATIC_CLASS ${KINEMATIC_CLASS}/Drake)

set(USABLE_FUNCTIONS ${CMAKE_CURRENT_SOURCE_DIR}/usable_functions)

set(FILES
    ${SNS_LIB}

    ${MAIN_KINEMATIC}/main_kinematic.cpp                ${MAIN_KINEMATIC}/main_kinematic.hpp

    ${KINEMATIC_CLASS}/base_kinematic.hpp               ${KINEMATIC_CLASS}/base_kinematic.cpp
    ${KDL_KINEMATIC_CLASS}/KDL_kinematic.hpp            ${KDL_KINEMATIC_CLASS}/KDL_kinematic.cpp
    ${SNS_KINEMATIC_CLASS}/SNS_kinematic.hpp            ${SNS_KINEMATIC_CLASS}/SNS_kinematic.cpp
    ${PIN_KINEMATIC_CLASS}/Pinocchio_kinematic.hpp      ${PIN_KINEMATIC_CLASS}/Pinocchio_kinematic.cpp
    ${DRAKE_KINEMATIC_CLASS}/Drake_kinematic.hpp        ${DRAKE_KINEMATIC_CLASS}/Drake_kinematic.cpp

    # ${USABLE_FUNCTIONS}/usable_functions.hpp    ${USABLE_FUNCTIONS}/usable_functions.cpp
)

add_library(${PROJECT} STATIC ${FILES})

target_link_libraries(${PROJECT} 
    Eigen3::Eigen
    ${KDL}
    Threads::Threads
    pinocchio::pinocchio
    drake::drake
)