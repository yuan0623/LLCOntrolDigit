cmake_minimum_required(VERSION 2.23)
project(LLControlDigit)

set(CMAKE_CXX_STANDARD 14)

include_directories(~/Dropbox/UML_dropbox/research/digit_agility/lowlevelapi_1206)
include_directories(include)
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
add_library(digitLowLevel
        ~/Dropbox/UML_dropbox/research/digit_agility/lowlevelapi_1206/lowlevelapi.c)

# Threads
find_package(Threads REQUIRED)

# json
find_package(nlohmann_json 3.9.1 REQUIRED)
# boost
include_directories(~/boost_1_76_0)


add_library(libInEKF
        include/InEKF.h
        include/LieGroup.h
        include/NoiseParams.h
        include/RobotState.h
        include/digit_json_helper.h
        src/InEKF.cpp
        src/LieGroup.cpp
        src/NoiseParams.cpp
        src/RobotState.cpp
        src/digit_json_helper.cpp)
add_library(libDigitKinematics
        include/digit_forward_kinematics.h
        src/digit_forward_kinematics.cpp)


add_executable(LLSSDigit src/main.cpp)
add_executable(LLControl src/main_llcontrol.cpp)


target_link_libraries(LLSSDigit
        digitLowLevel
        ~/Dropbox/UML_dropbox/research/digit_agility/lowlevelapi_1206/libartl/libartl.so
        libInEKF
        libDigitKinematics
        Threads::Threads
        ${Boost_LIBRARIES}
        )

target_link_libraries(
        LLControl
        digitLowLevel
        ~/Dropbox/UML_dropbox/research/digit_agility/lowlevelapi_1206/libartl/libartl.so
        libDigitKinematics
        Threads::Threads
        ${Boost_LIBRARIES}
)