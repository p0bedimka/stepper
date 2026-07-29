cmake_minimum_required(VERSION 3.30)

project(stepper)

file(GLOB_RECURSE STEPPER_MOTORS_SOURCES "App/Src/*.*" "MotionMath/Src/*.*" "Driver/Src/*.*")

add_library(${PROJECT_NAME} STATIC ${STEPPER_MOTORS_SOURCES})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/MotionMath/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Driver/Inc
)
