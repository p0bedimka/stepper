cmake_minimum_required(VERSION 3.30)

project(stepper)

file(GLOB_RECURSE STEPPER_MOTORS_SOURCES "App/Core/Src/*.*" "App/MotionMath/Src/*.*" "BSP/Src/*.*")

add_library(${PROJECT_NAME} STATIC ${STEPPER_MOTORS_SOURCES})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        App/Core/Inc
        App/MotionMath/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
)
