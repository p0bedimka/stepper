cmake_minimum_required(VERSION 3.30)

project(stepper-driver)

file(GLOB_RECURSE STEPPER_MOTORS_SOURCES "engine/Src/*.*" "planner/Src/*.*")

add_library(${PROJECT_NAME} STATIC ${STEPPER_MOTORS_SOURCES})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/engine/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/Inc
)