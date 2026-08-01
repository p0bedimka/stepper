//
// Created by Pobed on 27.07.2026.
//

#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdint.h>

#include "stepper.h"
#include "planner.h"

typedef struct {
    Stepper_t stp;
    Planner_t pl;

    volatile int32_t tar_vel;
    volatile int32_t cur_vel;
    volatile uint32_t time;
} StepMotor_t;

typedef struct {
    void (*Init)(StepMotor_t *motor);
    void (*Enable)(StepMotor_t *motor);
    void (*Disable)(StepMotor_t *motor);
    uint32_t (*GetAcc)(StepMotor_t *motor);
    void (*SetAcc)(StepMotor_t *motor, uint32_t acc);
    int32_t (*GetVel)(StepMotor_t *motor);
    void (*SetVel)(StepMotor_t *motor, int32_t vel);
    void (*Stop)(StepMotor_t *motor);
    void (*TaskPlanner)(StepMotor_t *motor);
    void (*TIMx_IRQHandler)(StepMotor_t *motor);
} StepMotor_Interface;

extern const StepMotor_Interface StepMotor_Itf;

#endif
