//
// Created by Pobed on 27.07.2026.
//
#include "step_motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void StepMotor_Init(StepMotor_t *motor) {
    if (!motor)
        return;

    Stepper_Itf.Init(&motor->stp);
    Planner_Itf.Init(&motor->pl);

    motor->tar_vel = 0;
    motor->cur_vel = 0;
    motor->time = motor->stp.GetTick();
}

void StepMotor_Enable(StepMotor_t *motor) {
    if (motor != NULL)
        Stepper_Itf.Enable(&motor->stp);
}

void StepMotor_Disable(StepMotor_t *motor) {
    if (motor == NULL)
        return;

    if (motor->cur_vel == 0)
        Stepper_Itf.Disable(&motor->stp);
}

uint32_t StepMotor_GetAcc(StepMotor_t *motor) {
    if (motor == NULL)
        return 0;

    return Planner_Itf.GetAcc(&motor->pl);
}

void StepMotor_SetAcc(StepMotor_t *motor, uint32_t acc) {
    if (motor != NULL)
        Planner_Itf.SetAcc(&motor->pl, acc);
}

int32_t StepMotor_GetVel(StepMotor_t *motor) {
    if (motor == NULL)
        return 0;

    return motor->cur_vel;
}

void StepMotor_SetVel(StepMotor_t *motor, int32_t vel) {
    if (motor == NULL)
        return;

    if (Stepper_Itf.СanSetVel(&motor->stp, vel))
        motor->tar_vel = vel;
}

void StepMotor_Stop(StepMotor_t *motor) {
    if (motor != NULL) {
        motor->tar_vel = 0;
        motor->cur_vel = 0;
        Stepper_Itf.SetVel(&motor->stp, 0);
    }
}

void StepMotor_TaskPlanner(StepMotor_t *motor) {
    if (motor == NULL)
        return;

    uint32_t timeout = Planner_Itf.GetTimeAcc(&motor->pl);

    uint32_t elapsed, time = motor->stp.GetTick();
    if (time >= motor->time)
        elapsed = time - motor->time;
    else
        elapsed = (UINT32_MAX - motor->time) + time + 1;

    if (timeout >= elapsed) {
        int32_t vel = Planner_Itf.GetVel(&motor->pl, (float)elapsed);
        if (vel != motor->cur_vel) {
            Stepper_Itf.SetVel(&motor->stp, vel);
            motor->cur_vel = vel;
        }
    }
    else if (motor->tar_vel != motor->cur_vel) {
        if ((motor->tar_vel ^ motor->cur_vel) < 0 && motor->cur_vel != 0 && motor->tar_vel != 0)
            Planner_Itf.SetVel(&motor->pl, motor->cur_vel, 0);
        else
            Planner_Itf.SetVel(&motor->pl, motor->cur_vel, motor->tar_vel);
        motor->time = motor->stp.GetTick();
    }
}

void StepMotor_TIMx_IRQHandler(StepMotor_t *motor) {
    if (motor != NULL)
        Stepper_Itf.TIMx_IRQHandler(&motor->stp);
}

const StepMotor_Interface StepMotor_Itf = {
    .Init = StepMotor_Init,
    .Enable = StepMotor_Enable,
    .Disable = StepMotor_Disable,
    .GetAcc = StepMotor_GetAcc,
    .SetAcc = StepMotor_SetAcc,
    .GetVel = StepMotor_GetVel,
    .SetVel = StepMotor_SetVel,
    .Stop = StepMotor_Stop,
    .TaskPlanner = StepMotor_TaskPlanner,
    .TIMx_IRQHandler = StepMotor_TIMx_IRQHandler,
};