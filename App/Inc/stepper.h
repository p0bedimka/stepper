//
// Created by Pobed on 27.07.2026.
//

#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>

#include "hal_stepper.h"
#include "planner.h"

typedef struct Stepper_Type Stepper_Type;

typedef struct {
    TIM_Type htim;
    GPIO_Type pul, dir, en;
    Planner_Init pl;
    uint32_t (*GetTick)(void);
} Stepper_Init;

typedef struct {
    Stepper_Type* (*Init)(Stepper_Init *init);
    void (*DeInit)(Stepper_Type *st);
    void (*Enable)(Stepper_Type *st);
    void (*Disable)(Stepper_Type *st);
    uint32_t (*GetAcc)(Stepper_Type *st);
    void (*SetAcc)(Stepper_Type *st, uint32_t acc);
    int32_t (*GetVel)(Stepper_Type *st);
    void (*SetVel)(Stepper_Type *st, int32_t vel);
    void (*Stop)(Stepper_Type *st);
    void (*TaskPlannerFunc)(Stepper_Type *st);
} Stepper_Interface;

extern const Stepper_Interface Stepper_Itf;

#endif
