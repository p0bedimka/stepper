//
// Created by Pobed on 27.07.2026.
//

#ifndef DRIVER_H
#define DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx.h"

#include "stepper_def_stm32.h"

typedef struct {
    struct {
        TIM_TypeDef *Instance;
        uint8_t Channel;
        uint8_t Bit;
        uint32_t Clock;
        IRQn_Type IRQn;
    } htim;

    struct {
        GPIO_TypeDef *Port;
        uint16_t Pin;
        uint32_t State;
    } dir;

    struct {
        GPIO_TypeDef *Port;
        uint16_t Pin;
    } en;

    uint32_t (*GetTick)(void);
} Stepper;

void Stepper_Init(Stepper *stp);
void Stepper_Enable(Stepper *stp);
void Stepper_Disable(Stepper *stp);
bool Stepper_СanSetVel(Stepper *stp, int32_t freq);
bool Stepper_SetVel(Stepper *stp, int32_t vel);
void Stepper_TIMx_IRQHandler(Stepper *stp);

#endif
