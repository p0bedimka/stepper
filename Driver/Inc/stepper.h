//
// Created by Pobed on 27.07.2026.
//

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>

#include "stepper_stm32.h"

typedef Stepper Stepper_t;

typedef struct {
    void (*Init)(Stepper_t *stp);
    void (*Enable)(Stepper_t *stp);
    void (*Disable)(Stepper_t *stp);
    bool (*СanSetVel)(Stepper_t *stp, int32_t vel);
    bool (*SetVel)(Stepper_t *stp, int32_t vel);
    void (*TIMx_IRQHandler)(Stepper_t *stp);
} Stepper_Interface;

extern const Stepper_Interface Stepper_Itf;

#endif
