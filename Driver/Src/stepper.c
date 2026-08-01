//
// Created by Pobed on 27.07.2026.
//
#include "stepper.h"

const Stepper_Interface Stepper_Itf = {
    .Init = Stepper_Init,
    .Enable = Stepper_Enable,
    .Disable = Stepper_Disable,
    .СanSetVel = Stepper_СanSetVel,
    .SetVel = Stepper_SetVel,
    .TIMx_IRQHandler = Stepper_TIMx_IRQHandler,
};
