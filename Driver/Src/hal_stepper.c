//
// Created by Pobed on 27.07.2026.
//
#include "hal_stepper.h"

const HAL_Stepper_Interface HAL_Stepper_Itf = {
    .TIM_Init = TIM_Initialization,
    .TIM_Enable = TIM_Enable,
    .TIM_Disable = TIM_Disable,
    .TIM_EnableCompare = TIM_EnableCompare,
    .TIM_DisableCompare = TIM_DisableCompare,
    .TIM_СanSetFrequency = TIM_СanSetFrequency,
    .TIM_SetFrequency = TIM_SetFrequency,
    .GPIO_Init = GPIO_Initialization,
    .GPIO_Set = GPIO_Set,
    .GPIO_Reset = GPIO_Reset,
};
