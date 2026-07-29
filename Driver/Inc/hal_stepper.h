//
// Created by Pobed on 27.07.2026.
//

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>

#include "hal_stm32f4.h"

typedef struct {
    void (*TIM_Init)(TIM_Type *htim, TIM_Init *init);
    void (*TIM_Enable)(TIM_Type *tim);
    void (*TIM_Disable)(TIM_Type *tim);
    void (*TIM_EnableCompare)(TIM_Type *tim);
    void (*TIM_DisableCompare)(TIM_Type *tim);
    bool (*TIM_СanSetFrequency)(TIM_Type *tim, uint32_t freq);
    bool (*TIM_SetFrequency)(TIM_Type *tim, uint32_t freq);
    void (*GPIO_Init)(GPIO_Type *tim, GPIO_Init *init);
    void (*GPIO_Set)(GPIO_Type *gpio);
    void (*GPIO_Reset)(GPIO_Type *gpio);
} HAL_Stepper_Interface;

extern const HAL_Stepper_Interface HAL_Stepper_Itf;

#endif
