//
// Created by Pobed on 27.07.2026.
//

#ifndef DRIVER_H
#define DRIVER_H

#include <stdbool.h>

#include "stm32f4xx.h"

typedef struct TIM_Type TIM_Type;
typedef struct TIM_Type TIM_Init;

struct TIM_Type {
    TIM_TypeDef *instance;
    uint16_t channel;
    uint32_t arr_max;
    uint32_t clock;
};

typedef struct GPIO_Type GPIO_Type;
typedef struct GPIO_Type GPIO_Init;

struct GPIO_Type {
    GPIO_TypeDef *port;
    uint16_t pin;
};

void TIM_Initialization(TIM_Type *tim, TIM_Init *init);
void TIM_Enable(TIM_Type *tim);
void TIM_Disable(TIM_Type *tim);
void TIM_EnableCompare(TIM_Type *tim);
void TIM_DisableCompare(TIM_Type *tim);
bool TIM_СanSetFrequency(TIM_Type *tim, uint32_t freq);
bool TIM_SetFrequency(TIM_Type *tim, uint32_t freq);
void GPIO_Initialization(GPIO_Type *gpio, GPIO_Init *init);
void GPIO_Set(GPIO_Type *gpio);
void GPIO_Reset(GPIO_Type *gpio);

#endif
