//
// Created by Pobed on 27.07.2026.
//
#include "hal_stm32f4.h"

void TIM_SetPulse(TIM_Type *htim, uint32_t pulse);

void TIM_Initialization(TIM_Type *tim, TIM_Init *init) {
    tim->instance = init->instance;
    tim->channel = init->channel;
    tim->arr_max = init->arr_max;
    tim->clock = init->clock;
}

void TIM_Enable(TIM_Type *tim) {
    tim->instance->CR1 |= TIM_CR1_CEN;
}

void TIM_Disable(TIM_Type *tim) {
     tim->instance->CR1 &= ~TIM_CR1_CEN;
}

void TIM_EnableCompare(TIM_Type *tim) {
    switch (tim->channel) {
        case TIM_CHANNEL_1:
            tim->instance->CCER |= TIM_CCER_CC1E;
            break;
        case TIM_CHANNEL_2:
            tim->instance->CCER |= TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            tim->instance->CCER |= TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            tim->instance->CCER |= TIM_CCER_CC4E;
            break;
        default:
            break;
    }
}

void TIM_DisableCompare(TIM_Type *tim) {
    switch (tim->channel) {
        case TIM_CHANNEL_1:
            tim->instance->CCER &= ~TIM_CCER_CC1E;
            break;
        case TIM_CHANNEL_2:
            tim->instance->CCER &= ~TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            tim->instance->CCER &= ~TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            tim->instance->CCER &= ~TIM_CCER_CC4E;
            break;
        default:
            break;
    };
}

void TIM_SetPulse(TIM_Type *htim, uint32_t pulse) {
    switch (htim->channel) {
        case TIM_CHANNEL_1:
            htim->instance->CCR1 = pulse;
            break;
        case TIM_CHANNEL_2:
            htim->instance->CCR2 = pulse;
            break;
        case TIM_CHANNEL_3:
            htim->instance->CCR3 = pulse;
            break;
        case TIM_CHANNEL_4:
            htim->instance->CCR4 = pulse;
            break;
        default:
            break;
    }
}

bool TIM_СanSetFrequency(TIM_Type *tim, uint32_t freq) {
    bool status = true;
    uint32_t period;

    if (freq != 0) {
        period = tim->clock / (freq * (tim->instance->PSC + 1)) - 1;
        if (period > tim->arr_max)
            status = false;
    }
    return status;
}

bool TIM_SetFrequency(TIM_Type *tim, uint32_t freq) {
    bool status = false;
    uint32_t period, pulse;

    if (freq != 0) {
        period = tim->clock / (freq * (tim->instance->PSC + 1)) - 1;
        if (period <= tim->arr_max) {
            pulse = (period + 1) / 2;

            tim->instance->ARR = period;
            TIM_SetPulse(tim, pulse);
            status = true;
        }
    }
    else {
        pulse = 0;
        TIM_SetPulse(tim, pulse);
        status = true;
    }
    return status;
}

void GPIO_Initialization(GPIO_Type *gpio, GPIO_Init *init) {
    gpio->port = init->port;
    gpio->pin = init->pin;
}

void GPIO_Set(GPIO_Type *gpio) {
    gpio->port->BSRR = gpio->pin;
}

void GPIO_Reset(GPIO_Type *gpio) {
    gpio->port->BSRR = gpio->pin << 16;
}