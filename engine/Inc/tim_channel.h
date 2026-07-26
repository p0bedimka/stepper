//
// Created by Pobed on 05.07.2026.
//

#ifndef PWM_CHAN_H
#define PWM_CHAN_H

#include "stepper.h"

Stepper_Status TIM_CheckCannel(TIM_TypeDef *htim, uint32_t channel);
void TIM_SetSettingsCannel(TIM_TypeDef *htim, uint32_t channel);
void TIM_EnableChannel(TIM_TypeDef *htim, uint32_t channel);
void TIM_DisableChannel(TIM_TypeDef *htim, uint32_t channel);
void TIM_ApplyChannel(TIM_TypeDef *htim, uint32_t channel, uint32_t pulse);
int64_t TIM_GetPulseChannel(TIM_TypeDef *htim, uint32_t channel);

#endif
