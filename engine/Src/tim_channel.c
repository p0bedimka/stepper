//
// Created by Pobed on 05.07.2026.
//
#include "tim_channel.h"

void TIM_SetSettingsCannel(TIM_TypeDef *htim, uint32_t channel) {
    const uint8_t PWM_MOD = 0b110U;
    switch (channel) {
        case TIM_CHANNEL_1:
            htim->CCMR1 = (htim->CCMR1 & ~TIM_CCMR1_OC1M) | (PWM_MOD << TIM_CCMR1_OC1M_Pos);
            htim->CCMR1 |= TIM_CCMR1_OC1PE;
            break;
        case TIM_CHANNEL_2:
            htim->CCMR1 = (htim->CCMR1 & ~TIM_CCMR1_OC2M) | (PWM_MOD << TIM_CCMR1_OC2M_Pos);
            htim->CCMR1 |= TIM_CCMR1_OC2PE;
            break;
        case TIM_CHANNEL_3:
            htim->CCMR2 = (htim->CCMR2 & ~TIM_CCMR2_OC3M) | (PWM_MOD << TIM_CCMR2_OC3M_Pos);
            htim->CCMR2 |= TIM_CCMR2_OC3PE;
            break;
        case TIM_CHANNEL_4:
            htim->CCMR2 = (htim->CCMR2 & ~TIM_CCMR2_OC4M) | (PWM_MOD << TIM_CCMR2_OC4M_Pos);
            htim->CCMR2 |= TIM_CCMR2_OC4PE;
            break;
        default:
            break;
    };
}

Stepper_Status TIM_CheckCannel(TIM_TypeDef *htim, uint32_t channel) {
    Stepper_Status status;
    switch (channel) {
        case TIM_CHANNEL_1:
            status = STEPPER_OK;
            break;
        case TIM_CHANNEL_2:
            status = STEPPER_OK;
            break;
        case TIM_CHANNEL_3:
            status = STEPPER_OK;
            break;
        case TIM_CHANNEL_4:
            status = STEPPER_OK;
            break;
        default:
            status = STEPPER_FAILED;
            break;
    }
    return status;
}

void TIM_EnableChannel(TIM_TypeDef *htim, uint32_t channel) {
    switch (channel) {
        case TIM_CHANNEL_1:
            htim->CCER |= TIM_CCER_CC1E;
            break;
        case TIM_CHANNEL_2:
            htim->CCER |= TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            htim->CCER |= TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            htim->CCER |= TIM_CCER_CC4E;
            break;
        default:
            break;
    }
}

void TIM_DisableChannel(TIM_TypeDef *htim, uint32_t channel) {
    switch (channel) {
        case TIM_CHANNEL_1:
            htim->CCER &= ~TIM_CCER_CC1E;
            break;
        case TIM_CHANNEL_2:
            htim->CCER &= ~TIM_CCER_CC2E;
            break;
        case TIM_CHANNEL_3:
            htim->CCER &= ~TIM_CCER_CC3E;
            break;
        case TIM_CHANNEL_4:
            htim->CCER &= ~TIM_CCER_CC4E;
            break;
        default:
            break;
    };
}

void TIM_ApplyChannel(TIM_TypeDef *htim, uint32_t channel, uint32_t pulse) {
    switch (channel) {
        case TIM_CHANNEL_1:
            htim->CCR1 = pulse;
            break;
        case TIM_CHANNEL_2:
            htim->CCR2 = pulse;
            break;
        case TIM_CHANNEL_3:
            htim->CCR3 = pulse;
            break;
        case TIM_CHANNEL_4:
            htim->CCR4 = pulse;
            break;
        default:
            break;
    }
}

int64_t TIM_GetPulseChannel(TIM_TypeDef *htim, uint32_t channel) {
    int64_t pulse;
    switch (channel) {
        case TIM_CHANNEL_1:
            pulse = htim->CCR1;
            break;
        case TIM_CHANNEL_2:
            pulse = htim->CCR2;
            break;
        case TIM_CHANNEL_3:
            pulse = htim->CCR3;
            break;
        case TIM_CHANNEL_4:
            pulse = htim->CCR4;
            break;
        default:
            pulse = -1;
            break;
    }
    return pulse;
}
