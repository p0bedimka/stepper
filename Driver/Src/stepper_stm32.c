//
// Created by Pobed on 27.07.2026.
//
#include "stepper_stm32.h"

#include <stdlib.h>


#define TIM_SET_COMPARE(__HANDLE__, __COMPARE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCR1 = (__COMPARE__)) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCR2 = (__COMPARE__)) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCR3 = (__COMPARE__)) : \
     ((__HANDLE__)->Instance->CCR4 = (__COMPARE__)))

#define TIM_ENABLE_OCxPRELOAD(__HANDLE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCMR1 |= TIM_CCMR1_OC1PE) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCMR1 |= TIM_CCMR1_OC2PE) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCMR2 |= TIM_CCMR2_OC3PE) : \
     ((__HANDLE__)->Instance->CCMR2 |= TIM_CCMR2_OC4PE))

#define TIM_SET_MODE_PWM(__HANDLE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCMR1 = ((__HANDLE__)->Instance->CCMR1 & ~TIM_CCMR1_OC1M) | (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCMR1 = ((__HANDLE__)->Instance->CCMR1 & ~TIM_CCMR1_OC2M) | (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2)) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCMR2 = ((__HANDLE__)->Instance->CCMR2 & ~TIM_CCMR2_OC3M) | (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2)) : \
     ((__HANDLE__)->Instance->CCMR2 = ((__HANDLE__)->Instance->CCMR2 & ~TIM_CCMR2_OC4M) | (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2)))

#define TIM_ENABLE_COMPARE(__HANDLE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCER |= TIM_CCER_CC1E) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCER |= TIM_CCER_CC2E) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCER |= TIM_CCER_CC3E) : \
     ((__HANDLE__)->Instance->CCER |= TIM_CCER_CC4E))

#define TIM_DISABLE_COMPARE(__HANDLE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCER &= ~TIM_CCER_CC1E) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCER &= ~TIM_CCER_CC2E) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCER &= ~TIM_CCER_CC3E) : \
     ((__HANDLE__)->Instance->CCER &= ~TIM_CCER_CC4E))

static uint32_t Stepper_CalcPeriod(const Stepper *stp, uint32_t freq);

void Stepper_Init(Stepper *stp) {
    if (stp == NULL)
        return;

    if (stp->htim.Instance == NULL)
        return;

    if (!IS_NVIC_DEVICE_IRQ(stp->htim.IRQn) || stp->htim.Clock < 2 * MIN_CLK_APB) {
        return;
    }

    if (stp->dir.Port == NULL && stp->en.Port == NULL)
        return;

    if (stp->GetTick == NULL)
        return;

    if (stp->htim.Bit != TIM_16_BIT || stp->htim.Bit != TIM_32_BIT)
        stp->htim.Bit = TIM_16_BIT;

    stp->htim.Instance->CR1 &= ~TIM_CR1_CEN;

    stp->htim.Instance->PSC = 0;
    stp->htim.Instance->ARR = Stepper_CalcPeriod(stp, 1000);

    stp->htim.Instance->CR1 |= TIM_CR1_ARPE;

    TIM_SET_MODE_PWM(&stp->htim);
    TIM_ENABLE_OCxPRELOAD(&stp->htim);

    TIM_SET_COMPARE(&stp->htim, 0);

    stp->htim.Instance->DIER |= TIM_DIER_UIE;
    stp->htim.Instance->SR &= ~TIM_SR_UIF;
}

void Stepper_Enable(Stepper *stp) {
    if (stp == NULL || stp->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp->htim.IRQn))
        return;

    stp->htim.Instance->CR1 |= TIM_CR1_CEN;
    TIM_ENABLE_COMPARE(&stp->htim);
    stp->en.Port->BSRR = stp->en.Pin;
    NVIC_EnableIRQ(stp->htim.IRQn);
}

void Stepper_Disable(Stepper *stp) {
    if (stp == NULL || stp->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp->htim.IRQn))
        return;

    stp->htim.Instance->CR1 &= ~TIM_CR1_CEN;
    TIM_DISABLE_COMPARE(&stp->htim);
    stp->en.Port->BSRR = stp->en.Pin << 16;
    NVIC_DisableIRQ(stp->htim.IRQn);
}

static uint32_t Stepper_CalcPeriod(const Stepper *stp, uint32_t freq) {
    if (freq == 0) return 0;
    return stp->htim.Clock / (freq * (stp->htim.Instance->PSC + 1)) - 1;
}

bool Stepper_СanSetVel(Stepper *stp, int32_t vel) {
    if (stp == NULL || stp->htim.Instance == NULL)
        return false;

    if (vel == 0)
        return true;

    uint32_t period = Stepper_CalcPeriod(stp, abs(vel));
    return period <= ((1ULL << stp->htim.Bit) - 1);
}

bool Stepper_SetVel(Stepper *stp, int32_t vel) {
        if (stp == NULL)
        return false;

    if (stp->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp->htim.IRQn))
        return false;

    if (stp->dir.Port == NULL)
        return false;

    uint32_t period = 0, pulse = 0;

    if (vel != 0) {
        period = Stepper_CalcPeriod(stp, abs(vel));
        if (period <= ((1ULL << stp->htim.Bit) - 1))
            pulse = (period + 1) / 2;
        else
            return false;
    }

    NVIC_DisableIRQ(stp->htim.IRQn);
    if (vel != 0)
        stp->htim.Instance->ARR = period;
    TIM_SET_COMPARE(&stp->htim, pulse);
    if (vel > 0)
        stp->dir.State = stp->dir.Pin;
    else
        stp->dir.State = stp->dir.Pin << 16;
    NVIC_EnableIRQ(stp->htim.IRQn);

    return true;
}

void Stepper_TIMx_IRQHandler(Stepper *stp) {
    if (stp == NULL || stp->htim.Instance == NULL)
        return;

    if (stp->htim.Instance->SR & TIM_SR_UIF) {
        stp->htim.Instance->SR &= ~TIM_SR_UIF;

        if (stp->dir.Port != NULL)
            stp->dir.Port->BSRR = stp->dir.State;
    }
}