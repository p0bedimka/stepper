//
// Created by Pobed on 05.07.2026.
//
#include "stepper.h"

#include <math.h>

#include "tim_channel.h"

#include <string.h>
#include <stdlib.h>

#define MSEC_PER_SEC        1000.f
#define FREQ_IQR_ZERO_VEL   1000

Stepper_Status Stepper_Init(Stepper_Type *st, uint32_t psc);
void Stepper_DeInit(Stepper_Type *st);
void Stepper_Enable(Stepper_Type *st);
void Stepper_Disable(Stepper_Type *st);
uint16_t Stepper_GetAcc(Stepper_Type *st);
Stepper_Status Stepper_SetAcc(Stepper_Type *st, uint16_t acc);
int32_t Stepper_GetVel(Stepper_Type *st);
Stepper_Status Stepper_SetVel(Stepper_Type *st, int32_t vel);
void Stepper_Stop(Stepper_Type *st);
void Stepper_TIM_IRQHandler(Stepper_Type *st);
void Stepper_Manager_IRQHandler(Stepper_Type *st);

const Stepper_Itf Stepper_Interface = {
    .Init               = Stepper_Init,
    .DeInit             = Stepper_DeInit,
    .Enable             = Stepper_Enable,
    .Disable            = Stepper_Disable,
    .GetAcc             = Stepper_GetAcc,
    .SetAcc             = Stepper_SetAcc,
    .GetVel             = Stepper_GetVel,
    .SetVel             = Stepper_SetVel,
    .Stop               = Stepper_Stop,
    .TIM_IRQHandler     = Stepper_TIM_IRQHandler,
    .Manager_IRQHandler = Stepper_Manager_IRQHandler,
};

Stepper_Status Stepper_Init(Stepper_Type *st, uint32_t psc) {
    Sigmoid_Interface.Init(&st->sigmoid, 200);

    st->tar_vel = 0;
    st->cur_vel = 0;

    st->period = 0;
    st->pulse = 0;

    st->accum_time = 0;
    st->last_time = 0;

    st->dir.port->BSRR = st->dir.pin << 16;
    st->en.port->BSRR = st->en.pin << 16;

    uint32_t pwm_period = st->htim.clock / (FREQ_IQR_ZERO_VEL * (psc + 1)) - 1;
    if (pwm_period > st->htim.max_period)
        return STEPPER_FAILED;

    st->htim.instance->CR1 &= ~TIM_CR1_CEN;
    st->htim.instance->PSC = psc;
    st->htim.instance->ARR = pwm_period;
    st->htim.instance->CNT = 0;

    TIM_DisableChannel(st->htim.instance, st->htim.channel);
    TIM_SetSettingsCannel(st->htim.instance, st->htim.channel);
    TIM_ApplyChannel(st->htim.instance, st->htim.channel, 0);

    st->htim.instance->CR1 &= ~TIM_CR1_DIR;
    st->htim.instance->DIER |= TIM_DIER_UIE;
    st->htim.instance->CR1 |= TIM_CR1_ARPE;
    return STEPPER_OK;
}

void Stepper_DeInit(Stepper_Type *st) {
    Sigmoid_Interface.DeInit(&st->sigmoid);

    st->dir.port->BSRR = st->dir.pin << 16;
    st->en.port->BSRR = st->en.pin << 16;

    memset(&st->pul, 0, sizeof(st->pul));
    memset(&st->dir, 0, sizeof(st->dir));
    memset(&st->en, 0, sizeof(st->en));
}

void Stepper_Enable(Stepper_Type *st) {
    st->en.port->BSRR = st->en.pin;
    TIM_EnableChannel(st->htim.instance, st->htim.channel);
    st->htim.instance->CR1 |= TIM_CR1_CEN;
}

void Stepper_Disable(Stepper_Type *st) {
    if (TIM_GetPulseChannel(st->htim.instance, st->htim.channel) == 0) {
        st->htim.instance->CR1 &= ~TIM_CR1_CEN;
        TIM_DisableChannel(st->htim.instance, st->htim.channel);
        st->en.port->BSRR = st->en.pin << 16;
        st->dir.port->BSRR = st->dir.pin << 16;
    }
}

uint16_t Stepper_GetAcc(Stepper_Type *st) {
    return Sigmoid_Interface.GetAccel(&st->sigmoid);
}

Stepper_Status Stepper_SetAcc(Stepper_Type *st, uint16_t acc) {
    Stepper_Status status = STEPPER_FAILED;
    if (Sigmoid_Interface.IsSteadyVel(&st->sigmoid)) {
        Sigmoid_Interface.SetAccel(&st->sigmoid, acc);
        status = STEPPER_OK;
    }
    return status;
}

int32_t Stepper_GetVel(Stepper_Type *st) {
    return st->cur_vel;
}

Stepper_Status Stepper_SetVel(Stepper_Type *st, int32_t vel) {
    Stepper_Status status = STEPPER_FAILED;
    if (st->tar_vel == st->cur_vel) {
        uint32_t period = st->htim.clock / (abs(vel) * (st->htim.instance->PSC + 1)) - 1;
        if (st->htim.max_period > period) {
            st->tar_vel = vel;
            status = STEPPER_OK;
        }
    }
    return status;
}

void Stepper_Stop(Stepper_Type *st) {
    // TODO:
}

void Stepper_TIM_IRQHandler(Stepper_Type *st) {
    if (st->htim.instance->SR & TIM_SR_UIF) {
        st->htim.instance->SR &= ~TIM_SR_UIF;

        if (st->cur_vel > 0)
            st->dir.port->BSRR = st->dir.pin;
        else
            st->dir.port->BSRR = st->dir.pin << 16;
    }
}

void Stepper_Manager_IRQHandler(Stepper_Type *st) {
    if (!Sigmoid_Interface.IsSteadyVel(&st->sigmoid)) {
        Sigmoid_Res res = Sigmoid_Interface.GetNextRes(&st->sigmoid, (float)st->accum_time);
        uint32_t period;
        uint32_t time;

        if (res.v != 0) {
            period = st->htim.clock / (abs(res.v) * (st->htim.instance->PSC + 1));
            st->period = period - 1;
            st->pulse = period / 2;
        }
        else {
            period = st->htim.clock / (FREQ_IQR_ZERO_VEL * (st->htim.instance->PSC + 1)) - 1;
            st->period = period;
            st->pulse = 0;
        }

        st->htim.instance->ARR = st->period;
        TIM_ApplyChannel(st->htim.instance, st->htim.channel, st->pulse);
        time = HAL_GetTick();

        st->cur_vel = res.v;
        st->accum_time += time - st->last_time;
        st->last_time = time;
    }
    else {
        st->accum_time = 0;

        if (st->tar_vel != st->cur_vel) {
            if ((st->tar_vel ^ st->cur_vel) < 0)
                Sigmoid_Interface.SetSpeed(&st->sigmoid, st->cur_vel, 0);
            else
                Sigmoid_Interface.SetSpeed(&st->sigmoid, st->cur_vel, st->tar_vel);
        }
    }
}
