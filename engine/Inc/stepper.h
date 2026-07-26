//
// Created by Pobed on 05.07.2026.
//

#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"
#include "sigmoid.h"

typedef enum {
    STEPPER_FAILED = 0U,
    STEPPER_OK
} Stepper_Status;

typedef struct {
    struct TIM_Handler {
        TIM_TypeDef *instance;
        uint32_t max_period;
        uint32_t clock;
        uint16_t channel;
    } htim;

    struct GPIO_Handler {
        GPIO_TypeDef *port;
        uint16_t pin;
    } pul, dir, en;

    Sigmoid_Type sigmoid;

    volatile int32_t tar_vel;
    volatile int32_t cur_vel;

    volatile uint32_t period;
    volatile uint32_t pulse;

    volatile uint32_t accum_time;
    volatile uint32_t last_time;
} Stepper_Type;

typedef struct {
    Stepper_Status (*Init)(Stepper_Type *st, uint32_t psc);
    void (*DeInit)(Stepper_Type *st);
    void (*Enable)(Stepper_Type *st);
    void (*Disable)(Stepper_Type *st);
    uint16_t (*GetAcc)(Stepper_Type *st);
    Stepper_Status (*SetAcc)(Stepper_Type *motor, uint16_t acc);
    int32_t (*GetVel)(Stepper_Type *st);
    Stepper_Status (*SetVel)(Stepper_Type *motor, int32_t vel);
    void (*Stop)(Stepper_Type *motor);
    void (*TIM_IRQHandler)(Stepper_Type *motor);
    void (*Manager_IRQHandler)(Stepper_Type *motor);
} Stepper_Itf;

extern const Stepper_Itf Stepper_Interface;

#endif
