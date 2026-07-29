//
// Created by Pobed on 27.07.2026.
//
#include "stepper.h"

#include <stdlib.h>
#include <string.h>

struct Stepper_Type {
    TIM_Type htim;
    GPIO_Type pul, dir, en;

    Planner_Type pl;

    volatile int32_t tar_vel;
    volatile int32_t cur_vel;
    volatile uint32_t time;

    uint32_t (*GetTick)(void);
};

Stepper_Type* Stepper_Initialization(Stepper_Init *init) {
    if (init == NULL)
        return NULL;

    Stepper_Type *st = (Stepper_Type*)malloc(sizeof(Stepper_Type));
    if (st == NULL)
        return NULL;

    memset(st, 0, sizeof(Stepper_Type));

    st->GetTick = init->GetTick;
    HAL_Stepper_Itf.TIM_Init(&st->htim, &init->htim);
    HAL_Stepper_Itf.GPIO_Init(&st->pul, &init->pul);
    HAL_Stepper_Itf.GPIO_Init(&st->dir, &init->dir);
    HAL_Stepper_Itf.GPIO_Init(&st->en, &init->en);

    st->time = st->GetTick();

    Planner_Itf.Init(&st->pl, &init->pl);

    return st;
}

void Stepper_Deinitialization(Stepper_Type *st) {
    if (st != NULL) {
        free(st);
    }
}

void Stepper_Enable(Stepper_Type *st) {
    if (st != NULL) {
        HAL_Stepper_Itf.TIM_Enable(&st->htim);
        HAL_Stepper_Itf.TIM_EnableCompare(&st->htim);
        HAL_Stepper_Itf.GPIO_Set(&st->en);
    }
}

void Stepper_Disable(Stepper_Type *st) {
    if (st != NULL) {
        if (st->cur_vel == 0) {
            HAL_Stepper_Itf.TIM_DisableCompare(&st->htim);
            HAL_Stepper_Itf.TIM_Disable(&st->htim);
            HAL_Stepper_Itf.GPIO_Reset(&st->en);
        }
    }
}

uint32_t Stepper_GetAcceleration(Stepper_Type *st) {
    if (st == NULL) {
        return 0;
    }
    return Planner_Itf.GetAcc(&st->pl);
}

void Stepper_SetAcceleration(Stepper_Type *st, uint32_t acc) {
    if (st != NULL) {
        Planner_Itf.SetAcc(&st->pl, acc);
    }
}

int32_t Stepper_GetVelocity(Stepper_Type *st) {
    if (st == NULL) {
        return 0;
    }
    return st->cur_vel;
}

void Stepper_SetVelocity(Stepper_Type *st, int32_t vel) {
    if (st == NULL) {
        return;
    }

    if (HAL_Stepper_Itf.TIM_СanSetFrequency(&st->htim, abs(vel))) {
        st->tar_vel = vel;
    }
}

void Stepper_Stop(Stepper_Type *st) {
    if (st != NULL) {
        st->tar_vel = 0;
        st->cur_vel = 0;
        HAL_Stepper_Itf.TIM_SetFrequency(&st->htim, 0);
    }
}

void Stepper_TaskPlannerFunc(Stepper_Type *st) {

    if (st == NULL) {
        return;
    }

    int32_t tar_vel = Planner_Itf.GetTargVel(&st->pl);

    if (tar_vel != st->cur_vel) {
        uint32_t time = st->GetTick();
        uint32_t elapsed = 0;

        if (time >= st->time) {
            elapsed = time - st->time;
        } else {
            elapsed = (UINT32_MAX - st->time) + time + 1;
        }

        int32_t vel = Planner_Itf.GetVel(&st->pl, (float)elapsed);

        if (vel != st->cur_vel) {
            HAL_Stepper_Itf.TIM_SetFrequency(&st->htim, abs(vel));
            st->cur_vel = vel;
        }
    } else if (tar_vel != st->tar_vel) {
        if ((st->tar_vel ^ tar_vel) < 0 && tar_vel != 0 && st->tar_vel != 0) {
            Planner_Itf.SetVel(&st->pl, st->cur_vel, 0);
        } else {
            Planner_Itf.SetVel(&st->pl, st->cur_vel, st->tar_vel);
        }
        st->time = st->GetTick();
    }
}

const Stepper_Interface Stepper_Itf = {
    .Init = Stepper_Initialization,
    .DeInit = Stepper_Deinitialization,
    .Enable = Stepper_Enable,
    .Disable = Stepper_Disable,
    .GetAcc = Stepper_GetAcceleration,
    .SetAcc = Stepper_SetAcceleration,
    .GetVel = Stepper_GetVelocity,
    .SetVel = Stepper_SetVelocity,
    .Stop = Stepper_Stop,
    .TaskPlannerFunc = Stepper_TaskPlannerFunc
};