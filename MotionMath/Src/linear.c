//
// Created by Pobed on 07.10.2026.
//
//
// Created for linear trajectory
//
#include "linear.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

void Linear_Init(Planner_t *pl) {
    if (pl == NULL) return;

    Linear_t *lin = (Linear_t *)pl;

    if (lin->a < 1)
        lin->a = 1;

    if (lin->v_jump < 1)
        lin->v_jump = 1;
}

uint32_t Linear_GetAcc(Planner_t *pl) {
    if (pl == NULL) return 0;

    Linear_t *lin = (Linear_t *)pl;

    return lin->a;
}

int32_t Linear_GetInitVel(Planner_t *pl) {
    if (pl == NULL) return 0;

    Linear_t *lin = (Linear_t *)pl;

    return lin->v0;
}

int32_t Linear_GetTargVel(Planner_t *pl) {
    if (pl == NULL) return 0;

    Linear_t *lin = (Linear_t *)pl;

    return lin->v1;
}

void Linear_SetAcc(Planner_t *pl, const uint32_t a) {
    if (pl == NULL) return;

    Linear_t *lin = (Linear_t *)pl;

    lin->a = a;
}

void Linear_SetVel(Planner_t *pl, int32_t v0, int32_t v1) {
    if (pl == NULL) return;

    Linear_t *lin = (Linear_t *)pl;

    lin->v0 = v0;
    lin->v1 = v1;

    /* Вычисляем разность скоростей */
    lin->dv = lin->v1 - lin->v0;

    /* Полное время разгона T_sec = |dv| / a. Переводим в мс */
    lin->t = 1000.0f * (float)abs(lin->dv) / (float)lin->a;

    /* Наклон (изменение скорости за 1 мс) */
    if (lin->t > 0.001f)
        lin->slope = (float)lin->dv / lin->t;
    else
        lin->slope = 0.0f;
}

int32_t Linear_GetVel(Planner_t *pl, float dt) {
    if (pl == NULL) return 0;

    Linear_t *lin = (Linear_t *)pl;

    int32_t v;

    /* Жесткое ограничение по времени T, после которого скорость равна v1 */
    if (dt <= 0.0f)
        v = lin->v0;
    else if (dt >= lin->t)
        v = lin->v1;
    else
        v = lin->v0 + (int32_t)roundf(lin->slope * dt);

    /* Устраняем нежелательную скорость */
    if (abs(v) < lin->v_jump)
        v = 0;

    return v;
}

uint32_t Linear_GetTimeAcc(Planner_t *pl) {
    if (pl == NULL) return 0;

    Linear_t *lin = (Linear_t *)pl;

    return (uint32_t)lin->t;
}

const Planner_Interface Linear_Itf = {
    .Init       = Linear_Init,
    .GetAcc     = Linear_GetAcc,
    .SetAcc     = Linear_SetAcc,
    .GetInitVel = Linear_GetInitVel,
    .GetTargVel = Linear_GetTargVel,
    .SetVel     = Linear_SetVel,
    .GetVel     = Linear_GetVel,
    .GetTimeAcc = Linear_GetTimeAcc,
};
