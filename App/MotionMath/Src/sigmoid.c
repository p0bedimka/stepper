//
// Created by p0bedimka on 02.07.2026.
//

/**
 * @file sigmoid.c
 * @brief Реализация алгоритма планирования скорости на основе сигмоиды.
 */
#include "sigmoid.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/** @brief Инициализация параметров. Защита от деления на ноль и невалидных значений. */
void Sigmoid_Init(StepperPlanner_t *pl) {
    if (pl == NULL) return;
    Sigmoid_t *sig = (Sigmoid_t *)pl;

    if (sig->a < 1) sig->a = 1;
    if (sig->v_jump < 1) sig->v_jump = 1;
    if (sig->k_jump < 1) sig->k_jump = 1;
    sig->eps = 0.01f;
}

/** @brief Получение ускорения. */
uint32_t Sigmoid_GetAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Sigmoid_t *)pl)->a;
}

/** @brief Установка ускорения. */
void Sigmoid_SetAcc(StepperPlanner_t *pl, const uint32_t a) {
    if (pl == NULL) return;
    ((Sigmoid_t *)pl)->a = a;
}

/** @brief Получение начальной скорости. */
int32_t Sigmoid_GetInitVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Sigmoid_t *)pl)->v0;
}

/** @brief Получение целевой скорости. */
int32_t Sigmoid_GetTargVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Sigmoid_t *)pl)->v1;
}

/**
 * @brief Установка скоростей и предварительный расчет параметров сигмоиды.
 *
 * Вычисляет время центра перехода (t0), крутизну (k) и время выхода
 * из мертвой зоны (t_jump).
 */
void Sigmoid_SetVel(StepperPlanner_t *pl, int32_t v0, int32_t v1) {
    if (pl == NULL) return;
    Sigmoid_t *sig = (Sigmoid_t *)pl;

    sig->v0 = v0;
    sig->v1 = v1;
    sig->dv = sig->v1 - sig->v0;

    // t0 = время достижения середины перехода (500 = 1000 мс / 2)
    sig->t0 = 500.f * (float)abs(sig->dv) / (float)sig->a;

    // k = коэффициент крутизны сигмоиды (зависит от eps)
    sig->k = -logf(sig->eps / (float)abs(sig->dv)) / sig->t0;

    // Расчет времени t_jump, когда |v(t)| достигает порога v_jump
    float ratio = fabsf((float)sig->dv) / (float)sig->v_jump;
    sig->t_jump = (sig->t0 - logf(ratio - 1.f) / sig->k) / sig->k_jump;
}

/**
 * @brief Вычисление текущей скорости в момент времени dt.
 *
 * Применяет логистическую функцию. Если скорость попадает в "мертвую зону"
 * (меньше v_jump), она обнуляется или приравнивается к границе зоны,
 * чтобы избежать "дребезга" и учесть трение покоя.
 */
int32_t Sigmoid_GetVel(StepperPlanner_t *pl, float dt) {
    if (pl == NULL) return 0;
    Sigmoid_t *sig = (Sigmoid_t *)pl;

    float arg = sig->k * (dt - sig->t0);
    int32_t v = sig->v0 + (int32_t)roundf((float)sig->dv / (1.f + expf(-arg)));

    // Обработка "мертвой зоны"
    if (abs(v) < sig->v_jump) {
        if ((abs(sig->v0) < sig->v_jump && sig->t_jump < dt) ||
            (abs(sig->v1) < sig->v_jump && sig->t_jump < ((2 * sig->t0) - dt))) {
            v = (v >= 0) ? sig->v_jump : -sig->v_jump;
        }
        else
            v = 0;
    }
    return v;
}

/** @brief Получение полного времени перехода (удвоенное время до центра сигмоиды). */
uint32_t Sigmoid_GetTimeAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return (uint32_t)(((Sigmoid_t *)pl)->t0 * 2.f);
}

/// Таблица функций для сигмоидального планировщика
const StepperPlanner_Interface Sigmoid_Itf = {
    .Init       = Sigmoid_Init,
    .GetAcc     = Sigmoid_GetAcc,
    .SetAcc     = Sigmoid_SetAcc,
    .GetInitVel = Sigmoid_GetInitVel,
    .GetTargVel = Sigmoid_GetTargVel,
    .SetVel     = Sigmoid_SetVel,
    .GetVel     = Sigmoid_GetVel,
    .GetTimeAcc = Sigmoid_GetTimeAcc,
};
