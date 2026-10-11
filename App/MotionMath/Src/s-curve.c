//
// Created by p0bedimka on 02.07.2026.
//

/**
 * @file s-curve.c
 * @brief Реализация алгоритма планирования скорости на основе S-кривой.
 *
 * Использует функцию Smoothstep (полином 3-й степени: 3*tau^2 - 2*tau^3)
 * для плавного изменения скорости без скачков ускорения.
 */
#include "s-curve.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/** @brief Инициализация параметров. */
void Scurve_Init(StepperPlanner_t *pl) {
    if (pl == NULL) return;
    Scurve_t *scurve = (Scurve_t *)pl;

    if (scurve->a < 1) scurve->a = 1;
    if (scurve->v_jump < 1) scurve->v_jump = 1;
}

/** @brief Получение ускорения. */
uint32_t Scurve_GetAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Scurve_t *)pl)->a;
}

/** @brief Установка ускорения. */
void Scurve_SetAcc(StepperPlanner_t *pl, const uint32_t a) {
    if (pl == NULL) return;
    ((Scurve_t *)pl)->a = a;
}

/** @brief Получение начальной скорости. */
int32_t Scurve_GetInitVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Scurve_t *)pl)->v0;
}

/** @brief Получение целевой скорости. */
int32_t Scurve_GetTargVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Scurve_t *)pl)->v1;
}

/**
 * @brief Установка скоростей и расчет общего времени перехода.
 *
 * Время перехода пропорционально модулю разности скоростей и обратно
 * пропорционально заданному ускорению.
 */
void Scurve_SetVel(StepperPlanner_t *pl, int32_t v0, int32_t v1) {
    if (pl == NULL) return;
    Scurve_t *scurve = (Scurve_t *)pl;

    scurve->v0 = v0;
    scurve->v1 = v1;
    scurve->dv = scurve->v1 - scurve->v0;
    scurve->t = 1000.f * (float)abs(scurve->dv) / (float)scurve->a;
}

/**
 * @brief Вычисление текущей скорости в момент времени dt.
 *
 * Используется полиномиальная интерполяция (Smoothstep).
 * Если скорость попадает в "мертвую зону" (меньше v_jump), она обнуляется.
 */
int32_t Scurve_GetVel(StepperPlanner_t *pl, float dt) {
    if (pl == NULL) return 0;
    Scurve_t *scurve = (Scurve_t *)pl;
    int32_t v;

    if (scurve->t < 0.001f || dt <= 0.0f)
        v = scurve->v0;
    else if (dt >= scurve->t)
        v = scurve->v1;
    else {
        // Нормализованное время [0..1]
        const float tau = dt / scurve->t;
        // Формула Smoothstep: 3*tau^2 - 2*tau^3
        v = scurve->v0 + (int32_t)roundf((float)scurve->dv * tau * tau * (3.0f - 2.0f * tau));
    }

    // Обработка "мертвой зоны"
    if (abs(v) < scurve->v_jump)
        v = 0;

    return v;
}

/** @brief Получение полного времени перехода. */
uint32_t Scurve_GetTimeAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return (uint32_t)((Scurve_t *)pl)->t;
}

/// Таблица функций для S-образного планировщика
const StepperPlanner_Interface Scurve_Itf = {
    .Init       = Scurve_Init,
    .GetAcc     = Scurve_GetAcc,
    .SetAcc     = Scurve_SetAcc,
    .GetInitVel = Scurve_GetInitVel,
    .GetTargVel = Scurve_GetTargVel,
    .SetVel     = Scurve_SetVel,
    .GetVel     = Scurve_GetVel,
    .GetTimeAcc = Scurve_GetTimeAcc,
};
