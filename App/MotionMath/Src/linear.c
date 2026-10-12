//
// Created by p0bedimka on 07.10.2026.
//

/**
 * @file linear.c
 * @brief Реализация алгоритма планирования скорости с постоянным ускорением.
 *
 * Скорость изменяется по линейному закону:
 *   v(t) = v0 + slope * t,  где slope = dv / t
 * Время перехода вычисляется как:
 *   t = 1000 * |dv| / a  (в миллисекундах, исходя из a в шагах/с^2)
 */
#include "linear.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
 * @brief Инициализация линейного планировщика.
 *
 * Защищает от некорректных значений ускорения и порога "мертвой зоны"
 * (они не должны быть меньше 1).
 *
 * @param pl Указатель на базовую структуру планировщика.
 */
void Linear_Init(StepperPlanner_t *pl) {
    if (pl == NULL) return;
    Linear_t *lin = (Linear_t *)pl;

    if (lin->a < 1) lin->a = 1;
    if (lin->v_jump < 1) lin->v_jump = 1;
}

/** @brief Получение текущего значения ускорения. */
uint32_t Linear_GetAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Linear_t *)pl)->a;
}

/** @brief Получение начальной скорости. */
int32_t Linear_GetInitVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Linear_t *)pl)->v0;
}

/** @brief Получение целевой скорости. */
int32_t Linear_GetTargVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Linear_t *)pl)->v1;
}

/** @brief Установка нового значения ускорения. */
void Linear_SetAcc(StepperPlanner_t *pl, const uint32_t a) {
    if (pl == NULL) return;
    ((Linear_t *)pl)->a = a;
}

/**
 * @brief Установка начальной и целевой скоростей с пересчётом параметров.
 *
 * Вычисляет:
 *   - dv  = v1 - v0  (знак учитывает направление)
 *   - t   = 1000 * |dv| / a  (время перехода в мс)
 *   - slope = dv / t (наклон линейной зависимости, шаг/мс)
 *
 * @param pl Указатель на планировщик.
 * @param v0 Начальная скорость.
 * @param v1 Целевая скорость.
 */
void Linear_SetVel(StepperPlanner_t *pl, int32_t v0, int32_t v1) {
    if (pl == NULL) return;
    Linear_t *lin = (Linear_t *)pl;

    lin->v0 = v0;
    lin->v1 = v1;
    lin->dv = lin->v1 - lin->v0;

    // Время перехода (мс). Коэффициент 1000 — перевод из с в мс.
    lin->t = 1000.0f * (float)abs(lin->dv) / (float)lin->a;

    // Наклон (шаг/мс). Защита от деления на практически нулевое время.
    if (lin->t > 0.001f)
        lin->slope = (float)lin->dv / lin->t;
    else
        lin->slope = 0.0f;
}

/**
 * @brief Вычисление текущей скорости в момент времени dt.
 *
 * Используется линейная интерполяция:
 *   - если dt <= 0       -> v = v0
 *   - если dt >= t       -> v = v1
 *   - иначе              -> v = v0 + slope * dt
 *
 * Если итоговая скорость попадает в "мёртвую зону" (|v| < v_jump),
 * она обнуляется — это учитывает трение покоя.
 *
 * @param pl Указатель на планировщик.
 * @param dt Время от начала перехода (мс).
 * @return Текущая скорость в шагах/с.
 */
int32_t Linear_GetVel(StepperPlanner_t *pl, float dt) {
    if (pl == NULL) return 0;
    Linear_t *lin = (Linear_t *)pl;
    int32_t v;

    if (dt <= 0.0f)
        v = lin->v0;
    else if (dt >= lin->t)
        v = lin->v1;
    else
        v = lin->v0 + (int32_t)roundf(lin->slope * dt);

    // Обработка "мёртвой зоны"
    if (abs(v) < lin->v_jump)
        v = 0;

    return v;
}

/** @brief Получение полного времени разгона/торможения (мс). */
uint32_t Linear_GetTimeAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return (uint32_t)((Linear_t *)pl)->t;
}

/// Таблица функций для линейного планировщика
const StepperPlanner_Interface Linear_Itf = {
    .Init       = Linear_Init,
    .GetAcc     = Linear_GetAcc,
    .SetAcc     = Linear_SetAcc,
    .GetInitVel = Linear_GetInitVel,
    .GetTargVel = Linear_GetTargVel,
    .SetVel     = Linear_SetVel,
    .GetVel     = Linear_GetVel,
    .GetTimeAcc = Linear_GetTimeAcc,
};
