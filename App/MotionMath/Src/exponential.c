//
// Created by p0bedimka on 07.10.2026.
//

/**
 * @file exponential.c
 * @brief Реализация алгоритма планирования скорости по экспоненциальному закону.
 *
 * Закон изменения скорости:
 *   v(t) = v1 - dv * exp(-lambda * t)
 *
 * Коэффициент затухания lambda подбирается из условия:
 *   lambda = a / (200 * |dv|)
 *
 * Время переходного процесса принимается равным 5 постоянным времени:
 *   t = 5 / lambda
 * (за это время экспонента exp(-5) ≈ 0.0067, т.е. достигается ~99.3% цели).
 */
#include "exponential.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
 * @brief Инициализация экспоненциального планировщика.
 *
 * Защищает от некорректных значений ускорения и порога "мертвой зоны".
 *
 * @param pl Указатель на базовую структуру планировщика.
 */
void Exp_Init(StepperPlanner_t *pl) {
    if (pl == NULL) return;
    Exponential_t *exp = (Exponential_t *)pl;

    if (exp->a < 1) exp->a = 1;
    if (exp->v_jump < 1) exp->v_jump = 1;
}

/** @brief Получение текущего значения ускорения. */
uint32_t Exp_GetAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Exponential_t *)pl)->a;
}

/** @brief Получение начальной скорости. */
int32_t Exp_GetInitVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Exponential_t *)pl)->v0;
}

/** @brief Получение целевой скорости. */
int32_t Exp_GetTargVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return ((Exponential_t *)pl)->v1;
}

/** @brief Установка нового значения ускорения. */
void Exp_SetAcc(StepperPlanner_t *pl, const uint32_t a) {
    if (pl == NULL) return;
    ((Exponential_t *)pl)->a = a;
}

/**
 * @brief Установка начальной и целевой скоростей с пересчётом параметров.
 *
 * Вычисляет:
 *   - dv     = v1 - v0
 *   - lambda = a / (200 * |dv|)  — коэффициент затухания
 *   - t      = 5 / lambda        — время переходного процесса (5 постоянных времени)
 *
 * @param pl Указатель на планировщик.
 * @param v0 Начальная скорость.
 * @param v1 Целевая скорость.
 */
void Exp_SetVel(StepperPlanner_t *pl, int32_t v0, int32_t v1) {
    if (pl == NULL) return;
    Exponential_t *exp = (Exponential_t *)pl;

    exp->v0 = v0;
    exp->v1 = v1;
    exp->dv = exp->v1 - exp->v0;

    // Коэффициент затухания. Чем больше ускорение — тем быстрее переход.
    exp->lambda = (float)exp->a / (100.0f * (float)abs(exp->dv));

    // Время переходного процесса (5 постоянных времени -> ~99.3% достижения цели)
    if (exp->lambda > 0.000001f)
        exp->t = 10.0f / exp->lambda;
    else
        exp->t = 0.0f;
}

/**
 * @brief Вычисление текущей скорости в момент времени dt.
 *
 * Используется экспоненциальный закон:
 *   - если dt <= 0       -> v = v0
 *   - если dt >= t       -> v = v1
 *   - иначе              -> v = v1 - dv * exp(-lambda * dt)
 *
 * Если итоговая скорость попадает в "мёртвую зону" (|v| < v_jump),
 * она обнуляется — это учитывает трение покоя.
 *
 * @param pl Указатель на планировщик.
 * @param dt Время от начала перехода (мс).
 * @return Текущая скорость в шагах/с.
 */
int32_t Exp_GetVel(StepperPlanner_t *pl, float dt) {
    if (pl == NULL) return 0;
    Exponential_t *exp = (Exponential_t *)pl;
    int32_t v;

    if (dt <= 0.0f)
        v = exp->v0;
    else if (dt >= exp->t)
        v = exp->v1;
    else
        v = exp->v1 - (int32_t)roundf((float)exp->dv * expf(-exp->lambda * dt));

    // Обработка "мёртвой зоны"
    if (abs(v) < exp->v_jump)
        v = 0;

    return v;
}

/** @brief Получение полного времени переходного процесса (мс). */
uint32_t Exp_GetTimeAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;
    return (uint32_t)((Exponential_t *)pl)->t;
}

/// Таблица функций для экспоненциального планировщика
const StepperPlanner_Interface Exponential_Itf = {
    .Init       = Exp_Init,
    .GetAcc     = Exp_GetAcc,
    .SetAcc     = Exp_SetAcc,
    .GetInitVel = Exp_GetInitVel,
    .GetTargVel = Exp_GetTargVel,
    .SetVel     = Exp_SetVel,
    .GetVel     = Exp_GetVel,
    .GetTimeAcc = Exp_GetTimeAcc,
};
