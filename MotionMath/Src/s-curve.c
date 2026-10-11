//
// Created by pobedimka on 02.07.2026.
//
#include "s-curve.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
 * @brief Инициализация сигмоидальной кривой
 * @param pl Указатель на структуру сигмоиды
 *
 * Проверяет и корректирует параметры:
 * - Ускорение не менее 1
 * - Минимальная скорость не менее 1
 */
void Scurve_Init(Planner_t *pl) {
    if (pl == NULL) return;

    Scurve_t *scurve = (Scurve_t *)pl;

    if (scurve->a < 1)
        scurve->a = 1;

    if (scurve->v_jump < 1)
        scurve->v_jump = 1;
}

/**
 * @brief Получение текущего ускорения
 * @param pl Указатель на структуру сигмоиды
 * @return Ускорение (импульсов/сек²)
 */
uint32_t Scurve_GetAcc(Planner_t *pl) {
    if (pl == NULL) return 0;

    Scurve_t *scurve = (Scurve_t *)pl;

    return scurve->a;
}

/**
 * @brief Установка ускорения
 * @param pl Указатель на структуру сигмоиды
 * @param a Ускорение (импульсов/сек²)
 */
void Scurve_SetAcc(Planner_t *pl, const uint32_t a) {
    if (pl == NULL) return;

    Scurve_t *scurve = (Scurve_t *)pl;

    scurve->a = a;
}

/**
 * @brief Получение начальной скорости
 * @param pl Указатель на структуру сигмоиды
 * @return Начальная скорость (импульсов/сек)
 */
int32_t Scurve_GetInitVel(Planner_t *pl) {
    if (pl == NULL) return 0;

    Scurve_t *scurve = (Scurve_t *)pl;

    return scurve->v0;
}

/**
 * @brief Получение целевой скорости
 * @param pl Указатель на структуру сигмоиды
 * @return Целевая скорость (импульсов/сек)
 */
int32_t Scurve_GetTargVel(Planner_t *pl) {
    if (pl == NULL) return 0;

    Scurve_t *scurve = (Scurve_t *)pl;

    return scurve->v1;
}

/**
 * @brief Установка начальной и конечной скорости
 * @param pl Указатель на структуру сигмоиды
 * @param v0 Начальная скорость (импульсов/сек)
 * @param v1 Конечная скорость (импульсов/сек)
 *
 * @details Алгоритм расчёта параметров:
 * 1. Сохраняет v0 и v1
 * 2. Вычисляет разность скоростей dv с учётом знака
 * 3. Рассчитывает t0 = 500 * |dv| / a (время до середины перехода)
 * 4. Рассчитывает k = ln(|dv|) / t0 (крутизна сигмоиды)
 * 5. Вычисляет t_jump - время достижения минимальной скорости
 */
void Scurve_SetVel(Planner_t *pl, int32_t v0, int32_t v1) {
    if (pl == NULL) return;

    Scurve_t *scurve = (Scurve_t *)pl;

    /* Сохраняем начальную скорость */
    scurve->v0 = v0;
    scurve->v1 = v1;

    /* Вычисляем разность скоростей */
    scurve->dv = scurve->v1 - scurve->v0;

    /* Время разгона до v1 */
    scurve->t = 1000.f * (float)abs(scurve->dv) / (float)scurve->a;
}

/**
 * @brief Получение скорости в заданный момент времени
 * @param pl Указатель на структуру сигмоиды
 * @param dt Приращение времени от начала движения (мс)
 * @return Текущая скорость (импульсов/сек)
 *
 * @details Формула расчёта:
 * v(t) = v0 + dv / (1 + exp(-k*(t - t0)))
 *
 * Особенности:
 * - Учитывает минимальную скорость v_jump
 * - При достижении минимальной скорости удерживает её
 * - Возвращает 0, если скорость ниже порога в начале/конце движения
 */
int32_t Scurve_GetVel(Planner_t *pl, float dt) {
    if (pl == NULL) return 0;

    Scurve_t *scurve = (Scurve_t *)pl;

    int32_t v;

    /* Ограничиваем время диапазоном [0, T], так как полином за этими
       пределами уходит в бесконечность (в отличие от сигмоиды) */
    if (scurve->t < 0.001f || dt <= 0.0f)
        v = scurve->v0;
    else if (dt >= scurve->t)
        v = scurve->v1;
    else {
        /* Нормализованное время tau = t / T (от 0 до 1) */
        const float tau = dt / scurve->t;

        /* Формула скорости */
        v = scurve->v0 + (int32_t)roundf((float)scurve->dv * tau * tau * (3.0f - 2.0f * tau));
    }

    /* Устраняем нежелательную скорость (защита от микро-шагов и люфтов) */
    if (abs(v) < scurve->v_jump)
            v = 0;

    return v;
}

/**
 * @brief Получение времени разгона
 * @param pl Указатель на структуру сигмоиды
 * @return Время разгона (мс)
 *
 * Возвращает полное время переходного процесса:
 * TimeAcc = 2 * t0
 */
uint32_t Scurve_GetTimeAcc(Planner_t *pl) {
    if (pl == NULL) return 0;

    Scurve_t *scurve = (Scurve_t *)pl;

    return (uint32_t)scurve->t;
}

const Planner_Interface Scurve_Itf = {
    .Init       = Scurve_Init,
    .GetAcc     = Scurve_GetAcc,
    .SetAcc     = Scurve_SetAcc,
    .GetInitVel = Scurve_GetInitVel,
    .GetTargVel = Scurve_GetTargVel,
    .SetVel     = Scurve_SetVel,
    .GetVel     = Scurve_GetVel,
    .GetTimeAcc = Scurve_GetTimeAcc,
};
