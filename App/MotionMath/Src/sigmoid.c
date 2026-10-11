//
// Created by pobedimka on 02.07.2026.
//
#include "sigmoid.h"

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
 * - Коэффициент прыжка не менее 1
 */
void Sigmoid_Init(StepperPlanner_t *pl) {
    if (pl == NULL) return;

    Sigmoid_t *sig = (Sigmoid_t *)pl;

    if (sig->a < 1)
        sig->a = 1;

    if (sig->v_jump < 1)
        sig->v_jump = 1;

    if (sig->k_jump < 1)
        sig->k_jump = 1;

    sig->eps = 0.01f;
}

/**
 * @brief Получение текущего ускорения
 * @param pl Указатель на структуру сигмоиды
 * @return Ускорение (импульсов/сек²)
 */
uint32_t Sigmoid_GetAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;

    Sigmoid_t *sig = (Sigmoid_t *)pl;

    return sig->a;
}

/**
 * @brief Установка ускорения
 * @param pl Указатель на структуру сигмоиды
 * @param a Ускорение (импульсов/сек²)
 */
void Sigmoid_SetAcc(StepperPlanner_t *pl, const uint32_t a) {
    if (pl == NULL) return;

    Sigmoid_t *sig = (Sigmoid_t *)pl;

    sig->a = a;
}

/**
 * @brief Получение начальной скорости
 * @param pl Указатель на структуру сигмоиды
 * @return Начальная скорость (импульсов/сек)
 */
int32_t Sigmoid_GetInitVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;

    Sigmoid_t *sig = (Sigmoid_t *)pl;

    return sig->v0;
}

/**
 * @brief Получение целевой скорости
 * @param pl Указатель на структуру сигмоиды
 * @return Целевая скорость (импульсов/сек)
 */
int32_t Sigmoid_GetTargVel(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;

    Sigmoid_t *sig = (Sigmoid_t *)pl;

    return sig->v1;
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
void Sigmoid_SetVel(StepperPlanner_t *pl, int32_t v0, int32_t v1) {
    if (pl == NULL) return;

    Sigmoid_t *sig = (Sigmoid_t *)pl;

    /* Сохраняем начальную скорость */
    sig->v0 = v0;
    sig->v1 = v1;

    /* Вычисляем разность скоростей */
    sig->dv = sig->v1 - sig->v0;

    /* t0 = время достижения середины перехода (центр сигмоиды)
     * 500 = 1000 мс / 2 (половина времени разгона при линейном ускорении) */
    sig->t0 = 500.f * (float)abs(sig->dv) / (float)sig->a;

    /* k = коэффициент крутизны сигмоиды */
    sig->k = -logf(sig->eps / (float)abs(sig->dv)) / sig->t0;

    /* Находим время t_jump / k_jump, когда |v(t)| достигает v_jump */
    float ratio = fabsf((float)sig->dv) / (float)sig->v_jump;
    sig->t_jump = (sig->t0 - logf(ratio - 1.f) / sig->k) / sig->k_jump;
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
int32_t Sigmoid_GetVel(StepperPlanner_t *pl, float dt) {
    if (pl == NULL) return 0;

    Sigmoid_t *sig = (Sigmoid_t *)pl;

    /* Вычисляем аргумент экспоненты сигмоиды */
    float arg = sig->k * (dt - sig->t0);

    /* Расчёт скорости по формуле сигмоиды: v(t) = v0 + dv / (1 + exp(-k*(t - t0))) */
    int32_t v = sig->v0 + (int32_t)roundf((float)sig->dv / (1.f + expf(-arg)));

    /* Устраняем не желательную скорость*/
    if (abs(v) < sig->v_jump) {
        if ((abs(sig->v0) < sig->v_jump && sig->t_jump < dt) ||
            (abs(sig->v1) < sig->v_jump && sig->t_jump < ((2 * sig->t0) - dt)))
            v = (v >= 0) ? sig->v_jump : -sig->v_jump;
        else
            v = 0;
    }

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
uint32_t Sigmoid_GetTimeAcc(StepperPlanner_t *pl) {
    if (pl == NULL) return 0;

    Sigmoid_t *sig = (Sigmoid_t *)pl;

    return (uint32_t)(sig->t0 * 2.f);
}

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
