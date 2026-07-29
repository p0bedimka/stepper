//
// Created by pobedimka on 02.07.2026.
//
#include "sigmoid.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
 * @brief Инициализация сигмоидальной кривой
 * @param sig        Указатель на структуру сигмоидальной кривой
 * @param init    Прыжок от нулевой скорости (недопустимо малая скорость)
 */
void Sigmoid_Initialization(Sigmoid_Type *sig, Sigmoid_Init *init) {
    if (sig != NULL) {
        memset(sig, 0, sizeof(Sigmoid_Type));
        sig->v_jump = init->v_jump;
        if (init->k_jump < 1)
            init->k_jump = 1;
        sig->k_jump = init->k_jump;
    }
}

/**
 * @brief Деинициализация структуры сигмоидальной кривой (сброс состояния)
 * @param sig Указатель на структуру сигмоидальной кривой
 */
void Sigmoid_DeInit(Sigmoid_Type *sig) {
    memset(sig, 0, sizeof(Sigmoid_Type));
}

uint32_t Sigmoid_GetAcc(Sigmoid_Type *sig) {
    uint32_t acc = 0;
    if (sig != NULL)
        acc = sig->a;
    return acc;
}

int32_t Sigmoid_GetInitVel(Sigmoid_Type *sig) {
    int32_t v = 0;
    if (sig != NULL)
        v = sig->v0;
    return v;
}

int32_t Sigmoid_GetTargVel(Sigmoid_Type *sig) {
    int32_t v = 0;
    if (sig != NULL)
        v = sig->v1;
    return v;
}

/**
 * @brief Установка ускорения
 * @param pl Указатель на структуру сигмоидальной кривой
 * @param a  Ускорение (ед/с²)
 */
void Sigmoid_SetAcc(Sigmoid_Type *pl, const uint32_t a) {
    if (pl != NULL)
        pl->a = a;
}

/**
 * @brief Установка начальной и конечной скорости
 * @param sig Указатель на структуру сигмоидальной кривой
 * @param v0 Начальная скорость (ед/с)
 * @param v1 Конечная скорость (ед/с)
 * @details Вычисляет параметры сигмоиды: dv, t0 и k
 */
void Sigmoid_SetVel(Sigmoid_Type *sig, int32_t v0, int32_t v1) {
    if (sig == NULL)
        return;

    /* Сохраняем начальную скорость */
    sig->v0 = v0;
    sig->v1 = v1;

    /* Вычисляем разность скоростей */
    sig->dv = v1 > v0 ? sig->v1 - sig->v0 + 1 : sig->v1 - sig->v0 - 1;

    /* t0 = время достижения середины перехода (центр сигмоиды)
     * 500 = 1000 мс / 2 (половина времени разгона при линейном ускорении) */
    sig->t0 = 500.f * (float)abs(sig->dv) / (float)sig->a;

    /* k = коэффициент крутизны сигмоиды */
    sig->k = logf((float)abs(sig->dv)) / sig->t0;

    /* Находим время t_jump / 4, когда |v(t)| достигает v_jump */
    float ratio = fabsf((float)sig->dv) / (float)sig->v_jump;
    sig->t_jump = (sig->t0 - logf(ratio - 1.f) / sig->k) / sig->k_jump;
}

/**
 * @brief Получение следующего значения скорости и дистанции
 * @param sig Указатель на структуру сигмоидальной кривой
 * @param dt  Приращение времени (мс)
 * @return   Структура с результатом расчёта
 * @details Вычисляет скорость по сигмоидальной функции:
 *          v(t) = v0 + dv / (1 + exp(-k*(t - t0)))
 *          Если включён расчёт дистанции, интегрирует скорость:
 *          pos = v0*t + dv/k * ln(1 + e^(k*(t-t0)))
 */
int32_t Sigmoid_GetVel(Sigmoid_Type *sig, float dt) {
    if (sig == NULL)
        return 0;

    /* Вычисляем аргумент экспоненты сигмоиды */
    float arg = sig->k * (dt - sig->t0);

    /* Расчёт скорости по формуле сигмоиды: v(t) = v0 + dv / (1 + exp(-k*(t - t0))) */
    float v = (float)sig->v0 + (float)sig->dv / (1.f + expf(-arg));

    /* Устраняем не желательную скорость*/
    if ((float)sig->v_jump > fabsf(v)) {
        if ((abs(sig->v0) < sig->v_jump && sig->t_jump < dt) ||
            (abs(sig->v1) < sig->v_jump && sig->t_jump < ((2 * sig->t0) - dt)))
            v = (v > 0) ? (float)sig->v_jump : -(float)sig->v_jump;
        else
            v = 0;
    }
    return (int32_t)roundf(v);
}

uint32_t Sigmoid_GetTimeAcc(Sigmoid_Type *sig) {
    return (uint32_t)(sig->t0 * 2.f);
}
