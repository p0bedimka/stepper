//
// Created by pobedimka on 02.07.2026.
//
#include "sigmoid.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Прототипы функций */
void Sigmoid_Init(Sigmoid_Type *pl, uint16_t v_jump);
void Sigmoid_DeInit(Sigmoid_Type *pl);
uint16_t Sigmoid_GetAccel(Sigmoid_Type *pl);
void Sigmoid_SetAccel(Sigmoid_Type *pl, uint16_t a);
void Sigmoid_SetSpeed(Sigmoid_Type *pl, int32_t v0, int32_t v1);
Sigmoid_Res Sigmoid_GetNextResult(Sigmoid_Type *pl, float t);
bool Sigmoid_IsSteadyVel(Sigmoid_Type *pl);

/**
 * @brief Интерфейсная таблица с указателями на реализации функций
 */
const Sigmoid_Itf Sigmoid_Interface = {
    .Init        = Sigmoid_Init,
    .DeInit      = Sigmoid_DeInit,
    .GetAccel    = Sigmoid_GetAccel,
    .SetAccel    = Sigmoid_SetAccel,
    .SetSpeed    = Sigmoid_SetSpeed,
    .GetNextRes  = Sigmoid_GetNextResult,
    .IsSteadyVel = Sigmoid_IsSteadyVel,
};

/**
 * @brief Инициализация сигмоидальной кривой
 * @param pl        Указатель на структуру сигмоидальной кривой
 * @param v_jump    Прыжок от нулевой скорости (недопустимо малая скорость)
 */
void Sigmoid_Init(Sigmoid_Type *pl, uint16_t v_jump) {
    if (pl != NULL) {
        memset(pl, 0, sizeof(Sigmoid_Type));
        pl->v_jump = v_jump;
    }
}

/**
 * @brief Деинициализация структуры сигмоидальной кривой (сброс состояния)
 * @param pl Указатель на структуру сигмоидальной кривой
 */
void Sigmoid_DeInit(Sigmoid_Type *pl) {
    memset(pl, 0, sizeof(Sigmoid_Type));
}

uint16_t Sigmoid_GetAccel(Sigmoid_Type *pl) {
    uint16_t acc = 0;
    if (pl != NULL)
        acc = pl->a;
    return acc;
}

/**
 * @brief Установка ускорения
 * @param pl Указатель на структуру сигмоидальной кривой
 * @param a  Ускорение (ед/с²)
 */
void Sigmoid_SetAccel(Sigmoid_Type *pl, const uint16_t a) {
    if (pl != NULL)
        pl->a = a;
}

/**
 * @brief Установка начальной и конечной скорости
 * @param pl Указатель на структуру сигмоидальной кривой
 * @param v0 Начальная скорость (ед/с)
 * @param v1 Конечная скорость (ед/с)
 * @details Вычисляет параметры сигмоиды: dv, t0 и k
 */
void Sigmoid_SetSpeed(Sigmoid_Type *pl, int32_t v0, int32_t v1) {
    if (pl == NULL)
        return;

    /* Сбрасываем накопленное время и позицию */
    pl->dt = 0;
#if CALCULATE_DISTANCE
    pl->p = 0;
#endif

    /* Сохраняем начальную скорость */
    pl->v0 = v0;
    pl->v1 = v1;

    /* Вычисляем разность скоростей */
    pl->dv = v1 > v0 ? pl->v1 - pl->v0 + 1 : pl->v1 - pl->v0 - 1;

    /* t0 = время достижения середины перехода (центр сигмоиды)
     * 500 = 1000 мс / 2 (половина времени разгона при линейном ускорении) */
    pl->t0 = 500.f * (float)abs(pl->dv) / (float)pl->a;

    /* k = коэффициент крутизны сигмоиды */
    pl->k = logf((float)abs(pl->dv)) / pl->t0;
}

/**
 * @brief Получение следующего значения скорости и дистанции
 * @param pl Указатель на структуру сигмоидальной кривой
 * @param t  Приращение времени (мс)
 * @return   Структура с результатом расчёта
 * @details Вычисляет скорость по сигмоидальной функции:
 *          v(t) = v0 + dv / (1 + exp(-k*(t - t0)))
 *          Если включён расчёт дистанции, интегрирует скорость:
 *          pos = v0*t + dv/k * ln(1 + e^(k*(t-t0)))
 */
Sigmoid_Res Sigmoid_GetNextResult(Sigmoid_Type *pl, float t) {
    Sigmoid_Res res = {0};

    if (pl == NULL)
        return res;

    /* Увеличиваем накопленное время на шаг t */
    pl->dt += t;

    /* Проверяем завершение разгона: время превысило 2*t0 */
    if (pl->dt >= 2.f * pl->t0) {
        res.v = pl->v1;
        res.c = true;
        return res;
    }

    /* Вычисляем аргумент экспоненты сигмоиды */
    float arg = pl->k * (pl->dt - pl->t0);

    /* Расчёт скорости по формуле сигмоиды:
     * v(t) = v0 + dv / (1 + exp(-k*(t - t0))) */
    float v = (float)pl->v0 + (float)pl->dv / (1.f + expf(-arg));

    /* Устраняем не желательную скорость*/
    if ((float)pl->v_jump > fabsf(v)) {
        if (fabsf(v) > K_JUMP * (float)pl->v_jump) {
            v = v > 0 ? (float)pl->v_jump : -(float)pl->v_jump;
        }
        else {
            v = 0;
        }
    }
    res.v = (int32_t)roundf(v);

#if CALCULATE_DISTANCE
    /* Интеграл от сигмоиды для вычисления пройденного пути */
    float term1 = logf(1.f + expf(arg));
    float term2 = logf(1.f + expf(-pl->k * pl->t0));

    float inv_k = 1.f / pl->k;

    /* Позиция = интеграл от скорости:
     * pos = v0*t + dv/k * ln(1 + e^(k*(t-t0))) */
    int32_t pos = (int32_t)roundf((float)pl->v0 * pl->dt + (float)pl->dv * inv_k * (term1 - term2));

    /* Вычисляем приращение дистанции за текущий шаг */
    res.d = pos - pl->p;

    /* Обновляем абсолютную позицию */
    pl->p = pos;
#endif

    return res;
}

bool Sigmoid_IsSteadyVel(Sigmoid_Type *pl) {
    bool status = false;
    if (pl != NULL) {
        if (pl->dt >= 2.f * pl->t0)
            status = true;
    }
    return status;
}
