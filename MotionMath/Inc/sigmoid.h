//
// Created by pobedimka on 02.07.2026.
//

#ifndef SIGMOID_H
#define SIGMOID_H

#include <stdint.h>

/**
 * @brief Структура сигмоидальной кривой
 * @details Хранит все параметры движения и состояние расчёта
 */
typedef struct {
    int32_t v0; ///< Начальная скорость (ед/с)
    int32_t v1; ///< Конечная скорость (ед/с)

    uint16_t v_jump; ///< Прыжок от нулевой скорости (недопустимо малая скорость)
    float t_jump;
    float k_jump;
    uint32_t a;      ///< Ускорение (ед/с²)

    float k;    ///< Коэффициент крутизны сигмоиды
    float t0;   ///< Время достижения середины перехода (мс)
    int32_t dv; ///< Разность скоростей (v1 - v0)
} Sigmoid_t;

void Sigmoid_Init(Sigmoid_t *sig);
uint32_t Sigmoid_GetAcc(Sigmoid_t *sig);
void Sigmoid_SetAcc(Sigmoid_t *sig, uint32_t a);
int32_t Sigmoid_GetInitVel(Sigmoid_t *sig);
int32_t Sigmoid_GetTargVel(Sigmoid_t *sig);
void Sigmoid_SetVel(Sigmoid_t *sig, int32_t v0, int32_t v1);
int32_t Sigmoid_GetVel(Sigmoid_t *sig, float dt);
uint32_t Sigmoid_GetTimeAcc(Sigmoid_t *sig);

#endif
