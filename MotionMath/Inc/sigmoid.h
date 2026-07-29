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
    uint16_t a;      ///< Ускорение (ед/с²)

    float k;    ///< Коэффициент крутизны сигмоиды
    float t0;   ///< Время достижения середины перехода (мс)
    int32_t dv; ///< Разность скоростей (v1 - v0)
} Sigmoid_Type;

typedef struct {
    uint32_t v_jump;
} Sigmoid_Init;

void Sigmoid_Initialization(Sigmoid_Type *sig, Sigmoid_Init *init);
void Sigmoid_DeInit(Sigmoid_Type *sig);
uint32_t Sigmoid_GetAcc(Sigmoid_Type *sig);
void Sigmoid_SetAcc(Sigmoid_Type *sig, uint32_t a);
int32_t Sigmoid_GetInitVel(Sigmoid_Type *sig);
int32_t Sigmoid_GetTargVel(Sigmoid_Type *sig);
void Sigmoid_SetVel(Sigmoid_Type *sig, int32_t v0, int32_t v1);
int32_t Sigmoid_GetVel(Sigmoid_Type *sig, float dt);

#endif
