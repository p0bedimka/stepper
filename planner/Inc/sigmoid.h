//
// Created by pobedimka on 02.07.2026.
//

#ifndef SIGMOID_H
#define SIGMOID_H

#include <stdbool.h>
#include <stdint.h>

/** Флаг расчёта дистанции: true - вычислять пройденное расстояние */
#define CALCULATE_DISTANCE      false
/** Коэффициент прыжка от нулевой скорости */
#define K_JUMP                  0.1f

/**
 * @brief Структура результата расчёта сигмоидальной кривой
 */
typedef struct {
    int32_t v;  ///< Текущая скорость
    int32_t d;  ///< Приращение дистанции за текущий шаг
    bool c; ///< Флаг завершения разгона: true - разгон окончен
} Sigmoid_Res;

/**
 * @brief Структура сигмоидальной кривой
 * @details Хранит все параметры движения и состояние расчёта
 */
typedef struct {
    int32_t v0; ///< Начальная скорость (ед/с)
    int32_t v1; ///< Конечная скорость (ед/с)

    uint16_t v_jump; ///< Прыжок от нулевой скорости (недопустимо малая скорость)
    uint16_t a;      ///< Ускорение (ед/с²)

    float k;  ///< Коэффициент крутизны сигмоиды
    float t0; ///< Время достижения середины перехода (мс)
    float dt; ///< Накопленное время с начала разгона (мс)
    int32_t dv;   ///< Разность скоростей (v1 - v0)

#if CALCULATE_DISTANCE
    int32_t p; ///< Абсолютная позиция от начала движения
#endif
} Sigmoid_Type;

/**
 * @brief Интерфейсная структура с указателями на функции сигмоидальной кривой
 */
typedef struct {
    /**
     * @brief Инициализация сигмоидальной кривой
     * @param pl         Указатель на структуру сигмоидальной кривой
     * @param v_jump    Прыжок от нулевой скорости (недопустимо малая скорость)
     */
    void (*Init)(Sigmoid_Type *pl, uint16_t v_jump);
    /**
     * @brief Деинициализация структуры сигмоидальной кривой (сброс состояния)
     * @param pl Указатель на структуру сигмоидальной кривой
     */
    void (*DeInit)(Sigmoid_Type *pl);
    uint16_t (*GetAccel)(Sigmoid_Type *p);
    /**
     * @brief Установка ускорения
     * @param pl  Указатель на структуру сигмоидальной кривой
     * @param a  Ускорение (ед/с²)
     */
    void (*SetAccel)(Sigmoid_Type *pl, uint16_t a);
    /**
     * @brief Установка начальной и конечной скорости
     * @param pl Указатель на структуру сигмоидальной кривой
     * @param v0 Начальная скорость (ед/с)
     * @param v1 Конечная скорость (ед/с)
     * @details Вычисляет параметры сигмоиды: dv, t0 и k
     */
    void (*SetSpeed)(Sigmoid_Type *pl, int32_t v0, int32_t v1);
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
    Sigmoid_Res (*GetNextRes)(Sigmoid_Type *pl, float t);
    bool (*IsSteadyVel)(Sigmoid_Type *pl);
} Sigmoid_Itf;

extern const Sigmoid_Itf Sigmoid_Interface;

#endif
