//
// Created by Pobed on 07.10.2026.
//

#ifndef EXPONENTIAL_H
#define EXPONENTIAL_H

#include <stdint.h>

#include "planner.h"

/**
@brief Структура экспоненциальной кривой для планирования движения
Реализует профиль скорости: v(t) = v1 - (v1 - v0) * exp(-lambda * t)
Обеспечивает плавный разгон с максимальным ускорением в начале и его затуханием к концу.
*/
typedef struct {
    Planner_t *pl;

    int32_t v0;        /**< Начальная скорость (импульсов/сек) */
    int32_t v1;        /**< Конечная скорость (импульсов/сек) */

    uint16_t v_jump;   /**< Минимальная скорость (прыжок от нуля) */
    uint32_t a;        /**< Ускорение (импульсов/сек²) */

    float lambda;      /**< Коэффициент затухания (1/мс) */
    float t;           /**< Практическое время окончания разгона (мс) */
    int32_t dv;        /**< Разность скоростей (v1 - v0) */
} Exponential_t;

extern const Planner_Interface Exponential_Itf;

#endif
