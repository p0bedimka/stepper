//
// Created by pobedimka on 02.07.2026.
//

#ifndef SIGMOID_H
#define SIGMOID_H

#include <stdint.h>

#include "stepper_planner.h"

/**
 * @brief Структура сигмоидальной кривой для планирования движения
 *
 * Реализует S-образный профиль скорости с использованием
 * сигмоидальной функции для обеспечения плавного разгона
 * и торможения без рывков.
 */
typedef struct {
    StepperPlanner_t base;

    int32_t v0;        /**< Начальная скорость (импульсов/сек) */
    int32_t v1;        /**< Конечная скорость (импульсов/сек) */

    uint16_t v_jump;   /**< Минимальная скорость (прыжок от нуля) */
    float t_jump;      /**< Время достижения минимальной скорости (мс) */
    float k_jump;      /**< Коэффициент для расчёта времени прыжка */
    uint32_t a;        /**< Ускорение (импульсов/сек²) */

    float k;           /**< Коэффициент крутизны сигмоиды */
    float t0;          /**< Время достижения середины перехода (мс) */
    int32_t dv;        /**< Разность скоростей (v1 - v0) */
    float eps;
} Sigmoid_t;

extern const StepperPlanner_Interface Sigmoid_Itf;

#endif
