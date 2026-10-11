//
// Created by p0bedimka on 02.07.2026.
//

/**
 * @file s-curve.h
 * @brief Заголовочный файл для планировщика скорости на основе S-образной кривой.
 *
 * Использует полиномиальную интерполяцию (Smoothstep), обеспечивая нулевые
 * начальные и конечные ускорения.
 */
#ifndef S_CURVE_H
#define S_CURVE_H

#include <stdint.h>
#include "../../Core/Inc/stepper_planner.h"

/**
 * @brief Структура состояния планировщика на основе S-кривой.
 */
typedef struct {
    StepperPlanner_t base;  ///< Базовый интерфейс

    int32_t v0;             ///< Начальная скорость
    int32_t v1;             ///< Целевая скорость
    int32_t dv;             ///< Разность скоростей (v1 - v0)

    uint16_t v_jump;        ///< Порог "мертвой зоны"
    uint32_t a;             ///< Максимальное ускорение
    float t;                ///< Общее время разгона/торможения
} Scurve_t;

/// Экземпляр интерфейса для планировщика на основе S-кривой
extern const StepperPlanner_Interface Scurve_Itf;

#endif // S_CURVE_H
