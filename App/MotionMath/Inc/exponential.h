//
// Created by p0bedimka on 07.10.2026.
//

/**
 * @file exponential.h
 * @brief Заголовочный файл для экспоненциального планировщика скорости.
 *
 * Реализует профиль скорости по экспоненциальному закону:
 *   v(t) = v1 - dv * exp(-lambda * t)
 * где lambda — коэффициент затухания, подбираемый так, чтобы за время
 * t = 5 / lambda скорость достигла ~99.3% от целевого значения.
 */
#ifndef EXPONENTIAL_H
#define EXPONENTIAL_H

#include <stdint.h>
#include "../../Core/Inc/stepper_planner.h"

/**
 * @brief Структура состояния экспоненциального планировщика.
 */
typedef struct {
    StepperPlanner_t base;  ///< Базовый интерфейс

    int32_t  v0;            ///< Начальная скорость (шагов/с)
    int32_t  v1;            ///< Целевая скорость (шагов/с)
    uint16_t v_jump;        ///< Порог "мертвой зоны"
    uint32_t a;             ///< Характерное ускорение (используется для подбора lambda)
    float    lambda;        ///< Коэффициент затухания экспоненты (1/мс)
    float    t;             ///< Полное время переходного процесса (мс)
    int32_t  dv;            ///< Разность скоростей (v1 - v0)
} Exponential_t;

/// Экземпляр интерфейса для экспоненциального планировщика
extern const StepperPlanner_Interface Exponential_Itf;

#endif // EXPONENTIAL_H
