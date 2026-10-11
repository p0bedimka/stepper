//
// Created by p0bedimka on 27.07.2026.
//

/**
 * @file stepper.h
 * @brief Высокоуровневый модуль управления шаговым двигателем.
 *
 * Объединяет драйвер (аппаратную часть) и планировщик (алгоритм изменения скорости)
 * в единый объект Stepper_t. Предоставляет пользователю простой API:
 *   - установка целевой скорости;
 *   - плавный разгон/торможение по выбранному закону;
 *   - аварийная остановка;
 *   - включение/отключение питания.
 */
#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include "stepper_driver.h"
#include "stepper_planner.h"

#include "sigmoid.h"
#include "s-curve.h"
#include "exponential.h"
#include "linear.h"

/**
 * @brief Структура состояния шагового двигателя.
 *
 * Содержит в себе драйвер, планировщик и текущее состояние (скорости, время).
 */
typedef struct {
    StepperDriver_t  dr;       ///< Экземпляр драйвера (копируется при инициализации)
    StepperPlanner_t pl;       ///< Экземпляр планировщика (копируется при инициализации)

    volatile int32_t tar_vel;  ///< Целевая скорость, заданная пользователем (шагов/с)
    volatile int32_t cur_vel;  ///< Текущая фактическая скорость (шагов/с)
    volatile uint32_t time;    ///< Время начала текущего переходного процесса (мс)
} Stepper_t;

/**
 * @brief Интерфейс (таблица виртуальных функций) модуля Stepper.
 */
typedef struct {
    void     (*Init)(Stepper_t *motor, StepperDriver_t *dr, StepperPlanner_t *pl);
    void     (*Enable)(Stepper_t *motor);
    void     (*Disable)(Stepper_t *motor);
    uint32_t (*GetAcc)(Stepper_t *motor);
    void     (*SetAcc)(Stepper_t *motor, uint32_t acc);
    int32_t  (*GetVel)(Stepper_t *motor);
    void     (*SetVel)(Stepper_t *motor, int32_t vel);
    void     (*Stop)(Stepper_t *motor);
    void     (*TaskPlanner)(Stepper_t *motor);
    void     (*TIMx_IRQHandler)(Stepper_t *motor);
} Stepper_Interface;

/// Экземпляр интерфейса для модуля Stepper
extern const Stepper_Interface Stepper_Itf;

#endif // STEPPER_H
