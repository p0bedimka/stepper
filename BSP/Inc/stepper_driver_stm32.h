//
// Created by p0bedimka on 27.07.2026.
//

/**
 * @file stepper_driver_stm32.h
 * @brief Аппаратно-зависимый драйвер шагового двигателя для STM32.
 *
 * Реализация интерфейса StepperDriver_Interface на базе:
 *   - TIM (таймер в режиме PWM) — генерация шаговых импульсов;
 *   - GPIO (DIR)                — управление направлением;
 *   - GPIO (EN)                 — управление питанием драйвера.
 *
 * Принцип работы:
 *   Частота PWM = модуль скорости (шагов/с). Скважность 50% (pulse = (ARR+1)/2).
 *   Направление задаётся состоянием пина DIR, которое обновляется в обработчике
 *   прерывания по событию Update (чтобы избежать генерации ложных шагов при смене DIR).
 */
#ifndef STEPPER_DRIVER_STM32_H
#define STEPPER_DRIVER_STM32_H

#include <stdint.h>

#include "main.h"
#include "../../App/Core/Inc/stepper_driver.h"
#include "stepper_driver_def_stm32.h"

/**
 * @brief Структура состояния драйвера для STM32.
 */
typedef struct {
    StepperDriver_t base;     ///< Базовый интерфейс

    struct {
        TIM_TypeDef *Instance; ///< Указатель на регистры таймера (TIMx)
        uint8_t      Channel;  ///< Номер канала (TIM_CHAN_1..4)
        uint8_t      Bit;      ///< Разрядность таймера (TIM_16_BIT / TIM_32_BIT)
        uint32_t     Clock;    ///< Тактовая частота таймера (Гц)
        IRQn_Type    IRQn;     ///< Номер прерывания таймера
    } htim;

    struct {
        GPIO_TypeDef *Port;    ///< Порт пина DIR
        uint16_t      Pin;     ///< Пин DIR
        uint32_t      State;   ///< Текущее состояние (для атомарной записи через BSRR)
    } dir;

    struct {
        GPIO_TypeDef *Port;    ///< Порт пина EN
        uint16_t      Pin;     ///< Пин EN
    } en;

    uint32_t (*GetTick)(void); ///< Функция получения системного времени (например, HAL_GetTick)
} StepperDriverSTM32_t;

/// Экземпляр интерфейса драйвера для STM32
extern const StepperDriver_Interface StepperDriver_Itf;

#endif // STEPPER_DRIVER_STM32_H
