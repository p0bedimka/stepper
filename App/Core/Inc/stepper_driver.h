//
// Created by p0bedimka on 27.07.2026.
//

/**
 * @file stepper_driver.h
 * @brief Абстрактный интерфейс драйвера шагового двигателя.
 *
 * Определяет набор функций, которые должна реализовать конкретная
 * аппаратно-зависимая версия драйвера (например, для STM32, AVR, ESP32).
 * Драйвер отвечает за:
 *   - генерацию шаговых импульсов (обычно через PWM таймера);
 *   - управление направлением (DIR);
 *   - управление питанием (ENABLE);
 *   - обработку прерываний таймера.
 */
#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/// Прямое объявление базовой структуры драйвера
typedef struct StepperDriver_t StepperDriver_t;

/**
 * @brief Интерфейс (таблица виртуальных функций) драйвера.
 */
typedef struct {
    void     (*Init)(StepperDriver_t *stp);                       ///< Инициализация периферии
    void     (*Enable)(StepperDriver_t *stp);                     ///< Включение драйвера (подача питания)
    void     (*Disable)(StepperDriver_t *stp);                    ///< Отключение драйвера
    bool     (*CanSetVel)(StepperDriver_t *stp, int32_t vel);     ///< Проверка достижимости скорости
    bool     (*SetVel)(StepperDriver_t *stp, int32_t vel);        ///< Установка скорости (шагов/с)
    void     (*TIMx_IRQHandler)(StepperDriver_t *stp);            ///< Обработчик прерывания таймера
    uint32_t (*GetTick)(StepperDriver_t *stp);                    ///< Получение системного времени (мс)
} StepperDriver_Interface;

/**
 * @brief Базовая структура драйвера.
 *
 * Должна быть ПЕРВЫМ полем в производных структурах (например, StepperDriverSTM32_t)
 * для обеспечения полиморфизма через приведение указателей.
 */
struct StepperDriver_t {
    const StepperDriver_Interface *itf; ///< Указатель на таблицу функций конкретной реализации
};

// --- Inline-обертки для безопасного вызова функций интерфейса ---

/** @brief Инициализация драйвера. */
static inline void StepperDriver_Init(StepperDriver_t *dr) {
    if (dr && dr->itf) dr->itf->Init(dr);
}

/** @brief Включение драйвера. */
static inline void StepperDriver_Enable(StepperDriver_t *dr) {
    if (dr && dr->itf) dr->itf->Enable(dr);
}

/** @brief Отключение драйвера. */
static inline void StepperDriver_Disable(StepperDriver_t *dr) {
    if (dr && dr->itf) dr->itf->Disable(dr);
}

/**
 * @brief Проверка, может ли драйвер обеспечить заданную скорость.
 * @param dr  Указатель на драйвер.
 * @param vel Желаемая скорость (шагов/с).
 * @return true, если скорость достижима (не превышает возможности таймера).
 */
static inline bool StepperDriver_CanSetVel(StepperDriver_t *dr, int32_t vel) {
    if (dr && dr->itf) return dr->itf->CanSetVel(dr, vel);
    return false;
}

/**
 * @brief Установка новой скорости.
 * @param dr  Указатель на драйвер.
 * @param vel Скорость в шагах/с (знак определяет направление).
 * @return true в случае успеха.
 */
static inline bool StepperDriver_SetVel(StepperDriver_t *dr, int32_t vel) {
    if (dr && dr->itf) return dr->itf->SetVel(dr, vel);
    return false;
}

/** @brief Обработчик прерывания таймера (вызывается из ISR). */
static inline void StepperDriver_TIMx_IRQHandler(StepperDriver_t *dr) {
    if (dr && dr->itf) dr->itf->TIMx_IRQHandler(dr);
}

/**
 * @brief Получение текущего значения системного таймера (мс).
 * @note Функция GetTick привязана к драйверу, т.к. именно он знает источник времени.
 */
static inline uint32_t StepperDriver_GetTick(StepperDriver_t *dr) {
    if (dr && dr->itf) return dr->itf->GetTick(dr);
    return 0;
}

#endif // STEPPER_DRIVER_H
