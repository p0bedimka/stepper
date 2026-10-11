//
// Created by Pobed on 27.07.2026.
//

#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>

#include "motor_driver.h"
#include "planner.h"

#include "sigmoid.h"
#include "s-curve.h"
#include "exponential.h"
#include "linear.h"

/**
 * @brief Структура управления шаговым двигателем
 *
 * Объединяет драйвер нижнего уровня и планировщик движения.
 * Содержит информацию о текущем и целевом состоянии двигателя.
 */
typedef struct {
    MotorDriver_t dr;        /**< Экземпляр драйвера двигателя (таймер + GPIO) */
    Planner_t pl;            /**< Экземпляр планировщика движения (расчёт ускорений) */

    volatile int32_t tar_vel; /**< Целевая скорость (импульсов/сек). Знак определяет направление */
    volatile int32_t cur_vel; /**< Текущая фактическая скорость (импульсов/сек) */
    volatile uint32_t time;   /**< Метка времени последнего вызова планировщика (мкс) */
} Stepper_t;

/**
 * @brief Интерфейс управления шаговым двигателем
 *
 * Набор функций-членов для работы с двигателем.
 * Реализован как структура с указателями на функции для удобства инкапсуляции.
 */
typedef struct {
    /**
     * @brief Инициализация двигателя
     * @param motor Указатель на структуру двигателя
     */
    void (*Init)(Stepper_t *motor, MotorDriver_t dr, Planner_t pl);

    /**
     * @brief Включение двигателя (разрешение ШИМ)
     * @param motor Указатель на структуру двигателя
     */
    void (*Enable)(Stepper_t *motor);

    /**
     * @brief Отключение двигателя (только при нулевой скорости)
     * @param motor Указатель на структуру двигателя
     */
    void (*Disable)(Stepper_t *motor);

    /**
     * @brief Получение текущего ускорения
     * @param motor Указатель на структуру двигателя
     * @return Ускорение (импульсов/сек²)
     */
    uint32_t (*GetAcc)(Stepper_t *motor);

    /**
     * @brief Установка ускорения
     * @param motor Указатель на структуру двигателя
     * @param acc Ускорение (импульсов/сек²)
     */
    void (*SetAcc)(Stepper_t *motor, uint32_t acc);

    /**
     * @brief Получение текущей скорости
     * @param motor Указатель на структуру двигателя
     * @return Текущая скорость (импульсов/сек). Отрицательное - обратное направление
     */
    int32_t (*GetVel)(Stepper_t *motor);

    /**
     * @brief Установка целевой скорости (запуск движения)
     * @param motor Указатель на структуру двигателя
     * @param vel Целевая скорость (импульсов/сек)
     */
    void (*SetVel)(Stepper_t *motor, int32_t vel);

    /**
     * @brief Экстренная остановка двигателя
     * @param motor Указатель на структуру двигателя
     */
    void (*Stop)(Stepper_t *motor);

    /**
     * @brief Циклическая задача планировщика (вызывается в основном цикле)
     *
     * Расчитывает необходимую скорость на основе времени и текущего состояния.
     * Должна вызываться с частотой не реже чем требуется для обновления скорости.
     *
     * @param motor Указатель на структуру двигателя
     */
    void (*TaskPlanner)(Stepper_t *motor);

    /**
     * @brief Обработчик прерывания таймера
     *
     * Вызывается из ISR таймера для генерации импульсов шагового двигателя.
     *
     * @param motor Указатель на структуру двигателя
     */
    void (*TIMx_IRQHandler)(Stepper_t *motor);
} Stepper_Interface;

/** @brief Глобальный экземпляр интерфейса шагового двигателя */
extern const Stepper_Interface Stepper_Itf;

#endif
