//
// Created by Pobed on 27.07.2026.
//

#ifndef STEPPER_DRIVER_STM32_H
#define STEPPER_DRIVER_STM32_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "stepper_driver.h"
#include "stepper_driver_def_stm32.h"

/**
 * @brief Структура конфигурации драйвера для STM32
 *
 * Содержит все необходимые настройки для управления таймером
 * и выводами шагового двигателя на микроконтроллерах STM32.
 */
typedef struct {
    StepperDriver_t base;

    /**
     * @brief Конфигурация таймера
     */
    struct {
        TIM_TypeDef *Instance;  /**< Указатель на регистры таймера (например, TIM1) */
        uint8_t Channel;        /**< Номер канала таймера (1-4) */
        uint8_t Bit;            /**< Разрядность таймера: TIM_16_BIT или TIM_32_BIT */
        uint32_t Clock;         /**< Частота тактирования таймера в Гц */
        IRQn_Type IRQn;         /**< Номер прерывания таймера (например, TIM1_UP_IRQn) */
    } htim;

    /**
     * @brief Конфигурация вывода направления
     */
    struct {
        GPIO_TypeDef *Port;     /**< Порт GPIO для управления направлением */
        uint16_t Pin;           /**< Номер пина (GPIO_Pin_x) */
        uint32_t State;         /**< Текущее состояние (BSRR значение для установки) */
    } dir;

    /**
     * @brief Конфигурация вывода разрешения (Enable)
     */
    struct {
        GPIO_TypeDef *Port;     /**< Порт GPIO для управления разрешением */
        uint16_t Pin;           /**< Номер пина (GPIO_Pin_x) */
    } en;

    /**
     * @brief Функция получения текущего времени в микросекундах
     * @return Текущее время (мкс)
     *
     * Должна возвращать монотонно возрастающее значение.
     * Используется планировщиком для расчёта интервалов.
     */
    uint32_t (*GetTick)(void);
} StepperDriverSTM32_t;

extern const StepperDriver_Interface StepperDriver_Itf;

#endif
