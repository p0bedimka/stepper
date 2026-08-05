//
// Created by Pobed on 01.08.2026.
//

#ifndef STEPPER_STM32F4_DEF_H
#define STEPPER_STM32F4_DEF_H

/**
 * @file motor_driver_def_stm32.h
 * @brief Определения констант для драйвера шагового двигателя на STM32
 */

/** @brief Номер канала таймера 1 */
#define TIM_CHAN_1      0x01
/** @brief Номер канала таймера 2 */
#define TIM_CHAN_2      0x02
/** @brief Номер канала таймера 3 */
#define TIM_CHAN_3      0x03
/** @brief Номер канала таймера 4 */
#define TIM_CHAN_4      0x04

/** @brief Разрядность таймера: 16 бит */
#define TIM_16_BIT      16
/** @brief Разрядность таймера: 32 бита */
#define TIM_32_BIT      32

/** @brief Минимальная частота тактирования APB (кГц) */
#define MIN_CLK_APB     20000

#endif
