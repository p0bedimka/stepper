//
// Created by p0bedimka on 27.07.2026.
//

/**
 * @file stepper.c
 * @brief Реализация модуля управления шаговым двигателем.
 *
 * Основная периодическая функция — Stepper_TaskPlanner(). Она должна вызываться
 * в главном цикле (или по таймеру) с периодом, меньшим, чем время переходного
 * процесса планировщика. Функция:
 *   1. Вычисляет прошедшее время с начала перехода.
 *   2. Если переход ещё идёт — запрашивает у планировщика текущую скорость
 *      и передаёт её драйверу.
 *   3. Если переход завершился, но целевая скорость не достигнута —
 *      запускает новый переходный процесс.
 */
#include "stepper.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief Инициализация шагового двигателя.
 *
 * Копирует переданные экземпляры драйвера и планировщика в структуру мотора
 * и вызывает их функции инициализации. Сбрасывает скорости и фиксирует текущее время.
 *
 * @param motor Указатель на структуру мотора.
 * @param dr    Указатель на инициализированный драйвер.
 * @param pl    Указатель на инициализированный планировщик.
 */
void Stepper_Init(Stepper_t *motor, StepperDriver_t *dr, StepperPlanner_t *pl) {
    if (motor == NULL) return;

    motor->dr = *dr;
    StepperDriver_Init(&motor->dr);

    motor->pl = *pl;
    StepperPlanner_Init(&motor->pl);

    motor->tar_vel = 0;
    motor->cur_vel = 0;
    motor->time = StepperDriver_GetTick(&motor->dr);
}

/** @brief Включение питания драйвера. */
void Stepper_Enable(Stepper_t *motor) {
    if (motor != NULL)
        StepperDriver_Enable(&motor->dr);
}

/**
 * @brief Отключение питания драйвера.
 *
 * Отключение выполняется ТОЛЬКО если двигатель полностью остановлен
 * (cur_vel == 0), чтобы избежать потери шагов или резкой остановки под нагрузкой.
 */
void Stepper_Disable(Stepper_t *motor) {
    if (motor == NULL) return;
    if (motor->cur_vel == 0)
        StepperDriver_Disable(&motor->dr);
}

/** @brief Получение текущего значения ускорения из планировщика. */
uint32_t Stepper_GetAcc(Stepper_t *motor) {
    if (motor == NULL) return 0;
    return StepperPlanner_GetAcc(&motor->pl);
}

/** @brief Установка нового значения ускорения. */
void Stepper_SetAcc(Stepper_t *motor, uint32_t acc) {
    if (motor != NULL)
        StepperPlanner_SetAcc(&motor->pl, acc);
}

/** @brief Получение текущей фактической скорости. */
int32_t Stepper_GetVel(Stepper_t *motor) {
    if (motor == NULL) return 0;
    return motor->cur_vel;
}

/**
 * @brief Установка целевой скорости.
 *
 * Значение принимается только если драйвер способен обеспечить такую скорость
 * (проверка через StepperDriver_CanSetVel). Реальный переход начнётся
 * при следующем вызове Stepper_TaskPlanner().
 */
void Stepper_SetVel(Stepper_t *motor, int32_t vel) {
    if (motor == NULL) return;
    if (StepperDriver_CanSetVel(&motor->dr, vel))
        motor->tar_vel = vel;
}

/**
 * @brief Аварийная остановка двигателя.
 *
 * Немедленно обнуляет целевую и текущую скорости, передаёт 0 в драйвер.
 * Переходный процесс не используется — двигатель останавливается мгновенно.
 */
void Stepper_Stop(Stepper_t *motor) {
    if (motor != NULL) {
        motor->tar_vel = 0;
        motor->cur_vel = 0;
        StepperDriver_SetVel(&motor->dr, 0);
    }
}

/**
 * @brief Периодическая задача планировщика.
 *
 * Должна вызываться достаточно часто (рекомендуется ≥ 1 кГц).
 *
 * Логика работы:
 *   1. Вычисляется прошедшее время elapsed с момента начала текущего перехода.
 *      Учитывается возможное переполнение 32-битного счётчика тиков.
 *   2. Если elapsed < timeout (переход ещё идёт):
 *        - запрашивается текущая скорость у планировщика;
 *        - если она изменилась — передаётся в драйвер.
 *   3. Если elapsed >= timeout (переход завершился), но целевая скорость
 *      не достигнута:
 *        - если требуется смена направления (знаки cur_vel и tar_vel различны,
 *          и оба не равны 0), сначала выполняется переход к нулю;
 *        - иначе запускается переход от cur_vel к tar_vel.
 */
void Stepper_TaskPlanner(Stepper_t *motor) {
    if (motor == NULL) return;

    uint32_t timeout = StepperPlanner_GetTimeAcc(&motor->pl);
    uint32_t elapsed, time = StepperDriver_GetTick(&motor->dr);

    // Обработка переполнения 32-битного счётчика тиков
    if (time >= motor->time)
        elapsed = time - motor->time;
    else
        elapsed = (UINT32_MAX - motor->time) + time + 1;

    if (timeout >= elapsed) {
        // Переходный процесс ещё идёт — получаем текущую скорость
        int32_t vel = StepperPlanner_GetVel(&motor->pl, (float)elapsed);
        if (vel != motor->cur_vel) {
            StepperDriver_SetVel(&motor->dr, vel);
            motor->cur_vel = vel;
        }
    }
    else if (motor->tar_vel != motor->cur_vel) {
        // Переход завершился, но цель не достигнута — запускаем новый переход

        // Если нужна смена направления — сначала тормозим до нуля
        if ((motor->tar_vel ^ motor->cur_vel) < 0 &&
            motor->cur_vel != 0 && motor->tar_vel != 0) {
            StepperPlanner_SetVel(&motor->pl, motor->cur_vel, 0);
        }
        else {
            StepperPlanner_SetVel(&motor->pl, motor->cur_vel, motor->tar_vel);
        }
        motor->time = StepperDriver_GetTick(&motor->dr);
    }
}

/** @brief Прокси-вызов обработчика прерывания таймера драйвера. */
void Stepper_TIMx_IRQHandler(Stepper_t *motor) {
    if (motor != NULL)
        StepperDriver_TIMx_IRQHandler(&motor->dr);
}

/// Таблица функций модуля Stepper
const Stepper_Interface Stepper_Itf = {
    .Init            = Stepper_Init,
    .Enable          = Stepper_Enable,
    .Disable         = Stepper_Disable,
    .GetAcc          = Stepper_GetAcc,
    .SetAcc          = Stepper_SetAcc,
    .GetVel          = Stepper_GetVel,
    .SetVel          = Stepper_SetVel,
    .Stop            = Stepper_Stop,
    .TaskPlanner     = Stepper_TaskPlanner,
    .TIMx_IRQHandler = Stepper_TIMx_IRQHandler,
};
