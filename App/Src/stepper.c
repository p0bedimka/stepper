//
// Created by Pobed on 27.07.2026.
//
#include "stepper.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief Инициализация структуры шагового двигателя
 * @param motor Указатель на структуру двигателя
 *
 * Выполняет начальную конфигурацию драйвера и планировщика.
 * Устанавливает начальные значения скоростей и времени.
 */
void Stepper_Init(Stepper_t *motor, MotorDriver_t dr, Planner_t pl) {
    if (motor == NULL) return;

    /* Инициализация драйвера (таймеры, GPIO) */
    motor->dr = dr;
    MotorDriver_Itf.Init(&motor->dr);

    /* Инициализация планировщика (параметры движения) */
    motor->pl = pl;
    Planner_Init(&motor->pl);

    /* Сброс параметров движения */
    motor->tar_vel = 0;
    motor->cur_vel = 0;

    /* Сохранение текущего времени для расчёта интервалов */
    motor->time = motor->dr.GetTick();
}

/**
 * @brief Включение двигателя
 * @param motor Указатель на структуру двигателя
 *
 * Активирует выход ШИМ на таймере и включает силовую часть.
 */
void Stepper_Enable(Stepper_t *motor) {
    if (motor != NULL)
        MotorDriver_Itf.Enable(&motor->dr);
}

/**
 * @brief Отключение двигателя
 * @param motor Указатель на структуру двигателя
 *
 * Безопасное отключение - разрешено только при остановленном двигателе
 * (cur_vel == 0) для предотвращения потери шагов.
 */
void Stepper_Disable(Stepper_t *motor) {
    if (motor == NULL)
        return;

    /* Отключаем только если двигатель остановлен */
    if (motor->cur_vel == 0)
        MotorDriver_Itf.Disable(&motor->dr);
}

/**
 * @brief Получение значения ускорения из планировщика
 * @param motor Указатель на структуру двигателя
 * @return Ускорение (импульсов/сек²)
 */
uint32_t Stepper_GetAcc(Stepper_t *motor) {
    if (motor == NULL)
        return 0;

    return Planner_GetAcc(&motor->pl);
}

/**
 * @brief Установка ускорения в планировщик
 * @param motor Указатель на структуру двигателя
 * @param acc Ускорение (импульсов/сек²)
 */
void Stepper_SetAcc(Stepper_t *motor, uint32_t acc) {
    if (motor != NULL)
        Planner_SetAcc(&motor->pl, acc);
}

/**
 * @brief Получение текущей фактической скорости
 * @param motor Указатель на структуру двигателя
 * @return Текущая скорость (импульсов/сек)
 */
int32_t Stepper_GetVel(Stepper_t *motor) {
    if (motor == NULL)
        return 0;

    return motor->cur_vel;
}

/**
 * @brief Установка целевой скорости
 * @param motor Указатель на структуру двигателя
 * @param vel Целевая скорость (импульсов/сек)
 *
 * Проверяет возможность достижения скорости аппаратно,
 * затем сохраняет целевую скорость для планировщика.
 */
void Stepper_SetVel(Stepper_t *motor, int32_t vel) {
    if (motor == NULL)
        return;

    /* Проверка: может ли драйвер обеспечить данную скорость */
    if (MotorDriver_Itf.СanSetVel(&motor->dr, vel))
        motor->tar_vel = vel;
}

/**
 * @brief Экстренная остановка двигателя
 * @param motor Указатель на структуру двигателя
 *
 * Мгновенная остановка - обнуляет все скорости
 * и немедленно отключает генерацию импульсов.
 */
void Stepper_Stop(Stepper_t *motor) {
    if (motor != NULL) {
        motor->tar_vel = 0;
        motor->cur_vel = 0;
        MotorDriver_Itf.SetVel(&motor->dr, 0);
    }
}

/**
 * @brief Основная циклическая задача планировщика
 * @param motor Указатель на структуру двигателя
 *
 * Выполняет расчёт текущей скорости на основе времени,
 * прошедшего с последнего вызова, и текущего состояния.
 * Автоматически перепланирует траекторию при изменении скорости.
 *
 * @note Должна вызываться в основном цикле с частотой >= 1/таймаут_планировщика
 */
void Stepper_TaskPlanner(Stepper_t *motor) {
    if (motor == NULL)
        return;

    /* Получаем время обновления планировщика */
    uint32_t timeout = Planner_GetTimeAcc(&motor->pl);

    /* Расчёт времени, прошедшего с последнего обновления */
    uint32_t elapsed, time = motor->dr.GetTick();
    if (time >= motor->time)
        elapsed = time - motor->time;
    else
        /* Обработка переполнения 32-битного счётчика */
        elapsed = (UINT32_MAX - motor->time) + time + 1;

    /* Если планировщик активен (в пределах таймаута) */
    if (timeout >= elapsed) {
        /* Расчёт новой скорости по времени */
        int32_t vel = Planner_GetVel(&motor->pl, (float)elapsed);
        /* Обновление драйвера только при изменении скорости */
        if (vel != motor->cur_vel) {
            MotorDriver_Itf.SetVel(&motor->dr, vel);
            motor->cur_vel = vel;
        }
    }
    /* Если таймаут истёк, но скорость изменилась */
    else if (motor->tar_vel != motor->cur_vel) {
        /* При смене направления через ноль - перепланирование от нуля */
        if ((motor->tar_vel ^ motor->cur_vel) < 0 && motor->cur_vel != 0 && motor->tar_vel != 0)
            Planner_SetVel(&motor->pl, motor->cur_vel, 0);
        else
            /* Плавное изменение от текущей к целевой скорости */
            Planner_SetVel(&motor->pl, motor->cur_vel, motor->tar_vel);

        /* Обновление временной метки для нового цикла планирования */
        motor->time = motor->dr.GetTick();
    }
}

/**
 * @brief Обработчик прерывания таймера
 * @param motor Указатель на структуру двигателя
 *
 * Передаёт управление драйверу для обработки аппаратного прерывания.
 * Должна вызываться из ISR соответствующего таймера.
 */
void Stepper_TIMx_IRQHandler(Stepper_t *motor) {
    if (motor != NULL)
        MotorDriver_Itf.TIMx_IRQHandler(&motor->dr);
}

/** @brief Глобальный экземпляр интерфейса шагового двигателя */
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
