//
// Created by Pobed on 27.07.2026.
//
#include "step_motor.h"

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
void StepMotor_Init(StepMotor_t *motor) {
    if (!motor)
        return;

    /* Инициализация драйвера (таймеры, GPIO) */
    MotorDriver_Itf.Init(&motor->dr);

    /* Инициализация планировщика (параметры движения) */
    Planner_Itf.Init(&motor->pl);

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
void StepMotor_Enable(StepMotor_t *motor) {
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
void StepMotor_Disable(StepMotor_t *motor) {
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
uint32_t StepMotor_GetAcc(StepMotor_t *motor) {
    if (motor == NULL)
        return 0;

    return Planner_Itf.GetAcc(&motor->pl);
}

/**
 * @brief Установка ускорения в планировщик
 * @param motor Указатель на структуру двигателя
 * @param acc Ускорение (импульсов/сек²)
 */
void StepMotor_SetAcc(StepMotor_t *motor, uint32_t acc) {
    if (motor != NULL)
        Planner_Itf.SetAcc(&motor->pl, acc);
}

/**
 * @brief Получение текущей фактической скорости
 * @param motor Указатель на структуру двигателя
 * @return Текущая скорость (импульсов/сек)
 */
int32_t StepMotor_GetVel(StepMotor_t *motor) {
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
void StepMotor_SetVel(StepMotor_t *motor, int32_t vel) {
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
void StepMotor_Stop(StepMotor_t *motor) {
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
void StepMotor_TaskPlanner(StepMotor_t *motor) {
    if (motor == NULL)
        return;

    /* Получаем время обновления планировщика */
    uint32_t timeout = Planner_Itf.GetTimeAcc(&motor->pl);

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
        int32_t vel = Planner_Itf.GetVel(&motor->pl, (float)elapsed);
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
            Planner_Itf.SetVel(&motor->pl, motor->cur_vel, 0);
        else
            /* Плавное изменение от текущей к целевой скорости */
            Planner_Itf.SetVel(&motor->pl, motor->cur_vel, motor->tar_vel);

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
void StepMotor_TIMx_IRQHandler(StepMotor_t *motor) {
    if (motor != NULL)
        MotorDriver_Itf.TIMx_IRQHandler(&motor->dr);
}

/** @brief Глобальный экземпляр интерфейса шагового двигателя */
const StepMotor_Interface StepMotor_Itf = {
    .Init = StepMotor_Init,
    .Enable = StepMotor_Enable,
    .Disable = StepMotor_Disable,
    .GetAcc = StepMotor_GetAcc,
    .SetAcc = StepMotor_SetAcc,
    .GetVel = StepMotor_GetVel,
    .SetVel = StepMotor_SetVel,
    .Stop = StepMotor_Stop,
    .TaskPlanner = StepMotor_TaskPlanner,
    .TIMx_IRQHandler = StepMotor_TIMx_IRQHandler,
};