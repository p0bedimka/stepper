//
// Created by Pobed on 27.07.2026.
//
#include "planner.h"

/**
 * @brief Глобальный экземпляр интерфейса планировщика
 *
 * Связывает функции сигмоидального планировщика с унифицированным
 * интерфейсом для использования в модуле управления двигателем.
 *
 * Все функции являются обёртками над соответствующими функциями
 * Sigmoid_*, что обеспечивает абстракцию реализации.
 */
const Planner_Interface Planner_Itf = {
    .Init       = Sigmoid_Init,      /**< Инициализация планировщика */
    .GetAcc     = Sigmoid_GetAcc,    /**< Получение ускорения */
    .SetAcc     = Sigmoid_SetAcc,    /**< Установка ускорения */
    .GetInitVel = Sigmoid_GetInitVel,/**< Получение начальной скорости */
    .GetTargVel = Sigmoid_GetTargVel,/**< Получение целевой скорости */
    .SetVel     = Sigmoid_SetVel,    /**< Установка профиля скорости */
    .GetVel     = Sigmoid_GetVel,    /**< Расчёт скорости по времени */
    .GetTimeAcc = Sigmoid_GetTimeAcc /**< Получение времени разгона */
};
