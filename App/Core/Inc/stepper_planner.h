//
// Created by p0bedimka on 27.07.2026.
//

/**
 * @file stepper_planner.h
 * @brief Абстрактный интерфейс для планировщиков скорости шагового двигателя.
 *
 * Предоставляет базовую структуру и таблицу функций (интерфейс),
 * которую должны реализовывать конкретные алгоритмы профилирования скорости
 * (например, сигмоида, S-образная кривая, экспонента и линейный разгон).
 */
#ifndef STEPPER_PLANNER_H
#define STEPPER_PLANNER_H

#include <stdint.h>

/// Прямое объявление базовой структуры планировщика
typedef struct StepperPlanner_t StepperPlanner_t;

/**
 * @brief Интерфейс (таблица виртуальных функций) планировщика.
 *
 * Содержит указатели на функции, реализующие логику конкретного
 * алгоритма планирования скорости.
 */
typedef struct {
    void (*Init)(StepperPlanner_t *pl);                                     ///< Инициализация планировщика
    uint32_t (*GetAcc)(StepperPlanner_t *p);                                ///< Получение текущего ускорения
    void (*SetAcc)(StepperPlanner_t *pl, uint32_t a);                       ///< Установка ускорения
    int32_t (*GetInitVel)(StepperPlanner_t *pl);                            ///< Получение начальной скорости
    int32_t (*GetTargVel)(StepperPlanner_t *pl);                            ///< Получение целевой скорости
    void (*SetVel)(StepperPlanner_t *pl, int32_t v0, int32_t v1);           ///< Установка начальной и целевой скоростей (пересчет параметров)
    int32_t (*GetVel)(StepperPlanner_t *pl, float dt);                      ///< Вычисление текущей скорости в момент времени dt
    uint32_t (*GetTimeAcc)(StepperPlanner_t *pl);                           ///< Получение общего времени разгона/торможения
} StepperPlanner_Interface;

/**
 * @brief Базовая структура планировщика.
 *
 * Должна быть ПЕРВЫМ полем в любых производных структурах конкретных
 * планировщиков (например, Sigmoid_t, Scurve_t, Exponential_t, Linear_t) для обеспечения
 * полиморфизма через приведение указателей.
 */
struct StepperPlanner_t {
    const StepperPlanner_Interface *itf; ///< Указатель на таблицу функций конкретной реализации
};

// --- Inline-обертки для безопасного вызова функций интерфейса ---

/** @brief Инициализация планировщика. */
static inline void StepperPlanner_Init(StepperPlanner_t *pl) {
    if (pl && pl->itf) pl->itf->Init(pl);
}

/** @brief Получение значения ускорения. */
static inline uint32_t StepperPlanner_GetAcc(StepperPlanner_t *pl) {
    if (pl && pl->itf) return pl->itf->GetAcc(pl);
    return 0;
}

/** @brief Установка значения ускорения. */
static inline void StepperPlanner_SetAcc(StepperPlanner_t *pl, uint32_t a) {
    if (pl && pl->itf) pl->itf->SetAcc(pl, a);
}

/** @brief Получение начальной скорости. */
static inline int32_t StepperPlanner_GetInitVel(StepperPlanner_t *pl) {
    if (pl && pl->itf) return pl->itf->GetInitVel(pl);
    return 0;
}

/** @brief Получение целевой скорости. */
static inline int32_t StepperPlanner_GetTargVel(StepperPlanner_t *pl) {
    if (pl && pl->itf) return pl->itf->GetTargVel(pl);
    return 0;
}

/** @brief Установка начальной и целевой скоростей. */
static inline void StepperPlanner_SetVel(StepperPlanner_t *pl, int32_t v0, int32_t v1) {
    if (pl && pl->itf) pl->itf->SetVel(pl, v0, v1);
}

/** @brief Вычисление текущей скорости в заданный момент времени. */
static inline int32_t StepperPlanner_GetVel(StepperPlanner_t *pl, float dt) {
    if (pl && pl->itf) return pl->itf->GetVel(pl, dt);
    return 0;
}

/** @brief Получение общего времени, необходимого для разгона/торможения. */
static inline uint32_t StepperPlanner_GetTimeAcc(StepperPlanner_t *pl) {
    if (pl && pl->itf) return pl->itf->GetTimeAcc(pl);
    return 0;
}

#endif // STEPPER_PLANNER_H
