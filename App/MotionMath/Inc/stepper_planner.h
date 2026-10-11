//
// Created by Pobed on 27.07.2026.
//

#ifndef STEPPER_PLANNER_H
#define STEPPER_PLANNER_H

#include <stdint.h>

/**
 * @file stepper_planner.h
 * @brief Интерфейс планировщика движения
 *
 * Определяет универсальный интерфейс для планировщиков движения.
 * Планировщик отвечает за расчёт профиля скорости двигателя
 * в зависимости от времени и заданных параметров движения.
 *
 * Контракт интерфейса:
 * 1. Планировщик должен хранить начальную (v0) и целевую (v1) скорость
 * 2. Планировщик должен поддерживать установку ускорения (a)
 * 3. Планировщик должен рассчитывать скорость в любой момент времени (dt)
 * 4. Планировщик должен предоставлять время завершения переходного процесса
 *
 * @note Любая реализация планировщика должна соответствовать этому интерфейсу
 */

/**
 * @brief Абстрактный тип планировщика
 *
 * Указатель на конкретную структуру планировщика.
 * Позволяет использовать различные реализации через единый интерфейс.
 */
typedef struct StepperPlanner_t StepperPlanner_t;

/**
 * @brief Интерфейс планировщика движения
 *
 * Определяет набор функций для работы с планировщиком.
 * Все реализации должны предоставлять эти функции.
 */
typedef struct {
    /**
     * @brief Инициализация планировщика
     * @param pl Указатель на структуру планировщика
     *
     * Выполняет начальную настройку планировщика.
     * Должна проверять корректность параметров и устанавливать
     * значения по умолчанию при необходимости.
     */
    void (*Init)(StepperPlanner_t *pl);

    /**
     * @brief Получение текущего ускорения
     * @param p Указатель на структуру планировщика
     * @return Ускорение (импульсов/сек²)
     *
     * Возвращает значение ускорения, используемое планировщиком
     * для расчёта профиля скорости.
     */
    uint32_t (*GetAcc)(StepperPlanner_t *p);

    /**
     * @brief Установка ускорения
     * @param pl Указатель на структуру планировщика
     * @param a Ускорение (импульсов/сек²)
     *
     * Устанавливает ускорение для расчёта профиля движения.
     * Влияет на крутизну изменения скорости.
     */
    void (*SetAcc)(StepperPlanner_t *pl, uint32_t a);

    /**
     * @brief Получение начальной скорости
     * @param pl Указатель на структуру планировщика
     * @return Начальная скорость (импульсов/сек)
     *
     * Возвращает скорость, с которой начинается движение.
     */
    int32_t (*GetInitVel)(StepperPlanner_t *pl);

    /**
     * @brief Получение целевой скорости
     * @param pl Указатель на структуру планировщика
     * @return Целевая скорость (импульсов/сек)
     *
     * Возвращает скорость, к которой стремится движение.
     */
    int32_t (*GetTargVel)(StepperPlanner_t *pl);

    /**
     * @brief Установка параметров движения
     * @param pl Указатель на структуру планировщика
     * @param v0 Начальная скорость (импульсов/сек)
     * @param v1 Целевая скорость (импульсов/сек)
     *
     * Задаёт начальные условия для расчёта профиля движения.
     * Планировщик должен построить траекторию перехода от v0 к v1.
     *
     * @note Разные реализации могут по-разному интерпретировать
     * переход в зависимости от алгоритма планирования.
     */
    void (*SetVel)(StepperPlanner_t *pl, int32_t v0, int32_t v1);

    /**
     * @brief Расчёт скорости в заданный момент времени
     * @param pl Указатель на структуру планировщика
     * @param dt Время от начала движения (мс)
     * @return Текущая скорость (импульсов/сек)
     *
     * Возвращает расчётную скорость в момент времени dt.
     * Время dt должно отсчитываться от момента вызова SetVel.
     *
     * @note dt должно монотонно возрастать между вызовами
     * для корректного воспроизведения профиля.
     */
    int32_t (*GetVel)(StepperPlanner_t *pl, float dt);

    /**
     * @brief Получение времени переходного процесса
     * @param pl Указатель на структуру планировщика
     * @return Время (мс)
     *
     * Возвращает время, необходимое для завершения перехода
     * от начальной скорости к целевой.
     * Используется для определения момента, когда движение завершено.
     */
    uint32_t (*GetTimeAcc)(StepperPlanner_t *pl);
} StepperPlanner_Interface;

/**
 * @brief Базовая структура планировщика
 * Содержит только указатель на vtable.
 * Любая конкретная реализация должна встраивать её ПЕРВЫМ полем.
 */
struct StepperPlanner_t {
    const StepperPlanner_Interface *itf;
};

/* ---------- Удобные обёртки-функции ---------- */

static inline void StepperPlanner_Init(StepperPlanner_t *pl) {
    if (pl && pl->itf) pl->itf->Init(pl);
}

static inline uint32_t StepperPlanner_GetAcc(StepperPlanner_t *pl) {
    if (pl && pl->itf)
        return pl->itf->GetAcc(pl);
    return 0;
}

static inline void StepperPlanner_SetAcc(StepperPlanner_t *pl, uint32_t a) {
    if (pl && pl->itf) pl->itf->SetAcc(pl, a);
}

static inline int32_t StepperPlanner_GetInitVel(StepperPlanner_t *pl) {
    if (pl && pl->itf)
        return pl->itf->GetInitVel(pl);
    return 0;
}

static inline int32_t StepperPlanner_GetTargVel(StepperPlanner_t *pl) {
    if (pl && pl->itf)
        return pl->itf->GetTargVel(pl);
    return 0;
}

static inline void StepperPlanner_SetVel(StepperPlanner_t *pl, int32_t v0, int32_t v1) {
    if (pl && pl->itf) pl->itf->SetVel(pl, v0, v1);
}

static inline int32_t StepperPlanner_GetVel(StepperPlanner_t *pl, float dt) {
    if (pl && pl->itf)
        return pl->itf->GetVel(pl, dt);
    return 0;
}

static inline uint32_t StepperPlanner_GetTimeAcc(StepperPlanner_t *pl) {
    if (pl && pl->itf)
        return pl->itf->GetTimeAcc(pl);
    return 0;
}

#endif
