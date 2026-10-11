//
// Created by Pobed on 27.07.2026.
//

#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/** @brief Тип драйвера двигателя (алиас для платформо-зависимой структуры) */
typedef struct StepperDriver_t StepperDriver_t;

/**
 * @brief Интерфейс драйвера шагового двигателя
 *
 * Предоставляет низкоуровневые функции для управления таймером и выводами
 * генерации импульсов шагового двигателя.
 */
typedef struct {
    /**
     * @brief Инициализация драйвера
     * @param stp Указатель на структуру драйвера
     *
     * Настраивает таймер в режим ШИМ, конфигурирует GPIO для направления и разрешения.
     * Проверяет корректность параметров конфигурации.
     */
    void (*Init)(StepperDriver_t *stp);

    /**
     * @brief Включение драйвера
     * @param stp Указатель на структуру драйвера
     *
     * Запускает таймер, включает выход сравнения, активирует вывод разрешения.
     */
    void (*Enable)(StepperDriver_t *stp);

    /**
     * @brief Отключение драйвера
     * @param stp Указатель на структуру драйвера
     *
     * Останавливает таймер, отключает выход сравнения, деактивирует вывод разрешения.
     */
    void (*Disable)(StepperDriver_t *stp);

    /**
     * @brief Проверка возможности установки скорости
     * @param stp Указатель на структуру драйвера
     * @param vel Проверяемая скорость (импульсов/сек)
     * @return true - скорость достижима, false - скорость слишком высокая
     *
     * Проверяет, может ли таймер сгенерировать требуемую частоту импульсов
     * с учётом разрядности таймера и тактовой частоты.
     */
    bool (*CanSetVel)(StepperDriver_t *stp, int32_t vel);

    /**
     * @brief Установка скорости вращения
     * @param stp Указатель на структуру драйвера
     * @param vel Скорость (импульсов/сек). Отрицательная - обратное направление
     * @return true - скорость установлена, false - ошибка
     *
     * Устанавливает период и скважность ШИМ, а также направление вращения.
     * Обновление выполняется с защитой от прерываний.
     */
    bool (*SetVel)(StepperDriver_t *stp, int32_t vel);

    /**
     * @brief Обработчик прерывания таймера
     * @param stp Указатель на структуру драйвера
     *
     * Обновляет состояние вывода направления в момент прерывания
     * для синхронизации с импульсами ШИМ.
     */
    void (*TIMx_IRQHandler)(StepperDriver_t *stp);

    /**
     * @brief
     */
    uint32_t (*GetTick)(StepperDriver_t *stp);
} StepperDriver_Interface;

struct StepperDriver_t {
    const StepperDriver_Interface *itf;
};

static inline void StepperDriver_Init(StepperDriver_t *dr) {
    if (dr && dr->itf) dr->itf->Init(dr);
}

static inline void StepperDriver_Enable(StepperDriver_t *dr) {
    if (dr && dr->itf) dr->itf->Enable(dr);
}

static inline void StepperDriver_Disable(StepperDriver_t *dr) {
    if (dr && dr->itf) dr->itf->Disable(dr);
}

static inline bool StepperDriver_CanSetVel(StepperDriver_t *dr, int32_t vel) {
    if (dr && dr->itf)
        return dr->itf->CanSetVel(dr, vel);
    return false;
}

static inline bool StepperDriver_SetVel(StepperDriver_t *dr, int32_t vel) {
    if (dr && dr->itf)
        return dr->itf->SetVel(dr, vel);
    return false;
}

static inline void StepperDriver_TIMx_IRQHandler(StepperDriver_t *std_dr) {
    if (std_dr && std_dr->itf) std_dr->itf->TIMx_IRQHandler(std_dr);
}

static inline uint32_t StepperDriver_GetTick(StepperDriver_t *dr) {
    if (dr && dr->itf)
        return dr->itf->GetTick(dr);
    return 0;
}

#endif
