//
// Created by Pobed on 27.07.2026.
//
#include "motor_driver_stm32.h"

#include <stdlib.h>

/* ============ Макросы для работы с таймером ============ */

/** @brief Установка значения регистра сравнения для выбранного канала */
#define TIM_SET_COMPARE(__HANDLE__, __COMPARE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCR1 = (__COMPARE__)) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCR2 = (__COMPARE__)) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCR3 = (__COMPARE__)) : \
     ((__HANDLE__)->Instance->CCR4 = (__COMPARE__)))

/** @brief Включение предварительной загрузки для выбранного канала */
#define TIM_ENABLE_OCxPRELOAD(__HANDLE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCMR1 |= TIM_CCMR1_OC1PE) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCMR1 |= TIM_CCMR1_OC2PE) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCMR2 |= TIM_CCMR2_OC3PE) : \
     ((__HANDLE__)->Instance->CCMR2 |= TIM_CCMR2_OC4PE))

/** @brief Установка режима ШИМ для выбранного канала (PWM Mode 2) */
#define TIM_SET_MODE_PWM(__HANDLE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCMR1 = ((__HANDLE__)->Instance->CCMR1 & ~TIM_CCMR1_OC1M) | (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCMR1 = ((__HANDLE__)->Instance->CCMR1 & ~TIM_CCMR2_OC3M) | (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2)) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCMR2 = ((__HANDLE__)->Instance->CCMR2 & ~TIM_CCMR2_OC3M) | (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2)) : \
     ((__HANDLE__)->Instance->CCMR2 = ((__HANDLE__)->Instance->CCMR2 & ~TIM_CCMR2_OC4M) | (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2)))

/** @brief Включение выхода сравнения для выбранного канала */
#define TIM_ENABLE_COMPARE(__HANDLE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCER |= TIM_CCER_CC1E) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCER |= TIM_CCER_CC2E) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCER |= TIM_CCER_CC3E) : \
     ((__HANDLE__)->Instance->CCER |= TIM_CCER_CC4E))

/** @brief Отключение выхода сравнения для выбранного канала */
#define TIM_DISABLE_COMPARE(__HANDLE__) \
    (((__HANDLE__)->Channel == TIM_CHAN_1) ? ((__HANDLE__)->Instance->CCER &= ~TIM_CCER_CC1E) : \
     ((__HANDLE__)->Channel == TIM_CHAN_2) ? ((__HANDLE__)->Instance->CCER &= ~TIM_CCER_CC2E) : \
     ((__HANDLE__)->Channel == TIM_CHAN_3) ? ((__HANDLE__)->Instance->CCER &= ~TIM_CCER_CC3E) : \
     ((__HANDLE__)->Instance->CCER &= ~TIM_CCER_CC4E))

/* ============ Статические функции ============ */

/**
 * @brief Расчёт периода таймера для заданной частоты
 * @param stp Указатель на структуру драйвера
 * @param freq Требуемая частота (Гц)
 * @return Значение регистра автоперезагрузки (ARR)
 *
 * Формула: ARR = (Clock / (freq * (PSC + 1))) - 1
 * При PSC = 0: ARR = Clock / freq - 1
 */
static uint32_t Stepper_CalcPeriod(const MotorDriver_Type *stp, uint32_t freq) {
    if (freq == 0) return 0;
    return stp->htim.Clock / (freq * (stp->htim.Instance->PSC + 1)) - 1;
}

/* ============ Публичные функции ============ */

/**
 * @brief Инициализация драйвера
 * @param dr Указатель на структуру драйвера
 *
 * Выполняет валидацию всех параметров конфигурации.
 * Настраивает таймер в режим ШИМ с периодом 1000 Гц по умолчанию.
 * Включает прерывания по обновлению таймера.
 */
void MotorDriver_Init(MotorDriver_Type *dr) {
    if (dr == NULL)
        return;

    if (dr->htim.Instance == NULL)
        return;

    if (!IS_NVIC_DEVICE_IRQ(dr->htim.IRQn) || dr->htim.Clock < 2 * MIN_CLK_APB) {
        return;
    }

    if (dr->dir.Port == NULL && dr->en.Port == NULL)
        return;

    if (dr->GetTick == NULL)
        return;

    /* Установка разрядности по умолчанию (16 бит) если указано неверное значение */
    if (dr->htim.Bit != TIM_16_BIT || dr->htim.Bit != TIM_32_BIT)
        dr->htim.Bit = TIM_16_BIT;

    /* Остановка таймера на время конфигурации */
    dr->htim.Instance->CR1 &= ~TIM_CR1_CEN;

    /* Настройка периода ~1000 Гц */
    dr->htim.Instance->PSC = 0;
    dr->htim.Instance->ARR = Stepper_CalcPeriod(dr, 1000);

    /* Включение автоматической предварительной загрузки */
    dr->htim.Instance->CR1 |= TIM_CR1_ARPE;

    /* Настройка режима ШИМ */
    TIM_SET_MODE_PWM(&dr->htim);
    TIM_ENABLE_OCxPRELOAD(&dr->htim);

    /* Начальное значение скважности = 0 (нет импульсов) */
    TIM_SET_COMPARE(&dr->htim, 0);

    /* Разрешение прерывания по обновлению */
    dr->htim.Instance->DIER |= TIM_DIER_UIE;
    dr->htim.Instance->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Включение драйвера
 * @param dr Указатель на структуру драйвера
 *
 * Запускает таймер, активирует выход сравнения,
 * включает вывод разрешения и разрешает прерывания.
 */
void MotorDriver_Enable(MotorDriver_Type *dr) {
    if (dr == NULL || dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(dr->htim.IRQn))
        return;

    dr->htim.Instance->CR1 |= TIM_CR1_CEN;      /* Запуск таймера */
    TIM_ENABLE_COMPARE(&dr->htim);              /* Включение выхода ШИМ */
    dr->en.Port->BSRR = dr->en.Pin;             /* Активация Enable (HIGH) */
    NVIC_EnableIRQ(dr->htim.IRQn);              /* Разрешение прерываний */
}

/**
 * @brief Отключение драйвера
 * @param dr Указатель на структуру драйвера
 *
 * Останавливает таймер, отключает выход сравнения,
 * деактивирует вывод разрешения и запрещает прерывания.
 */
void MotorDriver_Disable(MotorDriver_Type *dr) {
    if (dr == NULL || dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(dr->htim.IRQn))
        return;

    dr->htim.Instance->CR1 &= ~TIM_CR1_CEN;     /* Остановка таймера */
    TIM_DISABLE_COMPARE(&dr->htim);             /* Отключение выхода ШИМ */
    dr->en.Port->BSRR = dr->en.Pin << 16;       /* Деактивация Enable (LOW) */
    NVIC_DisableIRQ(dr->htim.IRQn);             /* Запрет прерываний */
}

/**
 * @brief Проверка достижимости частоты
 * @param dr Указатель на структуру драйвера
 * @param vel Проверяемая скорость (имп/сек)
 * @return true - достижима, false - превышает возможности таймера
 *
 * Расчитывает период и проверяет, не превышает ли он максимальное значение
 * для данной разрядности таймера.
 */
bool MotorDriver_СanSetVel(MotorDriver_Type *dr, int32_t vel) {
    if (dr == NULL || dr->htim.Instance == NULL)
        return false;

    if (vel == 0)
        return true;

    uint32_t period = Stepper_CalcPeriod(dr, abs(vel));
    return period <= ((1ULL << dr->htim.Bit) - 1);
}

/**
 * @brief Установка скорости вращения
 * @param dr Указатель на структуру драйвера
 * @param vel Скорость (имп/сек). Отрицательная - обратное направление.
 * @return true - успешно, false - ошибка
 *
 * @note Выполняется с отключением прерываний для атомарного обновления.
 */
bool MotorDriver_SetVel(MotorDriver_Type *dr, int32_t vel) {
    if (dr == NULL)
        return false;

    if (dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(dr->htim.IRQn))
        return false;

    if (dr->dir.Port == NULL)
        return false;

    uint32_t period = 0, pulse = 0;

    /* Расчёт параметров для ненулевой скорости */
    if (vel != 0) {
        period = Stepper_CalcPeriod(dr, abs(vel));
        if (period <= ((1ULL << dr->htim.Bit) - 1))
            pulse = (period + 1) / 2;  /* Скважность 50% */
        else
            return false;
    }

    /* Атомарное обновление с защитой от прерываний */
    NVIC_DisableIRQ(dr->htim.IRQn);

    if (vel != 0)
        dr->htim.Instance->ARR = period;

    TIM_SET_COMPARE(&dr->htim, pulse);

    /* Установка направления: положительная скорость - один уровень,
     * отрицательная - противоположный */
    if (vel > 0)
        dr->dir.State = dr->dir.Pin;
    else
        dr->dir.State = dr->dir.Pin << 16;

    NVIC_EnableIRQ(dr->htim.IRQn);

    return true;
}

/**
 * @brief Обработчик прерывания таймера
 * @param dr Указатель на структуру драйвера
 *
 * При срабатывании прерывания по обновлению таймера
 * устанавливает состояние вывода направления.
 * Это обеспечивает синхронизацию смены направления с импульсами ШИМ.
 */
void MotorDriver_TIMx_IRQHandler(MotorDriver_Type *dr) {
    if (dr == NULL || dr->htim.Instance == NULL)
        return;

    /* Проверка флага прерывания по обновлению */
    if (dr->htim.Instance->SR & TIM_SR_UIF) {
        dr->htim.Instance->SR &= ~TIM_SR_UIF;  /* Сброс флага */

        /* Обновление состояния направления в момент прерывания */
        if (dr->dir.Port != NULL)
            dr->dir.Port->BSRR = dr->dir.State;
    }
}