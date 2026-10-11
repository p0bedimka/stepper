//
// Created by Pobed on 27.07.2026.
//
#include "stepper_driver_stm32.h"

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
static uint32_t StepperDriver_CalcPeriod(const StepperDriverSTM32_t *stp, uint32_t freq) {
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
void StepperDriverSTM32_Init(StepperDriver_t *dr) {
    if (dr == NULL) return;

    StepperDriverSTM32_t* stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL)
        return;

    if (!IS_NVIC_DEVICE_IRQ(stp_dr->htim.IRQn) || stp_dr->htim.Clock < 2 * MIN_CLK_APB)
        return;

    if (stp_dr->dir.Port == NULL && stp_dr->en.Port == NULL)
        return;

    if (stp_dr->GetTick == NULL)
        return;

    /* Установка разрядности по умолчанию (16 бит) если указано неверное значение */
    if (stp_dr->htim.Bit != TIM_16_BIT || stp_dr->htim.Bit != TIM_32_BIT)
        stp_dr->htim.Bit = TIM_16_BIT;

    /* Остановка таймера на время конфигурации */
    stp_dr->htim.Instance->CR1 &= ~TIM_CR1_CEN;

    /* Настройка периода ~1000 Гц */
    stp_dr->htim.Instance->PSC = 0;
    stp_dr->htim.Instance->ARR = StepperDriver_CalcPeriod(stp_dr, 1000);

    /* Включение автоматической предварительной загрузки */
    stp_dr->htim.Instance->CR1 |= TIM_CR1_ARPE;

    /* Настройка режима ШИМ */
    TIM_SET_MODE_PWM(&stp_dr->htim);
    TIM_ENABLE_OCxPRELOAD(&stp_dr->htim);

    /* Начальное значение скважности = 0 (нет импульсов) */
    TIM_SET_COMPARE(&stp_dr->htim, 0);

    /* Разрешение прерывания по обновлению */
    stp_dr->htim.Instance->DIER |= TIM_DIER_UIE;
    stp_dr->htim.Instance->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Включение драйвера
 * @param dr Указатель на структуру драйвера
 *
 * Запускает таймер, активирует выход сравнения,
 * включает вывод разрешения и разрешает прерывания.
 */
void StepperDriverSTM32_Enable(StepperDriver_t *dr) {
    if (dr == NULL) return;

    StepperDriverSTM32_t* stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp_dr->htim.IRQn))
        return;

    stp_dr->htim.Instance->CR1 |= TIM_CR1_CEN;      /* Запуск таймера */
    TIM_ENABLE_COMPARE(&stp_dr->htim);              /* Включение выхода ШИМ */
    stp_dr->en.Port->BSRR = stp_dr->en.Pin;         /* Активация Enable (HIGH) */
    NVIC_EnableIRQ(stp_dr->htim.IRQn);              /* Разрешение прерываний */
}

/**
 * @brief Отключение драйвера
 * @param dr Указатель на структуру драйвера
 *
 * Останавливает таймер, отключает выход сравнения,
 * деактивирует вывод разрешения и запрещает прерывания.
 */
void StepperDriverSTM32_Disable(StepperDriver_t *dr) {
    if (dr == NULL) return;

    StepperDriverSTM32_t* stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp_dr->htim.IRQn))
        return;

    stp_dr->htim.Instance->CR1 &= ~TIM_CR1_CEN;     /* Остановка таймера */
    TIM_DISABLE_COMPARE(&stp_dr->htim);             /* Отключение выхода ШИМ */
    stp_dr->en.Port->BSRR = stp_dr->en.Pin << 16;   /* Деактивация Enable (LOW) */
    NVIC_DisableIRQ(stp_dr->htim.IRQn);             /* Запрет прерываний */
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
bool StepperDriverSTM32_CanSetVel(StepperDriver_t *dr, int32_t vel) {
    if (dr == NULL) return false;

    StepperDriverSTM32_t* stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL)
        return false;

    if (vel == 0)
        return true;

    uint32_t period = StepperDriver_CalcPeriod(stp_dr, abs(vel));
    return period <= ((1ULL << stp_dr->htim.Bit) - 1);
}

/**
 * @brief Установка скорости вращения
 * @param dr Указатель на структуру драйвера
 * @param vel Скорость (имп/сек). Отрицательная - обратное направление.
 * @return true - успешно, false - ошибка
 *
 * @note Выполняется с отключением прерываний для атомарного обновления.
 */
bool StepperDriverSTM32_SetVel(StepperDriver_t *dr, int32_t vel) {
    if (dr == NULL) return false;

    StepperDriverSTM32_t* stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp_dr->htim.IRQn))
        return false;

    if (stp_dr->dir.Port == NULL)
        return false;

    uint32_t period = 0, pulse = 0;

    /* Расчёт параметров для ненулевой скорости */
    if (vel != 0) {
        period = StepperDriver_CalcPeriod(stp_dr, abs(vel));
        if (period <= ((1ULL << stp_dr->htim.Bit) - 1))
            pulse = (period + 1) / 2;  /* Скважность 50% */
        else
            return false;
    }

    /* Атомарное обновление с защитой от прерываний */
    NVIC_DisableIRQ(stp_dr->htim.IRQn);

    if (vel != 0)
        stp_dr->htim.Instance->ARR = period;

    TIM_SET_COMPARE(&stp_dr->htim, pulse);

    /* Установка направления: положительная скорость - один уровень,
     * отрицательная - противоположный */
    if (vel > 0)
        stp_dr->dir.State = stp_dr->dir.Pin;
    else
        stp_dr->dir.State = stp_dr->dir.Pin << 16;

    NVIC_EnableIRQ(stp_dr->htim.IRQn);

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
void StepperDriverSTM32_TIMx_IRQHandler(StepperDriver_t *dr) {
    if (dr == NULL) return;

    StepperDriverSTM32_t* stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL)
        return;

    /* Проверка флага прерывания по обновлению */
    if (stp_dr->htim.Instance->SR & TIM_SR_UIF) {
        stp_dr->htim.Instance->SR &= ~TIM_SR_UIF;  /* Сброс флага */

        /* Обновление состояния направления в момент прерывания */
        if (stp_dr->dir.Port != NULL)
            stp_dr->dir.Port->BSRR = stp_dr->dir.State;
    }
}

const StepperDriver_Interface StepperDriver_Itf = {
    .Init = StepperDriverSTM32_Init,
    .Enable = StepperDriverSTM32_Enable,
    .Disable = StepperDriverSTM32_Disable,
    .CanSetVel = StepperDriverSTM32_CanSetVel,
    .SetVel = StepperDriverSTM32_SetVel,
    .TIMx_IRQHandler = StepperDriverSTM32_TIMx_IRQHandler,
};