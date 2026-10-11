//
// Created by p0bedimka on 27.07.2026.
//

/**
 * @file stepper_driver_stm32.c
 * @brief Реализация драйвера шагового двигателя для микроконтроллеров STM32.
 *
 * Таймер настраивается в режим PWM (режим 1: активный уровень, пока CNT < CCR).
 * Скважность всегда 50%: CCR = (ARR + 1) / 2.
 * Частота PWM задаётся через регистр ARR:
 *   f = TIM_CLK / ((PSC + 1) * (ARR + 1))
 * Предделитель PSC всегда равен 0 — вся грубая настройка через ARR.
 *
 * Направление (DIR) обновляется в обработчике прерывания по флагу UIF (Update),
 * что гарантирует смену направления строго в момент начала нового периода —
 * это исключает генерацию "коротких" импульсов-шагов при смене DIR.
 */

#include "stepper_driver_stm32.h"
#include <stdlib.h>

/* =========================================================================
 * Макросы для унифицированной работы с каналами таймера (1..4)
 * ========================================================================= */

/** @brief Запись значения в регистр сравнения выбранного канала. */
#define TIM_SET_COMPARE(HANDLE, COMPARE) \
    (((HANDLE)->Channel == TIM_CHAN_1) ? ((HANDLE)->Instance->CCR1 = (COMPARE)) : \
     ((HANDLE)->Channel == TIM_CHAN_2) ? ((HANDLE)->Instance->CCR2 = (COMPARE)) : \
     ((HANDLE)->Channel == TIM_CHAN_3) ? ((HANDLE)->Instance->CCR3 = (COMPARE)) : \
                                         ((HANDLE)->Instance->CCR4 = (COMPARE)))

/** @brief Включение предварительной загрузки регистра сравнения (OCxPE). */
#define TIM_ENABLE_OCxPRELOAD(HANDLE) \
    (((HANDLE)->Channel == TIM_CHAN_1) ? ((HANDLE)->Instance->CCMR1 |= TIM_CCMR1_OC1PE) : \
     ((HANDLE)->Channel == TIM_CHAN_2) ? ((HANDLE)->Instance->CCMR1 |= TIM_CCMR1_OC2PE) : \
     ((HANDLE)->Channel == TIM_CHAN_3) ? ((HANDLE)->Instance->CCMR2 |= TIM_CCMR2_OC3PE) : \
                                         ((HANDLE)->Instance->CCMR2 |= TIM_CCMR2_OC4PE))

/** @brief Настройка режима PWM (режим 1) для выбранного канала. */
#define TIM_SET_MODE_PWM(HANDLE) \
    (((HANDLE)->Channel == TIM_CHAN_1) ? ((HANDLE)->Instance->CCMR1 = ((HANDLE)->Instance->CCMR1  & ~TIM_CCMR1_OC1M) | (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)) : \
     ((HANDLE)->Channel == TIM_CHAN_2) ? ((HANDLE)->Instance->CCMR1 = ((HANDLE)->Instance->CCMR1  & ~TIM_CCMR2_OC3M) | (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2)) : \
     ((HANDLE)->Channel == TIM_CHAN_3) ? ((HANDLE)->Instance->CCMR2 = ((HANDLE)->Instance->CCMR2  & ~TIM_CCMR2_OC3M) | (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2)) : \
                                         ((HANDLE)->Instance->CCMR2 = ((HANDLE)->Instance->CCMR2  & ~TIM_CCMR2_OC4M) | (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2)))

/** @brief Включение выхода сравнения выбранного канала (CCxE). */
#define TIM_ENABLE_COMPARE(HANDLE) \
    (((HANDLE)->Channel == TIM_CHAN_1) ? ((HANDLE)->Instance->CCER |= TIM_CCER_CC1E) : \
     ((HANDLE)->Channel == TIM_CHAN_2) ? ((HANDLE)->Instance->CCER |= TIM_CCER_CC2E) : \
     ((HANDLE)->Channel == TIM_CHAN_3) ? ((HANDLE)->Instance->CCER |= TIM_CCER_CC3E) : \
                                         ((HANDLE)->Instance->CCER |= TIM_CCER_CC4E))

/** @brief Отключение выхода сравнения выбранного канала. */
#define TIM_DISABLE_COMPARE(HANDLE) \
    (((HANDLE)->Channel == TIM_CHAN_1) ? ((HANDLE)->Instance->CCER  &= ~TIM_CCER_CC1E) : \
     ((HANDLE)->Channel == TIM_CHAN_2) ? ((HANDLE)->Instance->CCER  &= ~TIM_CCER_CC2E) : \
     ((HANDLE)->Channel == TIM_CHAN_3) ? ((HANDLE)->Instance->CCER  &= ~TIM_CCER_CC3E) : \
                                         ((HANDLE)->Instance->CCER  &= ~TIM_CCER_CC4E))

/* =========================================================================
 * Внутренние функции
 * ========================================================================= */

/**
 * @brief Расчёт значения регистра ARR для заданной частоты PWM.
 *
 * Формула:
 *   ARR = TIM_CLK / (freq * (PSC + 1)) - 1
 *
 * @param stp  Указатель на драйвер (нужен для доступа к Clock и PSC).
 * @param freq Желаемая частота (Гц), равная модулю скорости (шагов/с).
 * @return Значение ARR, либо 0, если freq == 0.
 */
static uint32_t StepperDriver_CalcPeriod(const StepperDriverSTM32_t *stp, uint32_t freq) {
    if (freq == 0) return 0;
    return stp->htim.Clock / (freq * (stp->htim.Instance->PSC + 1)) - 1;
}

/* =========================================================================
 * Реализация интерфейса StepperDriver_Interface
 * ========================================================================= */

/**
 * @brief Инициализация драйвера.
 *
 * Выполняет:
 *   - проверку корректности переданных параметров (Instance, IRQn, Clock, GetTick);
 *   - остановку таймера;
 *   - настройку PSC = 0, ARR для частоты 1000 Гц (стартовое значение);
 *   - включение ARPE (auto-reload preload) и режима PWM;
 *   - включение прерывания по событию Update (UIE).
 *
 * @note Флаг UIF сбрасывается, чтобы избежать ложного срабатывания ISR сразу после Init.
 */
void StepperDriverSTM32_Init(StepperDriver_t *dr) {
    if (dr == NULL) return;
    StepperDriverSTM32_t *stp_dr = (StepperDriverSTM32_t *)dr;

    // Базовые проверки валидности
    if (stp_dr->htim.Instance == NULL) return;
    if (!IS_NVIC_DEVICE_IRQ(stp_dr->htim.IRQn) || stp_dr->htim.Clock < 2 * MIN_CLK_APB) return;
    if (stp_dr->dir.Port == NULL && stp_dr->en.Port == NULL) return;
    if (stp_dr->GetTick == NULL) return;

    // Разрядность таймера (по умолчанию 16 бит)
    if (stp_dr->htim.Bit != TIM_16_BIT && stp_dr->htim.Bit != TIM_32_BIT)
        stp_dr->htim.Bit = TIM_16_BIT;

    // Остановка таймера и базовая настройка
    stp_dr->htim.Instance->CR1 &= ~TIM_CR1_CEN;
    stp_dr->htim.Instance->PSC = 0;
    stp_dr->htim.Instance->ARR = StepperDriver_CalcPeriod(stp_dr, 1000);
    stp_dr->htim.Instance->CR1 |= TIM_CR1_ARPE;

    // Настройка канала в режим PWM с предварительной загрузкой
    TIM_SET_MODE_PWM(&stp_dr->htim);
    TIM_ENABLE_OCxPRELOAD(&stp_dr->htim);
    TIM_SET_COMPARE(&stp_dr->htim, 0);

    // Включение прерывания по Update и сброс флага
    stp_dr->htim.Instance->DIER |= TIM_DIER_UIE;
    stp_dr->htim.Instance->SR   &= ~TIM_SR_UIF;
}

/**
 * @brief Включение драйвера.
 *
 * Запускает таймер, разрешает выход канала, подаёт питание (EN = 1),
 * разрешает прерывание в NVIC.
 */
void StepperDriverSTM32_Enable(StepperDriver_t *dr) {
    if (dr == NULL) return;
    StepperDriverSTM32_t *stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp_dr->htim.IRQn)) return;

    stp_dr->htim.Instance->CR1 |= TIM_CR1_CEN;
    TIM_ENABLE_COMPARE(&stp_dr->htim);
    stp_dr->en.Port->BSRR = stp_dr->en.Pin;          // EN = 1
    NVIC_EnableIRQ(stp_dr->htim.IRQn);
}

/**
 * @brief Отключение драйвера.
 *
 * Останавливает таймер, запрещает выход канала, снимает питание (EN = 0),
 * запрещает прерывание в NVIC.
 */
void StepperDriverSTM32_Disable(StepperDriver_t *dr) {
    if (dr == NULL) return;
    StepperDriverSTM32_t *stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp_dr->htim.IRQn)) return;

    stp_dr->htim.Instance->CR1 &= ~TIM_CR1_CEN;
    TIM_DISABLE_COMPARE(&stp_dr->htim);
    stp_dr->en.Port->BSRR = (uint32_t)stp_dr->en.Pin << 16;  // EN = 0 (сброс через BR)
    NVIC_DisableIRQ(stp_dr->htim.IRQn);
}

/**
 * @brief Проверка, может ли таймер обеспечить заданную скорость.
 *
 * Вычисляет требуемое значение ARR и сравнивает с максимальным значением,
 * которое может вместить регистр (2^16 - 1 или 2^32 - 1).
 *
 * @return true, если скорость достижима.
 */
bool StepperDriverSTM32_CanSetVel(StepperDriver_t *dr, int32_t vel) {
    if (dr == NULL) return false;
    StepperDriverSTM32_t *stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL) return false;
    if (vel == 0) return true;

    uint32_t period = StepperDriver_CalcPeriod(stp_dr, (uint32_t)abs(vel));
    return period <= ((1ULL << stp_dr->htim.Bit) - 1);
}

/**
 * @brief Установка новой скорости.
 *
 * Выполняется в три этапа:
 *   1. Вычисление ARR и CCR для заданной скорости (скважность 50%).
 *   2. Временное запрещение прерываний NVIC — чтобы атомарно обновить ARR,
 *      CCR и состояние DIR.
 *   3. Восстановление прерываний.
 *
 * Направление задаётся через переменную dir.State, которая будет записана
 * в порт в следующем обработчике прерывания по Update.
 *
 * @return true в случае успеха, false — если скорость недостижима.
 */
bool StepperDriverSTM32_SetVel(StepperDriver_t *dr, int32_t vel) {
    if (dr == NULL) return false;
    StepperDriverSTM32_t *stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL || !IS_NVIC_DEVICE_IRQ(stp_dr->htim.IRQn)) return false;
    if (stp_dr->dir.Port == NULL) return false;

    uint32_t period = 0, pulse = 0;

    if (vel != 0) {
        period = StepperDriver_CalcPeriod(stp_dr, (uint32_t)abs(vel));
        if (period <= ((1ULL << stp_dr->htim.Bit) - 1))
            pulse = (period + 1) / 2;   // Скважность 50%
        else
            return false;
    }

    // Критическая секция: атомарное обновление регистров и DIR
    NVIC_DisableIRQ(stp_dr->htim.IRQn);

    if (vel != 0)
        stp_dr->htim.Instance->ARR = period;
    TIM_SET_COMPARE(&stp_dr->htim, pulse);

    // Формируем значение для BSRR: установка или сброс пина DIR
    if (vel > 0)
        stp_dr->dir.State = stp_dr->dir.Pin;          // DIR = 1
    else
        stp_dr->dir.State = (uint32_t)stp_dr->dir.Pin << 16;  // DIR = 0

    NVIC_EnableIRQ(stp_dr->htim.IRQn);

    return true;
}

/**
 * @brief Обработчик прерывания таймера.
 *
 * При возникновении события Update (UIF) сбрасывает флаг и записывает
 * подготовленное значение dir.State в порт через регистр BSRR.
 *
 * @note Запись в BSRR атомарна и не требует чтения-модификации-записи,
 *       что исключает гонки с другими операциями над портом.
 */
void StepperDriverSTM32_TIMx_IRQHandler(StepperDriver_t *dr) {
    if (dr == NULL) return;
    StepperDriverSTM32_t *stp_dr = (StepperDriverSTM32_t *)dr;

    if (stp_dr->htim.Instance == NULL) return;

    if (stp_dr->htim.Instance->SR & TIM_SR_UIF) {
        stp_dr->htim.Instance->SR &= ~TIM_SR_UIF;     // Сброс флага
        if (stp_dr->dir.Port != NULL)
            stp_dr->dir.Port->BSRR = stp_dr->dir.State;
    }
}

/// Таблица функций драйвера для STM32
const StepperDriver_Interface StepperDriver_Itf = {
    .Init             = StepperDriverSTM32_Init,
    .Enable           = StepperDriverSTM32_Enable,
    .Disable          = StepperDriverSTM32_Disable,
    .CanSetVel        = StepperDriverSTM32_CanSetVel,
    .SetVel           = StepperDriverSTM32_SetVel,
    .TIMx_IRQHandler  = StepperDriverSTM32_TIMx_IRQHandler,
};