//
// Created by Pobed on 27.07.2026.
//
#include "motor_driver.h"

/** @brief Глобальный экземпляр интерфейса драйвера */
const MotorDriver_Interface MotorDriver_Itf = {
    .Init = MotorDriver_Init,
    .Enable = MotorDriver_Enable,
    .Disable = MotorDriver_Disable,
    .СanSetVel = MotorDriver_СanSetVel,
    .SetVel = MotorDriver_SetVel,
    .TIMx_IRQHandler = MotorDriver_TIMx_IRQHandler,
};
