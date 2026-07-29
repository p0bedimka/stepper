//
// Created by Pobed on 27.07.2026.
//
#include "planner.h"

const Planner_Interface Planner_Itf = {
    .Init       = Sigmoid_Initialization,
    .DeInit     = Sigmoid_DeInit,
    .GetAcc     = Sigmoid_GetAcc,
    .SetAcc     = Sigmoid_SetAcc,
    .GetInitVel = Sigmoid_GetInitVel,
    .GetTargVel = Sigmoid_GetTargVel,
    .SetVel     = Sigmoid_SetVel,
    .GetVel     = Sigmoid_GetVel,
};
