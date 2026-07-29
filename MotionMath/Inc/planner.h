//
// Created by Pobed on 27.07.2026.
//

#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>

#include "sigmoid.h"

typedef Sigmoid_Type Planner_Type;
typedef Sigmoid_Init Planner_Init;

typedef struct {
    void (*Init)(Planner_Type *pl, Planner_Init *init);
    void (*DeInit)(Planner_Type *pl);
    uint32_t (*GetAcc)(Planner_Type *p);
    void (*SetAcc)(Planner_Type *pl, uint32_t a);
    int32_t (*GetInitVel)(Planner_Type *pl);
    int32_t (*GetTargVel)(Planner_Type *pl);
    void (*SetVel)(Planner_Type *pl, int32_t v0, int32_t v1);
    int32_t (*GetVel)(Planner_Type *pl, float dt);
    uint32_t (*GetTimeAcc)(Planner_Type *pl);
} Planner_Interface;

extern const Planner_Interface Planner_Itf;

#endif
