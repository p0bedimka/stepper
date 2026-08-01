//
// Created by Pobed on 27.07.2026.
//

#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>

#include "sigmoid.h"

typedef Sigmoid_t Planner_t;

typedef struct {
    void (*Init)(Planner_t *pl);
    void (*DeInit)(Planner_t *pl);
    uint32_t (*GetAcc)(Planner_t *p);
    void (*SetAcc)(Planner_t *pl, uint32_t a);
    int32_t (*GetInitVel)(Planner_t *pl);
    int32_t (*GetTargVel)(Planner_t *pl);
    void (*SetVel)(Planner_t *pl, int32_t v0, int32_t v1);
    int32_t (*GetVel)(Planner_t *pl, float dt);
    uint32_t (*GetTimeAcc)(Planner_t *pl);
} Planner_Interface;

extern const Planner_Interface Planner_Itf;

#endif
