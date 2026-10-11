//
// Created by Pobed on 07.10.2026.
//
//
// Created for exponential trajectory
//
#include "exponential.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

void Exp_Init(Planner_t *pl) {
    if (pl == NULL) return;

    Exponential_t *exp = (Exponential_t *)pl;

    if (exp->a < 1)
        exp->a = 1;

    if (exp->v_jump < 1)
        exp->v_jump = 1;
}

uint32_t Exp_GetAcc(Planner_t *pl) {
    if (pl == NULL) return 0;

    Exponential_t *exp = (Exponential_t *)pl;

    return exp->a;
}

int32_t Exp_GetInitVel(Planner_t *pl) {
    if (pl == NULL) return 0;

    Exponential_t *exp = (Exponential_t *)pl;

    return exp->v0;
}

int32_t Exp_GetTargVel(Planner_t *pl) {
    if (pl == NULL) return 0;

    Exponential_t *exp = (Exponential_t *)pl;

    return exp->v1;
}

void Exp_SetAcc(Planner_t *pl, const uint32_t a) {
    if (pl == NULL) return;

    Exponential_t *exp = (Exponential_t *)pl;

    exp->a = a;
}

void Exp_SetVel(Planner_t *pl, int32_t v0, int32_t v1) {
    if (pl == NULL) return;

    Exponential_t *exp = (Exponential_t *)pl;

    exp->v0 = v0;
    exp->v1 = v1;

    /* Вычисляем разность скоростей (защита от нуля) */
    exp->dv = exp->v1 - exp->v0;

    /* lambda_sec = a / |dv|.
       Так как dt передается в мс, переводим lambda в 1/мс: lambda_ms = lambda_sec / 1000 */
    exp->lambda = (float)exp->a / (200.0f * (float)abs(exp->dv));

    /* Практическое время окончания. 5 постоянных времени (tau = 1/lambda)
       обеспечивают достижение 99.3% от целевой скорости */
    if (exp->lambda > 0.000001f)
        exp->t = 5.0f / exp->lambda;
    else
        exp->t = 0.0f;
}

int32_t Exp_GetVel(Planner_t *pl, float dt) {
    if (pl == NULL) return 0;

    Exponential_t *exp = (Exponential_t *)pl;

    int32_t v;

    /* Ограничиваем время, так как функция асимптотическая */
    if (dt <= 0.0f)
        v = exp->v0;
    else if (dt >= exp->t)
        v = exp->v1;
    else
        v = exp->v1 - (int32_t)roundf((float)exp->dv * expf(-exp->lambda * dt));

    /* Устраняем нежелательную скорость (логика из оригинала) */
    if (abs(v) < exp->v_jump)
        v = 0;

    return v;
}

uint32_t Exp_GetTimeAcc(Planner_t *pl) {
    if (pl == NULL) return 0;

    Exponential_t *exp = (Exponential_t *)pl;

    return (uint32_t)exp->t;
}

const Planner_Interface Exponential_Itf = {
    .Init       = Exp_Init,
    .GetAcc     = Exp_GetAcc,
    .SetAcc     = Exp_SetAcc,
    .GetInitVel = Exp_GetInitVel,
    .GetTargVel = Exp_GetTargVel,
    .SetVel     = Exp_SetVel,
    .GetVel     = Exp_GetVel,
    .GetTimeAcc = Exp_GetTimeAcc,
};
