//
// Created by pobedimka on 02.07.2026.
//
#include "sigmoid.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

void Sigmoid_Init(Sigmoid_t *sig) {
    if (sig != NULL) {
        if (sig->a < 1)
            sig->a = 1;

        if (sig->v_jump < 1)
            sig->v_jump = 1;

        if (sig->t_jump < 1)
            sig->t_jump = 1;
    }
}

uint32_t Sigmoid_GetAcc(Sigmoid_t *sig) {
    uint32_t acc = 0;
    if (sig != NULL)
        acc = sig->a;
    return acc;
}

int32_t Sigmoid_GetInitVel(Sigmoid_t *sig) {
    int32_t v = 0;
    if (sig != NULL)
        v = sig->v0;
    return v;
}

int32_t Sigmoid_GetTargVel(Sigmoid_t *sig) {
    int32_t v = 0;
    if (sig != NULL)
        v = sig->v1;
    return v;
}

void Sigmoid_SetAcc(Sigmoid_t *pl, const uint32_t a) {
    if (pl != NULL)
        pl->a = a;
}

void Sigmoid_SetVel(Sigmoid_t *sig, int32_t v0, int32_t v1) {
    if (sig == NULL)
        return;

    /* Сохраняем начальную скорость */
    sig->v0 = v0;
    sig->v1 = v1;

    /* Вычисляем разность скоростей */
    sig->dv = v1 > v0 ? sig->v1 - sig->v0 + 1 : sig->v1 - sig->v0 - 1;

    /* t0 = время достижения середины перехода (центр сигмоиды)
     * 500 = 1000 мс / 2 (половина времени разгона при линейном ускорении) */
    sig->t0 = 500.f * (float)abs(sig->dv) / (float)sig->a;

    /* k = коэффициент крутизны сигмоиды */
    sig->k = logf((float)abs(sig->dv)) / sig->t0;

    /* Находим время t_jump / 4, когда |v(t)| достигает v_jump */
    float ratio = fabsf((float)sig->dv) / (float)sig->v_jump;
    sig->t_jump = (sig->t0 - logf(ratio - 1.f) / sig->k) / sig->k_jump;
}

int32_t Sigmoid_GetVel(Sigmoid_t *sig, float dt) {
    if (sig == NULL)
        return 0;

    /* Вычисляем аргумент экспоненты сигмоиды */
    float arg = sig->k * (dt - sig->t0);

    /* Расчёт скорости по формуле сигмоиды: v(t) = v0 + dv / (1 + exp(-k*(t - t0))) */
    float v = (float)sig->v0 + (float)sig->dv / (1.f + expf(-arg));

    /* Устраняем не желательную скорость*/
    if ((float)sig->v_jump > fabsf(v)) {
        if ((abs(sig->v0) < sig->v_jump && sig->t_jump < dt) ||
            (abs(sig->v1) < sig->v_jump && sig->t_jump < ((2 * sig->t0) - dt)))
            v = (v > 0) ? (float)sig->v_jump : -(float)sig->v_jump;
        else
            v = 0;
    }
    return (int32_t)roundf(v);
}

uint32_t Sigmoid_GetTimeAcc(Sigmoid_t *sig) {
    return (uint32_t)(sig->t0 * 2.f);
}
