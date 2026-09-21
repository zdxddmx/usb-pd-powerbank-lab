/* battery.c — 锂电池模型：OCV-SOC 查表 + 库仑计数 + 内阻 + 温升
 *
 * SOC 估算采用工业界最常见的两段式做法：
 *   1) 静置/低倍率时用 **OCV 查表**校正（分段线性插值）
 *   2) 动态过程中用 **库仑计数**（安时积分）跟踪，并对 OCV 校正做缓慢回拉
 * 端电压 = OCV(SOC) - I * R_internal（放电时 I > 0）
 */
#include "pb.h"
#include <math.h>
#include <string.h>

/* OCV-SOC 表（典型三元锂，10 点分段线性） */
static const double OCV_SOC[] = {
    0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00
};
static const double OCV_V[] = {
    3.00, 3.45, 3.58, 3.66, 3.71, 3.76, 3.82, 3.89, 3.97, 4.05, 4.13, 4.20
};
#define OCV_N (int)(sizeof(OCV_SOC) / sizeof(OCV_SOC[0]))

double pb_batt_ocv(double soc)
{
    int i;
    if (soc <= OCV_SOC[0]) return OCV_V[0];
    if (soc >= OCV_SOC[OCV_N - 1]) return OCV_V[OCV_N - 1];
    for (i = 1; i < OCV_N; i++) {
        if (soc <= OCV_SOC[i]) {
            double x0 = OCV_SOC[i - 1], x1 = OCV_SOC[i];
            double y0 = OCV_V[i - 1], y1 = OCV_V[i];
            double w = (soc - x0) / (x1 - x0);
            return y0 + w * (y1 - y0);
        }
    }
    return OCV_V[OCV_N - 1];
}

void pb_batt_init(pb_batt_t *b, const pb_batt_cfg_t *cfg, double soc0)
{
    memset(b, 0, sizeof(*b));
    b->cfg = *cfg;
    b->soc = (soc0 < 0.0) ? 0.0 : (soc0 > 1.0 ? 1.0 : soc0);
    b->temp_c = 25.0;
    b->v_term = pb_batt_ocv(b->soc);
}

void pb_batt_reset_soc(pb_batt_t *b, double soc)
{
    b->soc = (soc < 0.0) ? 0.0 : (soc > 1.0 ? 1.0 : soc);
    b->v_term = pb_batt_ocv(b->soc);
}

double pb_batt_update(pb_batt_t *b, double i_load, double dt)
{
    const pb_batt_cfg_t *c = &b->cfg;
    double d_ah, d_soc;

    b->i_load = i_load;

    /* 库仑计数：I>0 放电，I<0 充电 */
    d_ah = i_load * dt / 3600.0;
    if (i_load > 0.0) {
        d_soc = d_ah / c->capacity_ah;
        b->discharge_ah += d_ah;
    } else {
        d_soc = d_ah * c->coulomb_eff / c->capacity_ah;  /* 充电效率 */
        b->charge_ah += -d_ah;
    }
    b->soc -= d_soc;
    if (b->soc > 1.0) b->soc = 1.0;
    if (b->soc < 0.0) b->soc = 0.0;

    /* 端电压 = OCV - I*R */
    b->v_term = pb_batt_ocv(b->soc) - i_load * c->r_internal;

    /* 一阶热模型：损耗 ≈ I^2 * R，热阻 8 C/W，热时间常数 60 s */
    {
        double loss = i_load * i_load * c->r_internal;   /* W */
        double target = 25.0 + loss * 8.0;
        b->temp_c += (target - b->temp_c) * dt / 60.0;
    }

    b->t += dt;
    return b->v_term;
}
