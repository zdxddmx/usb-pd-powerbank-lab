/* bms.c — 锂电池保护逻辑
 *
 * 七类保护：过压 / 欠压 / 充电过流 / 放电过流 / 短路 / 过温 / 低温
 * 每类都有独立的**连续越限去抖计数**，避免负载跳变与采样噪声误触发；
 * 短路保护去抖最短（2 拍），且一旦触发即锁存，须显式 clear。
 */
#include "pb.h"
#include <math.h>
#include <string.h>

static const char *FLT_NAME[PB_FLT_COUNT] = {
    "NONE", "CELL_OV", "CELL_UV", "OC_CHG", "OC_DIS", "SHORT", "OT", "UT"
};

const char *pb_fault_name(pb_fault_t f)
{
    if ((int)f < 0 || f >= PB_FLT_COUNT) return "?";
    return FLT_NAME[f];
}

void pb_bms_init(pb_bms_t *m, uint8_t deb_ov, uint8_t deb_uv,
                 uint8_t deb_oc, uint8_t deb_sc, uint8_t deb_t)
{
    memset(m, 0, sizeof(*m));
    m->debounce[PB_FLT_CELL_OV] = deb_ov ? deb_ov : 1;
    m->debounce[PB_FLT_CELL_UV] = deb_uv ? deb_uv : 1;
    m->debounce[PB_FLT_OC_DIS]  = deb_oc ? deb_oc : 1;
    m->debounce[PB_FLT_OC_CHG]  = deb_oc ? deb_oc : 1;
    m->debounce[PB_FLT_SHORT]   = deb_sc ? deb_sc : 1;
    m->debounce[PB_FLT_OT]      = deb_t ? deb_t : 1;
    m->debounce[PB_FLT_UT]      = deb_t ? deb_t : 1;
}

void pb_bms_clear(pb_bms_t *m)
{
    memset(m->cnt, 0, sizeof(m->cnt));
    memset(m->active, 0, sizeof(m->active));
    m->last = PB_FLT_NONE;
}

/* 条件成立 -> 累加；条件消失 -> 立即清零（迟滞由阈值与去抖共同提供） */
static bool debounce(pb_bms_t *m, pb_fault_t f, bool cond)
{
    if (!cond) {
        m->cnt[f] = 0;
        if (f != PB_FLT_SHORT) m->active[f] = false;   /* 短路锁存 */
        return m->active[f];
    }
    if (m->cnt[f] < 255) m->cnt[f]++;
    if (m->cnt[f] >= m->debounce[f]) m->active[f] = true;
    return m->active[f];
}

pb_fault_t pb_bms_step(pb_bms_t *m, const pb_batt_t *b, bool mos_enabled)
{
    const pb_batt_cfg_t *c = &b->cfg;
    double i = b->i_load;
    pb_fault_t hit = PB_FLT_NONE;

    if (!mos_enabled) {
        return PB_FLT_NONE;      /* MOS 关断时不判定，避免自锁 */
    }

    /* 优先级：短路 > 过流 > 过温/低温 > 过压/欠压 */
    if (debounce(m, PB_FLT_SHORT, fabs(i) > c->i_sc))        hit = PB_FLT_SHORT;
    else if (debounce(m, PB_FLT_OC_DIS, i > c->i_oc_dis))    hit = PB_FLT_OC_DIS;
    else if (debounce(m, PB_FLT_OC_CHG, -i > c->i_oc_chg))   hit = PB_FLT_OC_CHG;
    else if (debounce(m, PB_FLT_OT, b->temp_c > c->t_ot))    hit = PB_FLT_OT;
    else if (debounce(m, PB_FLT_UT, b->temp_c < c->t_ut))    hit = PB_FLT_UT;
    else if (debounce(m, PB_FLT_CELL_OV, b->v_term > c->v_ov)) hit = PB_FLT_CELL_OV;
    else if (debounce(m, PB_FLT_CELL_UV, b->v_term < c->v_uv)) hit = PB_FLT_CELL_UV;

    if (hit != PB_FLT_NONE) {
        m->last = hit;
        m->total_trips++;
    }
    return hit;
}
