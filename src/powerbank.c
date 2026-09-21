/* powerbank.c — 整机状态机与低功耗调度
 *
 * 状态：SLEEP（休眠）/ STANDBY（待机）/ CHARGING（自身充电）/ DISCHARGING（对外放电）
 *       / PROTECT（保护锁死）/ RECOVER（故障恢复等待）
 *
 * 低功耗策略（对齐真实充电宝的做法）：
 *   · 无负载且无输入，连续 idle 超过 sleep_after_ticks -> 进 SLEEP，静态电流降到 μA 级
 *   · 有负载插入（isr/比较器唤醒）或插上输入 -> 立即回 STANDBY，记录一次唤醒事件
 *   · 保护触发 -> PROTECT（MOS 关断）；故障消失后等 recover_ticks 再自动重试
 */
#include "pb.h"
#include <string.h>

static const char *PB_ST_NAME[PB_ST_COUNT] = {
    "SLEEP", "STANDBY", "CHARGING", "DISCHARGING", "PROTECT", "RECOVER"
};

/* 判定"有负载"的电流门槛（A） */
#define LOAD_THRESHOLD_A   0.010
/* 判定"有输入"的门槛已在参数里给（vin_present） */

const char *pb_state_name(pb_state_t s)
{
    if ((int)s < 0 || s >= PB_ST_COUNT) return "?";
    return PB_ST_NAME[s];
}

void pb_sys_init(pb_sys_t *s, uint32_t sleep_after_ticks, uint32_t recover_ticks,
                 double sleep_ma, double standby_ma)
{
    memset(s, 0, sizeof(*s));
    s->state = PB_ST_STANDBY;
    s->prev = PB_ST_STANDBY;
    s->sleep_after_ticks = sleep_after_ticks ? sleep_after_ticks : 1;
    s->recover_ticks = recover_ticks ? recover_ticks : 1;
    s->sleep_current_ma = sleep_ma;
    s->standby_current_ma = standby_ma;
}

pb_state_t pb_sys_next_state(pb_state_t cur, double load_a, bool vin_present,
                             pb_fault_t flt, uint32_t idle_ticks,
                             uint32_t sleep_after)
{
    if (flt != PB_FLT_NONE) return PB_ST_PROTECT;

    if (cur == PB_ST_PROTECT) {
        /* 故障源消失后由调用方推进到 RECOVER，这里保持 */
        return PB_ST_PROTECT;
    }
    if (cur == PB_ST_RECOVER) return PB_ST_RECOVER;

    if (load_a > LOAD_THRESHOLD_A)  return PB_ST_DISCHARGING;
    if (vin_present)                return PB_ST_CHARGING;
    if (idle_ticks >= sleep_after)  return PB_ST_SLEEP;
    return PB_ST_STANDBY;
}

double pb_sys_step(pb_sys_t *s, double load_a, bool vin_present, pb_fault_t flt)
{
    pb_state_t next;

    s->prev = s->state;
    s->ticks_in_state++;

    /* 无负载计时（用于进休眠） */
    if (load_a <= LOAD_THRESHOLD_A && !vin_present) {
        if (s->idle_ticks < 0xFFFFFFFFu) s->idle_ticks++;
    } else {
        s->idle_ticks = 0;
    }

    /* 从休眠/保护/恢复态出来 */
    if (s->state == PB_ST_SLEEP) {
        if (load_a > LOAD_THRESHOLD_A || vin_present) {
            s->state = load_a > LOAD_THRESHOLD_A ? PB_ST_DISCHARGING : PB_ST_CHARGING;
            s->wake_events++;
            s->ticks_in_state = 0;
        } else {
            return s->sleep_current_ma;
        }
    }
    if (s->state == PB_ST_PROTECT) {
        if (flt == PB_FLT_NONE) {
            s->state = PB_ST_RECOVER;
            s->ticks_in_state = 0;
        } else {
            return s->standby_current_ma;
        }
    }
    if (s->state == PB_ST_RECOVER) {
        if (flt != PB_FLT_NONE) {
            s->state = PB_ST_PROTECT;
            s->ticks_in_state = 0;
        } else if (s->ticks_in_state >= s->recover_ticks) {
            s->state = PB_ST_STANDBY;
            s->ticks_in_state = 0;
        } else {
            return s->standby_current_ma;
        }
    }

    next = pb_sys_next_state(s->state, load_a, vin_present, flt,
                             s->idle_ticks, s->sleep_after_ticks);
    if (next != s->state) {
        s->state = next;
        s->ticks_in_state = 0;
    }

    if (s->state == PB_ST_SLEEP) return s->sleep_current_ma;
    return s->standby_current_ma;
}

/* ================================================== 整机（电池 + BMS + PD） */
void pb_dev_init(pb_dev_t *d, const pb_batt_cfg_t *bcfg, double soc0,
                 const pd_pdo_t *caps, int n_caps,
                 uint32_t sleep_after_ticks, uint32_t recover_ticks)
{
    memset(d, 0, sizeof(*d));
    pb_batt_init(&d->batt, bcfg, soc0);
    pb_bms_init(&d->bms, 3, 3, 2, 2, 5);
    pd_port_init(&d->pd, caps, n_caps);
    pb_sys_init(&d->sys, sleep_after_ticks, recover_ticks, 0.05, 8.0);
    d->load_a = 0.0;
    d->vin_a = 0.0;
    d->vin_present = false;
    d->static_ma = d->sys.standby_current_ma;
}

void pb_dev_set_load(pb_dev_t *d, double load_a) { d->load_a = load_a; }
void pb_dev_plug_in(pb_dev_t *d, bool present)   { d->vin_present = present; }

void pb_dev_step(pb_dev_t *d)
{
    const double dt = 0.001;                 /* 1 ms 调度周期 */
    double i_batt;
    pb_fault_t flt;
    bool mos_on;

    /* 1) 先按**上一拍状态**判断 MOS 是否导通，再跑保护判定
     *    （顺序很关键：保护必须先于状态机，否则故障会晚一拍生效） */
    mos_on = (d->sys.state != PB_ST_PROTECT) && (d->sys.state != PB_ST_RECOVER)
             && (d->sys.state != PB_ST_SLEEP || d->load_a > 0.010);
    flt = pb_bms_step(&d->bms, &d->batt, mos_on);

    /* 2) 整机状态机：把当前故障作为输入，决定休眠 / 待机 / 放电 / 充电 / 保护 / 恢复 */
    d->static_ma = pb_sys_step(&d->sys, d->load_a, d->vin_present, flt);

    /* 3) 电池电流：放电为正（含静态电流），充电为负 */
    if (mos_on && d->sys.state == PB_ST_DISCHARGING) {
        i_batt = d->load_a + d->static_ma / 1000.0;
    } else if (d->sys.state == PB_ST_CHARGING && d->vin_present) {
        i_batt = -(d->load_a + 0.5);          /* 0.5 A 充电 */
    } else {
        i_batt = d->static_ma / 1000.0;
    }

    /* 4) 推进电池模型 */
    pb_batt_update(&d->batt, i_batt, dt);

    d->t_ms++;
}
