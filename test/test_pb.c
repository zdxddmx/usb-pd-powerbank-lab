/* test_pb.c — 单元测试（纯 C，无第三方框架） */
#include "pb.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static int passed = 0, failed = 0;

#define CHECK(cond, ...) do {                     \
    if (cond) { passed++; printf("  [PASS] "); }   \
    else      { failed++; printf("  [FAIL] "); }   \
    printf(__VA_ARGS__);                           \
    printf("\n");                                  \
} while (0)

#define NEAR(a, b, tol) (fabs((a) - (b)) <= (tol))

static void cfg_basic(pb_batt_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->capacity_ah = 2.0;      /* 2 Ah */
    c->v_full = 4.20;
    c->v_nominal = 3.70;
    c->v_empty = 3.00;
    c->r_internal = 0.06;      /* 60 mOhm */
    c->coulomb_eff = 0.99;
    c->v_ov = 4.25;
    c->v_uv = 3.10;
    c->i_oc_chg = 3.0;
    c->i_oc_dis = 5.0;
    c->i_sc = 8.0;
    c->t_ot = 60.0;
    c->t_ut = -10.0;
}

/* ------------------------------------------------------------ T1 battery */
static void t1_battery(void)
{
    pb_batt_cfg_t cfg; pb_batt_t b;
    int i;

    printf("T1 battery（电池模型与 SOC）\n");
    cfg_basic(&cfg);

    CHECK(NEAR(pb_batt_ocv(0.0), 3.00, 1e-9), "SOC=0 -> OCV = %.3f V", pb_batt_ocv(0.0));
    CHECK(NEAR(pb_batt_ocv(1.0), 4.20, 1e-9), "SOC=1 -> OCV = %.3f V", pb_batt_ocv(1.0));
    CHECK(NEAR(pb_batt_ocv(0.5), 3.82, 1e-9), "SOC=0.5 -> OCV = %.3f V（查表点）", pb_batt_ocv(0.5));
    /* 插值检查：0.15 应在 3.58 与 3.66 之间 */
    CHECK(pb_batt_ocv(0.15) > 3.58 && pb_batt_ocv(0.15) < 3.66,
          "SOC=0.15 分段线性插值 = %.4f V", pb_batt_ocv(0.15));

    /* 库仑计数：2 Ah 电池以 2 A 放电 1800 s = 1 Ah -> SOC 掉 0.5 */
    pb_batt_init(&b, &cfg, 1.0);
    for (i = 0; i < 1800; i++) pb_batt_update(&b, 2.0, 1.0);
    CHECK(NEAR(b.soc, 0.5, 0.01), "2 A 放 1800 s 后 SOC = %.4f（期望 0.5）", b.soc);
    CHECK(NEAR(b.discharge_ah, 1.0, 0.01), "累计放电 = %.3f Ah", b.discharge_ah);

    /* 端电压含内阻压降 */
    CHECK(NEAR(b.v_term, pb_batt_ocv(b.soc) - 2.0 * cfg.r_internal, 1e-9),
          "端电压含内阻压降：%.4f V", b.v_term);

    /* SOC 边界钳位 */
    pb_batt_init(&b, &cfg, 0.99);
    for (i = 0; i < 100000; i++) pb_batt_update(&b, 2.0, 1.0);
    CHECK(b.soc >= 0.0 && b.soc <= 1.0, "SOC 不越界：%.6f", b.soc);

    /* 充电效率体现在库仑效率上 */
    pb_batt_init(&b, &cfg, 0.5);
    for (i = 0; i < 1800; i++) pb_batt_update(&b, -2.0, 1.0);
    CHECK(NEAR(b.soc, 0.5 + 1.0 * cfg.coulomb_eff / cfg.capacity_ah, 0.01),
          "充电 1 Ah 后 SOC = %.4f（含库仑效率 %.2f）", b.soc, cfg.coulomb_eff);
}

/* ---------------------------------------------------------------- T2 bms */
static void t2_bms(void)
{
    pb_batt_cfg_t cfg; pb_batt_t b; pb_bms_t m;
    pb_fault_t f;
    int i;

    printf("T2 bms（七类保护与去抖）\n");
    cfg_basic(&cfg);
    pb_batt_init(&b, &cfg, 0.5);

    /* 短路去抖 2 拍 + 锁存 */
    pb_bms_init(&m, 3, 3, 2, 2, 5);
    b.i_load = 10.0;
    f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_NONE, "短路第 1 拍不触发（去抖）");
    f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_SHORT, "短路连续 2 拍 -> SHORT");
    b.i_load = 0.0;
    f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_SHORT, "短路解锁（锁存）：故障源消失仍保持 SHORT");

    /* 过流去抖 2 拍 */
    pb_bms_init(&m, 3, 3, 2, 2, 5);
    b.i_load = 6.0;
    pb_bms_step(&m, &b, true);
    f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_OC_DIS, "放电过流 6 A 连续 2 拍 -> OC_DIS");

    /* 充电过流（负电流） */
    pb_bms_init(&m, 3, 3, 2, 2, 5);
    b.i_load = -3.5;
    pb_bms_step(&m, &b, true);
    f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_OC_CHG, "充电过流 -3.5 A 连续 2 拍 -> OC_CHG");

    /* 过压去抖 3 拍 */
    pb_bms_init(&m, 3, 3, 2, 2, 5);
    b.i_load = 0.0;
    b.v_term = 4.30;
    pb_bms_step(&m, &b, true);
    pb_bms_step(&m, &b, true);
    f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_CELL_OV, "过压 4.30 V 连续 3 拍 -> CELL_OV");

    /* 欠压去抖 3 拍 */
    pb_bms_init(&m, 3, 3, 2, 2, 5);
    b.v_term = 3.00;
    pb_bms_step(&m, &b, true);
    pb_bms_step(&m, &b, true);
    f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_CELL_UV, "欠压 3.00 V 连续 3 拍 -> CELL_UV");

    /* 过温 / 低温 */
    pb_bms_init(&m, 3, 3, 2, 2, 3);
    b.v_term = 3.80; b.temp_c = 70.0;
    for (i = 0; i < 3; i++) f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_OT, "过温 70 C 连续 3 拍 -> OT");

    pb_bms_init(&m, 3, 3, 2, 2, 3);
    b.temp_c = -20.0;
    for (i = 0; i < 3; i++) f = pb_bms_step(&m, &b, true);
    CHECK(f == PB_FLT_UT, "低温 -20 C 连续 3 拍 -> UT");

    /* MOS 关断时不判定，避免自锁 */
    pb_bms_init(&m, 3, 3, 2, 2, 3);
    b.temp_c = 70.0;
    for (i = 0; i < 10; i++) f = pb_bms_step(&m, &b, false);
    CHECK(f == PB_FLT_NONE, "MOS 关断时不判定故障");

    /* 故障名 */
    CHECK(strcmp(pb_fault_name(PB_FLT_SHORT), "SHORT") == 0
          && strcmp(pb_fault_name(PB_FLT_CELL_OV), "CELL_OV") == 0,
          "故障名映射正确");
}

/* ----------------------------------------------------------------- T3 pd */
static void t3_pd(void)
{
    pd_msg_t m, rx, out;
    uint8_t buf[64];
    int n;
    pd_pdo_t caps[4], p;
    pd_port_t port;
    bool has;
    uint32_t raw;

    printf("T3 pd（USB PD 消息层与握手状态机）\n");

    /* CRC-32 已知值：poly 0x04C11DB7，init 与 xorout 均为 0xFFFFFFFF
     * （USB PD 消息层用的就是这个口径），"123456789" 的 check = 0xFC891918 */
    CHECK(pd_crc32((const uint8_t *)"123456789", 9) == 0xFC891918u,
          "CRC-32(poly 0x04C11DB7, init/xorout 0xFFFFFFFF) 自检 = 0x%08X",
          pd_crc32((const uint8_t *)"123456789", 9));

    /* Fixed PDO 打包/解析往返 */
    p.v = 9.0; p.i_max = 3.0; p.type = 0;
    raw = pd_pack_fixed_pdo(&p);
    {
        pd_pdo_t q;
        pd_unpack_fixed_pdo(raw, &q);
        CHECK(NEAR(q.v, 9.0, 1e-9) && NEAR(q.i_max, 3.0, 1e-9),
              "Fixed PDO 往返：%.2f V / %.2f A", q.v, q.i_max);
    }
    p.v = 20.0; p.i_max = 5.0;
    raw = pd_pack_fixed_pdo(&p);
    {
        pd_pdo_t q;
        pd_unpack_fixed_pdo(raw, &q);
        CHECK(NEAR(q.v, 20.0, 1e-9) && NEAR(q.i_max, 5.0, 1e-9),
              "Fixed PDO 往返（20 V/5 A）：%.2f V / %.2f A", q.v, q.i_max);
    }

    /* Request RDO */
    raw = pd_pack_request(2, 3.0, 3.0);
    CHECK(((raw >> 25) & 0x07) == 3, "RDO 档位（1-based）= %u", (unsigned)((raw >> 25) & 0x07));

    /* 消息编解码往返 + CRC 校验 */
    memset(&m, 0, sizeof(m));
    m.msg_type = PD_DATA_SOURCE_CAP;
    m.is_data = true;
    m.n_obj = 2;
    m.id = 1;
    m.spec_rev = 3;
    p.v = 5.0; p.i_max = 3.0;  m.obj[0] = pd_pack_fixed_pdo(&p);
    p.v = 9.0; p.i_max = 2.0;  m.obj[1] = pd_pack_fixed_pdo(&p);

    n = pd_encode(&m, buf, (int)sizeof(buf), PD_SOP);
    CHECK(n == 2 + 8 + 4, "编码长度 = %d（2 header + 2 PDO + 4 CRC）", n);
    CHECK(pd_decode(buf, n, &rx) == 0, "解码成功");
    CHECK(rx.msg_type == PD_DATA_SOURCE_CAP && rx.n_obj == 2 && rx.id == 1,
          "解码字段一致：type=0x%02X n_obj=%u id=%u", rx.msg_type, rx.n_obj, rx.id);
    CHECK(rx.obj[0] == m.obj[0] && rx.obj[1] == m.obj[1], "数据对象一致");

    /* CRC 破坏 -> 解码失败 */
    buf[3] ^= 0xFF;
    CHECK(pd_decode(buf, n, &rx) < 0, "CRC 被破坏 -> 解码报错");

    /* 缓冲不足 */
    CHECK(pd_encode(&m, buf, 4, PD_SOP) < 0, "缓冲区不足 -> 返回错误");

    /* 握手全流程：IDLE -> SEND_CAP -> WAIT_REQUEST -> WAIT_PS_RDY -> CONTRACT */
    caps[0].v = 5.0;  caps[0].i_max = 3.0; caps[0].type = 0;
    caps[1].v = 9.0;  caps[1].i_max = 3.0; caps[1].type = 0;
    caps[2].v = 12.0; caps[2].i_max = 2.25; caps[2].type = 0;
    caps[3].v = 20.0; caps[3].i_max = 1.5; caps[3].type = 0;
    pd_port_init(&port, caps, 4);
    CHECK(port.state == PD_ST_IDLE, "初态 = %s", pd_state_name(port.state));
    CHECK(pd_pick_pdo(&port, 9.0) == 1, "选档：要 9 V -> PDO[1]（9 V）");
    CHECK(pd_pick_pdo(&port, 15.0) == 2, "选档：要 15 V -> PDO[2]（12 V，不超）");
    CHECK(pd_pick_pdo(&port, 3.0) == 0, "选档：要 3 V -> 退到最低档 PDO[0]（5 V）");

    memset(&m, 0, sizeof(m));
    m.msg_type = PD_DATA_SINK_CAP; m.is_data = true; m.n_obj = 1;
    pd_port_rx(&port, &m, &out, &has);
    CHECK(port.state == PD_ST_WAIT_REQUEST, "收到对端消息 -> %s", pd_state_name(port.state));
    CHECK(has && out.msg_type == PD_DATA_SOURCE_CAP && out.n_obj == 4,
          "发出 Source_Capabilities（%u 档）", out.n_obj);

    /* 发一个合法 Request（要 9 V 档，3 A） */
    memset(&m, 0, sizeof(m));
    m.msg_type = PD_DATA_REQUEST; m.is_data = true; m.n_obj = 1;
    m.obj[0] = pd_pack_request(1, 3.0, 9.0);
    pd_port_rx(&port, &m, &out, &has);
    CHECK(port.state == PD_ST_WAIT_PS_RDY && has && out.msg_type == PD_CTRL_ACCEPT,
          "合法 Request -> Accept，状态 = %s", pd_state_name(port.state));

    memset(&m, 0, sizeof(m));
    m.msg_type = PD_CTRL_PS_RDY;
    pd_port_rx(&port, &m, &out, &has);
    CHECK(port.state == PD_ST_CONTRACT && port.contract_valid,
          "收到 PS_RDY -> %s，合约电压 %.1f V", pd_state_name(port.state), port.contract_v);
    CHECK(port.handshakes == 1, "成功握手计数 = %u", port.handshakes);

    /* 非法档位 -> Reject */
    pd_port_init(&port, caps, 4);
    memset(&m, 0, sizeof(m));
    m.msg_type = PD_DATA_SINK_CAP; m.is_data = true; m.n_obj = 1;
    pd_port_rx(&port, &m, &out, &has);                 /* -> WAIT_REQUEST */
    memset(&m, 0, sizeof(m));
    m.msg_type = PD_DATA_REQUEST; m.is_data = true; m.n_obj = 1;
    m.obj[0] = pd_pack_request(4, 1.0, 5.0);           /* pos=5，超过本端口 4 个档位 */
    pd_port_rx(&port, &m, &out, &has);
    CHECK(has && out.msg_type == PD_CTRL_REJECT && port.rejects == 1,
          "越界档位 -> Reject（计数 %u）", port.rejects);

    /* 电流超限 -> Reject */
    pd_port_init(&port, caps, 4);
    pd_port_rx(&port, &m, &out, &has);
    out.msg_type = 0;
    memset(&m, 0, sizeof(m));
    m.msg_type = PD_DATA_REQUEST; m.is_data = true; m.n_obj = 1;
    m.obj[0] = pd_pack_request(4, 3.0, 20.0);          /* 20 V 档最大 1.5 A，要 3 A */
    pd_port_rx(&port, &m, &out, &has);
    CHECK(has && out.msg_type == PD_CTRL_REJECT, "电流超限 -> Reject");

    /* Soft_Reset：合约作废，回到 SEND_CAP */
    pd_port_init(&port, caps, 4);
    memset(&m, 0, sizeof(m));
    m.msg_type = PD_CTRL_SOFT_RESET;
    pd_port_rx(&port, &m, &out, &has);
    CHECK(port.state == PD_ST_SEND_CAP && !port.contract_valid,
          "Soft_Reset -> %s，合约作废", pd_state_name(port.state));
}

/* --------------------------------------------------------- T4 powerbank */
static void t4_powerbank(void)
{
    pb_sys_t s;
    pb_state_t st;
    double ma;
    int i;

    printf("T4 powerbank（整机状态机与低功耗）\n");

    pb_sys_init(&s, 100, 50, 0.05, 8.0);
    CHECK(s.state == PB_ST_STANDBY, "初态 = %s", pb_state_name(s.state));

    /* 无负载 -> 累计到 sleep_after 后进休眠 */
    for (i = 0; i < 120; i++) ma = pb_sys_step(&s, 0.0, false, PB_FLT_NONE);
    CHECK(s.state == PB_ST_SLEEP, "无负载 120 tick -> %s", pb_state_name(s.state));
    CHECK(NEAR(ma, 0.05, 1e-9), "休眠静态电流 = %.2f mA", ma);

    /* 负载插入 -> 唤醒 */
    ma = pb_sys_step(&s, 2.0, false, PB_FLT_NONE);
    CHECK(s.state == PB_ST_DISCHARGING, "插负载 -> %s", pb_state_name(s.state));
    CHECK(s.wake_events == 1, "唤醒事件计数 = %u", s.wake_events);
    CHECK(NEAR(ma, 8.0, 1e-9), "放电时静态电流 = %.2f mA", ma);

    /* 输入插入 -> 充电态优先 */
    pb_sys_init(&s, 100, 50, 0.05, 8.0);
    pb_sys_step(&s, 0.0, true, PB_FLT_NONE);
    CHECK(s.state == PB_ST_CHARGING, "插输入 -> %s", pb_state_name(s.state));

    /* 故障 -> PROTECT；故障消失 -> RECOVER -> STANDBY */
    pb_sys_init(&s, 100, 50, 0.05, 8.0);
    pb_sys_step(&s, 1.0, false, PB_FLT_OC_DIS);
    CHECK(s.state == PB_ST_PROTECT, "故障 -> %s", pb_state_name(s.state));
    pb_sys_step(&s, 1.0, false, PB_FLT_NONE);
    CHECK(s.state == PB_ST_RECOVER, "故障消失 -> %s", pb_state_name(s.state));
    for (i = 0; i < 60; i++) pb_sys_step(&s, 1.0, false, PB_FLT_NONE);
    CHECK(s.state == PB_ST_DISCHARGING || s.state == PB_ST_STANDBY,
          "恢复等待结束 -> %s（自动重试）", pb_state_name(s.state));

    /* 目标状态推荐函数 */
    st = pb_sys_next_state(PB_ST_STANDBY, 3.0, false, PB_FLT_NONE, 0, 100);
    CHECK(st == PB_ST_DISCHARGING, "推荐：有负载 -> %s", pb_state_name(st));
    st = pb_sys_next_state(PB_ST_STANDBY, 0.0, true, PB_FLT_NONE, 0, 100);
    CHECK(st == PB_ST_CHARGING, "推荐：有输入 -> %s", pb_state_name(st));
    st = pb_sys_next_state(PB_ST_STANDBY, 0.0, false, PB_FLT_NONE, 200, 100);
    CHECK(st == PB_ST_SLEEP, "推荐：长时间无负载 -> %s", pb_state_name(st));
}

/* ------------------------------------------------------------- T5 整机 */
static void t5_device(void)
{
    pb_batt_cfg_t cfg;
    pb_dev_t dev;
    pd_pdo_t caps[3];
    int i, saw_disch = 0, saw_protect = 0;

    printf("T5 device（整机联动：放电到保护）\n");
    cfg_basic(&cfg);
    caps[0].v = 5.0;  caps[0].i_max = 3.0;  caps[0].type = 0;
    caps[1].v = 9.0;  caps[1].i_max = 3.0;  caps[1].type = 0;
    caps[2].v = 12.0; caps[2].i_max = 1.5;  caps[2].type = 0;

    pb_dev_init(&dev, &cfg, 0.30, caps, 3, 100, 50);
    CHECK(dev.batt.soc > 0.29 && dev.batt.soc < 0.31, "初始 SOC = %.3f", dev.batt.soc);

    /* 以 4 A 放电（未到 5 A 过流阈值），SOC 会持续下降直到欠压保护 */
    pb_dev_set_load(&dev, 4.0);
    for (i = 0; i < 600000; i++) {           /* 600 s 仿真 */
        pb_dev_step(&dev);
        if (dev.sys.state == PB_ST_DISCHARGING) saw_disch = 1;
        if (dev.sys.state == PB_ST_PROTECT) { saw_protect = 1; break; }
    }
    CHECK(saw_disch, "进入放电态");
    CHECK(saw_protect, "放电到欠压 -> 触发保护（SOC = %.3f，端压 = %.3f V，故障 = %s）",
          dev.batt.soc, dev.batt.v_term, pb_fault_name(dev.bms.last));
    /* 保护触发后 MOS 关断，端压会从「含内阻压降的放电端压」回升到 OCV，
     * 所以这里断言的是：故障类型正确 + 关断后端压回升到阈值以上 */
    CHECK(dev.bms.last == PB_FLT_CELL_UV, "故障类型 = %s", pb_fault_name(dev.bms.last));
    CHECK(dev.batt.v_term > cfg.v_uv,
          "MOS 关断后端压回升到 %.3f V（高于阈值 %.2f V，保护前含内阻压降低于阈值）",
          dev.batt.v_term, cfg.v_uv);

    /* 插上输入后应能充电并缓慢恢复 */
    {
        double soc_before = dev.batt.soc;
        pb_dev_set_load(&dev, 0.0);
        pb_dev_plug_in(&dev, true);
        pb_bms_clear(&dev.bms);
        for (i = 0; i < 200000; i++) pb_dev_step(&dev);
        CHECK(dev.batt.soc > soc_before,
              "插输入后 SOC 回升：%.4f -> %.4f（状态 %s）",
              soc_before, dev.batt.soc, pb_state_name(dev.sys.state));
    }
}

int main(void)
{
    printf("==== usb-pd-powerbank-lab 单元测试 ====\n\n");
    t1_battery();
    t2_bms();
    t3_pd();
    t4_powerbank();
    t5_device();
    printf("\n==== 结果：%d passed, %d failed ====\n", passed, failed);
    return failed ? 1 : 0;
}
