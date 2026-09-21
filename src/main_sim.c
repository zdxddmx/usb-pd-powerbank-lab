/* main_sim.c — 仿真主程序：三个场景输出 CSV
 *   discharge  4 A 放电到欠压保护 -> 插输入充电恢复
 *   charge     从低 SOC 恒流充电到过压保护
 *   sleep      无负载进休眠 -> 负载插入唤醒（含 PD 握手）
 */
#include "pb.h"
#include <stdio.h>
#include <string.h>

static void caps_default(pd_pdo_t *c)
{
    c[0].v = 5.0;  c[0].i_max = 3.0;  c[0].type = 0;
    c[1].v = 9.0;  c[1].i_max = 3.0;  c[1].type = 0;
    c[2].v = 12.0; c[2].i_max = 2.25; c[2].type = 0;
    c[3].v = 20.0; c[3].i_max = 1.5;  c[3].type = 0;
}

static void cfg_default(pb_batt_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->capacity_ah = 2.0; c->v_full = 4.20; c->v_nominal = 3.70; c->v_empty = 3.00;
    c->r_internal = 0.06; c->coulomb_eff = 0.99;
    c->v_ov = 4.25; c->v_uv = 3.10;
    c->i_oc_chg = 3.0; c->i_oc_dis = 5.0; c->i_sc = 8.0;
    c->t_ot = 60.0; c->t_ut = -10.0;
}

int main(int argc, char **argv)
{
    pb_batt_cfg_t cfg;
    pd_pdo_t caps[4];
    pb_dev_t dev;
    const char *scen = (argc > 1) ? argv[1] : "discharge";
    int i, total;
    pd_msg_t msg, out;
    bool has;

    cfg_default(&cfg);
    caps_default(caps);

    if (strcmp(scen, "charge") == 0) {
        pb_dev_init(&dev, &cfg, 0.10, caps, 4, 100000, 50);
        pb_dev_plug_in(&dev, true);
        total = 400000;                     /* 400 s */
        printf("t_ms,state,soc,v_term,i_load,temp_c,fault\n");
        for (i = 0; i < total; i++) {
            pb_dev_step(&dev);
            if (i % 100 == 0)
                printf("%u,%s,%.5f,%.4f,%.4f,%.3f,%s\n",
                       dev.t_ms, pb_state_name(dev.sys.state), dev.batt.soc,
                       dev.batt.v_term, dev.batt.i_load, dev.batt.temp_c,
                       pb_fault_name(dev.bms.last));
        }
        fprintf(stderr, "[charge] 末态 %s · SOC %.4f · 端压 %.4f V · 故障 %s\n",
                pb_state_name(dev.sys.state), dev.batt.soc, dev.batt.v_term,
                pb_fault_name(dev.bms.last));
        return 0;
    }

    if (strcmp(scen, "sleep") == 0) {
        pb_dev_init(&dev, &cfg, 0.60, caps, 4, 300, 50);   /* 300 ms 无负载进休眠 */
        total = 1200;
        printf("t_ms,state,soc,v_term,i_load,static_ma,wake_events\n");
        for (i = 0; i < total; i++) {
            if (i == 800) pb_dev_set_load(&dev, 2.0);      /* 800 ms 插入负载 */
            pb_dev_step(&dev);
            if (i % 20 == 0)
                printf("%u,%s,%.5f,%.4f,%.4f,%.3f,%u\n",
                       dev.t_ms, pb_state_name(dev.sys.state), dev.batt.soc,
                       dev.batt.v_term, dev.batt.i_load, dev.static_ma,
                       dev.sys.wake_events);
        }
        fprintf(stderr, "[sleep] 末态 %s · 唤醒次数 %u · 静态电流 %.2f mA\n",
                pb_state_name(dev.sys.state), dev.sys.wake_events, dev.static_ma);
        return 0;
    }

    if (strcmp(scen, "discharge") != 0) {
        fprintf(stderr, "unknown scenario: %s\n", scen);
        return 2;
    }

    /* ---- discharge：放电到保护 -> 插输入恢复 ---- */
    pb_dev_init(&dev, &cfg, 0.40, caps, 4, 100000, 50);
    pb_dev_set_load(&dev, 4.0);

    /* 先演示一次完整 PD 握手 */
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = PD_DATA_SINK_CAP; msg.is_data = true; msg.n_obj = 1;
    pd_port_rx(&dev.pd, &msg, &out, &has);
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = PD_DATA_REQUEST; msg.is_data = true; msg.n_obj = 1;
    msg.obj[0] = pd_pack_request(2, 2.0, 9.0);              /* 要 9 V 档 */
    pd_port_rx(&dev.pd, &msg, &out, &has);
    memset(&msg, 0, sizeof(msg));
    msg.msg_type = PD_CTRL_PS_RDY;
    pd_port_rx(&dev.pd, &msg, &out, &has);
    fprintf(stderr, "[pd] 握手结果：%s · 合约 %.1f V / %.2f A · 成功 %u 次\n",
            pd_state_name(dev.pd.state), dev.pd.contract_v, dev.pd.contract_i,
            dev.pd.handshakes);

    total = 1400000;   /* 1400 s（4 A 放电从头放到欠压保护），每 200 ms 打一行 */
    printf("t_ms,state,soc,v_term,i_load,temp_c,fault,pd_state,contract_v\n");
    for (i = 0; i < total; i++) {
        pb_dev_step(&dev);
        if (i % 200 == 0)
            printf("%u,%s,%.5f,%.4f,%.4f,%.3f,%s,%s,%.2f\n",
                   dev.t_ms, pb_state_name(dev.sys.state), dev.batt.soc,
                   dev.batt.v_term, dev.batt.i_load, dev.batt.temp_c,
                   pb_fault_name(dev.bms.last), pd_state_name(dev.pd.state),
                   dev.pd.contract_v);
        if (dev.sys.state == PB_ST_PROTECT) break;
    }
    fprintf(stderr, "[discharge] 保护触发：%s · SOC %.4f · 端压 %.4f V（用时 %.1f s）\n",
            pb_fault_name(dev.bms.last), dev.batt.soc, dev.batt.v_term, dev.t_ms / 1000.0);

    /* 插输入 -> 充电恢复 */
    pb_dev_set_load(&dev, 0.0);
    pb_dev_plug_in(&dev, true);
    pb_bms_clear(&dev.bms);
    for (i = 0; i < 300000; i++) pb_dev_step(&dev);
    fprintf(stderr, "[recover] 插输入后：%s · SOC %.4f · 端压 %.4f V\n",
            pb_state_name(dev.sys.state), dev.batt.soc, dev.batt.v_term);
    return 0;
}
