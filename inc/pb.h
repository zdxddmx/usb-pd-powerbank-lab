/*
 * pb.h — Power-bank firmware lab: USB-PD negotiation + Li-ion BMS + low power
 *
 * 面向「快充移动电源 / 无线充电宝」固件方向的纯 C 仿真与实践工程。
 *
 * 分层（对齐真实充电宝固件的主控分层）：
 *   battery    电池模型：库仑计数 SOC + OCV 查表 + 内阻 + 温升
 *   bms        保护：过压 / 欠压 / 过流 / 过温 / 短路，带去抖与恢复
 *   pd         USB PD 消息层（Header / PDO / RDO / CRC32）+ 策略状态机（握手）
 *   powerbank  整机状态机：待机 / 充电 / 放电 / 保护 / 低功耗，含休眠唤醒
 *
 * 纯 C99，无外部依赖，可在 PC 上单元测试；bms / pd / powerbank 三层
 * 不含平台相关代码，可原样搬到 MCU（杰理 / 芯海 / 中科蓝讯 等）。
 */
#ifndef PB_H
#define PB_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================ 电池与 SOC */
typedef struct {
    double capacity_ah;      /* 额定容量 Ah            */
    double v_full;           /* 满充电压 V（4.20）      */
    double v_nominal;        /* 标称电压 V（3.70）      */
    double v_empty;          /* 放空电压 V（3.00）      */
    double r_internal;       /* 内阻 Ohm               */
    double coulomb_eff;      /* 库仑效率（充电）        */
    /* 保护阈值 */
    double v_ov;             /* 过压阈值 V             */
    double v_uv;             /* 欠压阈值 V             */
    double i_oc_chg;         /* 充电过流 A             */
    double i_oc_dis;         /* 放电过流 A             */
    double i_sc;             /* 短路电流阈值 A          */
    double t_ot;             /* 过温阈值 C             */
    double t_ut;             /* 低温阈值 C             */
} pb_batt_cfg_t;

typedef struct {
    pb_batt_cfg_t cfg;
    double soc;              /* 0..1                   */
    double v_term;           /* 端电压 V（含内阻压降）    */
    double i_load;           /* 当前电流 A（正=放电）     */
    double temp_c;           /* 温度 C                 */
    double charge_ah;        /* 累计充入 Ah             */
    double discharge_ah;     /* 累计放出 Ah             */
    double t;                /* 仿真时间 s              */
} pb_batt_t;

void   pb_batt_init(pb_batt_t *b, const pb_batt_cfg_t *cfg, double soc0);
/* OCV-SOC 查表（分段线性）+ 内阻压降，返回端电压 */
double pb_batt_ocv(double soc);
double pb_batt_update(pb_batt_t *b, double i_load, double dt);
void   pb_batt_reset_soc(pb_batt_t *b, double soc);

/* ================================================================ BMS 保护 */
typedef enum {
    PB_FLT_NONE = 0,
    PB_FLT_CELL_OV,      /* 过压   */
    PB_FLT_CELL_UV,      /* 欠压   */
    PB_FLT_OC_CHG,       /* 充电过流 */
    PB_FLT_OC_DIS,       /* 放电过流 */
    PB_FLT_SHORT,        /* 短路   */
    PB_FLT_OT,           /* 过温   */
    PB_FLT_UT,           /* 低温   */
    PB_FLT_COUNT
} pb_fault_t;

typedef struct {
    uint8_t  debounce[PB_FLT_COUNT];   /* 去抖阈值（连续 tick） */
    uint8_t  cnt[PB_FLT_COUNT];        /* 当前计数             */
    bool     active[PB_FLT_COUNT];     /* 锁存状态             */
    pb_fault_t last;                   /* 最近一次触发的故障     */
    uint32_t total_trips;
} pb_bms_t;

const char *pb_fault_name(pb_fault_t f);
void  pb_bms_init(pb_bms_t *m, uint8_t deb_ov, uint8_t deb_uv,
                  uint8_t deb_oc, uint8_t deb_sc, uint8_t deb_t);
/* 返回当前生效的最高优先级故障（无故障返回 PB_FLT_NONE） */
pb_fault_t pb_bms_step(pb_bms_t *m, const pb_batt_t *b, bool mos_enabled);
void  pb_bms_clear(pb_bms_t *m);

/* ============================================================ USB PD 协议 */
#define PB_PD_MAX_PDO 7
#define PB_PD_MSG_MAX 34          /* 消息层最大字节数（含 header） */

typedef enum {
    PD_SOP      = 0,
    PD_SOP_PRIME,
    PD_SOP_DPRIME
} pd_sop_t;

typedef enum {
    PD_CTRL_GOOD_CRC    = 0x01,
    PD_CTRL_ACCEPT      = 0x03,
    PD_CTRL_REJECT      = 0x04,
    PD_CTRL_PS_RDY      = 0x06,
    PD_CTRL_SOFT_RESET  = 0x0D
} pd_ctrl_t;

typedef enum {
    PD_DATA_SOURCE_CAP  = 0x01,
    PD_DATA_REQUEST     = 0x02,
    PD_DATA_SINK_CAP    = 0x03
} pd_data_t;

typedef enum {
    PD_ST_IDLE = 0,
    PD_ST_SEND_CAP,
    PD_ST_WAIT_REQUEST,
    PD_ST_WAIT_PS_RDY,
    PD_ST_CONTRACT,        /* 握手完成，供电中 */
    PD_ST_SOFT_RESET,
    PD_ST_ERROR,
    PD_ST_COUNT
} pd_state_t;

typedef struct {
    double v;              /* 电压 V      */
    double i_max;          /* 最大电流 A   */
    uint8_t type;          /* 0=fixed     */
} pd_pdo_t;

typedef struct {
    uint8_t  msg_type;     /* pd_ctrl_t 或 pd_data_t */
    uint8_t  n_obj;        /* 数据对象个数 */
    uint8_t  id;           /* 消息 ID      */
    uint8_t  spec_rev;
    uint32_t obj[PB_PD_MAX_PDO];
    bool     is_data;
} pd_msg_t;

/* 消息层：编解码（不含 BMC 物理层） */
int  pd_encode(const pd_msg_t *m, uint8_t *buf, int buf_len, pd_sop_t sop);
int  pd_decode(const uint8_t *buf, int len, pd_msg_t *m);
uint32_t pd_crc32(const uint8_t *data, int len);

/* PDO / RDO 打包与解析 */
uint32_t pd_pack_fixed_pdo(const pd_pdo_t *p);
void     pd_unpack_fixed_pdo(uint32_t raw, pd_pdo_t *p);
uint32_t pd_pack_request(int pdo_index, double current, double voltage);

typedef struct {
    pd_state_t state;
    pd_state_t prev;
    pd_pdo_t   src_caps[PB_PD_MAX_PDO];
    int        n_caps;
    int        active_pdo;      /* 已协商的 PDO 索引 */
    double     contract_v;
    double     contract_i;
    uint8_t    msg_id;
    uint32_t   tick;
    uint32_t   handshakes;      /* 成功握手次数 */
    uint32_t   rejects;
    bool       contract_valid;
} pd_port_t;

const char *pd_state_name(pd_state_t s);
void pd_port_init(pd_port_t *p, const pd_pdo_t *caps, int n_caps);
/* 接收一个消息并推进状态机；out 非空时把要发送的响应写进去 */
void pd_port_rx(pd_port_t *p, const pd_msg_t *in, pd_msg_t *out, bool *has_out);
/* 推荐一个 PDO 索引（按需求电压选最接近且不超的档位） */
int  pd_pick_pdo(const pd_port_t *p, double want_v);

/* ============================================== 整机状态机与低功耗调度 */
typedef enum {
    PB_ST_SLEEP = 0,       /* 低功耗休眠 */
    PB_ST_STANDBY,         /* 待机      */
    PB_ST_CHARGING,        /* 自身充电   */
    PB_ST_DISCHARGING,     /* 对外放电（快充） */
    PB_ST_PROTECT,         /* 保护锁死   */
    PB_ST_RECOVER,         /* 故障恢复等待 */
    PB_ST_COUNT
} pb_state_t;

typedef struct {
    pb_state_t state;
    pb_state_t prev;
    uint32_t   ticks_in_state;
    uint32_t   idle_ticks;          /* 无负载累计 tick */
    uint32_t   sleep_after_ticks;   /* 多久无负载进休眠 */
    uint32_t   recover_ticks;
    uint32_t   wake_events;
    double     sleep_current_ma;    /* 休眠电流 mA */
    double     standby_current_ma;  /* 待机电流 mA */
} pb_sys_t;

const char *pb_state_name(pb_state_t s);
void pb_sys_init(pb_sys_t *s, uint32_t sleep_after_ticks, uint32_t recover_ticks,
                 double sleep_ma, double standby_ma);
/* 负载电流 A、是否有输入、BMS 故障 -> 推进一 tick，返回本 tick 的整机静态电流 mA */
double pb_sys_step(pb_sys_t *s, double load_a, bool vin_present, pb_fault_t flt);
/* 按负载/输入/故障推荐目标状态（供测试与说明用） */
pb_state_t pb_sys_next_state(pb_state_t cur, double load_a, bool vin_present,
                             pb_fault_t flt, uint32_t idle_ticks,
                             uint32_t sleep_after);

/* ================================================== 整机（电池 + BMS + PD） */
typedef struct {
    pb_batt_t  batt;
    pb_bms_t   bms;
    pd_port_t  pd;
    pb_sys_t   sys;
    double     load_a;         /* 输出负载 A */
    double     vin_a;          /* 输入充电电流 A */
    bool       vin_present;
    double     static_ma;      /* 本 tick 静态电流 */
    uint32_t   t_ms;
} pb_dev_t;

void pb_dev_init(pb_dev_t *d, const pb_batt_cfg_t *bcfg, double soc0,
                 const pd_pdo_t *caps, int n_caps,
                 uint32_t sleep_after_ticks, uint32_t recover_ticks);
void pb_dev_set_load(pb_dev_t *d, double load_a);
void pb_dev_plug_in(pb_dev_t *d, bool present);
/* 推进一个 tick（默认 1 ms） */
void pb_dev_step(pb_dev_t *d);

#endif /* PB_H */
