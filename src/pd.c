/* pd.c — USB PD 消息层与策略状态机
 *
 * 实现范围（**不含 BMC 物理层与 4b5b 编码**，那是 PHY 芯片的事）：
 *   1) 消息层：16 位 Header（Message Type / Num Data Objects / Message ID / Spec Revision）
 *      + 数据对象 + CRC-32 编解码
 *   2) 数据对象：Fixed PDO（电压 50 mV / 电流 10 mA 单位）、Request RDO（档位 + 工作电流）
 *   3) 策略状态机：Source 侧握手流程
 *        IDLE -> SEND_CAP -> WAIT_REQUEST -> WAIT_PS_RDY -> CONTRACT
 *      含 Reject（档位不合法）与 Soft_Reset（回到 SEND_CAP）
 *
 * 设计取舍：这里做的是「协议逻辑与握手时序」，上板时把收发替换成 PHY 芯片的
 * BMC 收发 + 中断即可；CRC 与状态机可原样复用。
 */
#include "pb.h"
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------ CRC-32 */
/* PD 消息层 CRC：poly 0x04C11DB7，init 0xFFFFFFFF，MSB-first，无反射，末取反 */
uint32_t pd_crc32(const uint8_t *data, int len)
{
    uint32_t crc = 0xFFFFFFFFu;
    int i, b;
    for (i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i]) << 24;
        for (b = 0; b < 8; b++) {
            if (crc & 0x80000000u) crc = (crc << 1) ^ 0x04C11DB7u;
            else                   crc = (crc << 1);
        }
    }
    return ~crc;
}

/* ------------------------------------------------------------- 消息层编解 */
int pd_encode(const pd_msg_t *m, uint8_t *buf, int buf_len, pd_sop_t sop)
{
    int need = 2 + (m->is_data ? m->n_obj * 4 : 0) + 4;
    uint16_t hdr;
    int i, off;

    (void)sop;                       /* SOP 类型由 PHY 层决定，这里仅编码消息体 */
    if (buf_len < need) return -1;

    hdr  = (uint16_t)(m->msg_type & 0x1F);
    hdr |= (uint16_t)((m->n_obj & 0x07) << 6);
    hdr |= (uint16_t)((m->id & 0x07) << 9);
    hdr |= (uint16_t)((m->spec_rev & 0x0F) << 12);

    buf[0] = (uint8_t)(hdr & 0xFF);
    buf[1] = (uint8_t)(hdr >> 8);

    off = 2;
    for (i = 0; i < m->n_obj; i++) {
        buf[off++] = (uint8_t)(m->obj[i] & 0xFF);
        buf[off++] = (uint8_t)((m->obj[i] >> 8) & 0xFF);
        buf[off++] = (uint8_t)((m->obj[i] >> 16) & 0xFF);
        buf[off++] = (uint8_t)((m->obj[i] >> 24) & 0xFF);
    }

    {
        uint32_t crc = pd_crc32(buf, off);
        buf[off++] = (uint8_t)(crc & 0xFF);
        buf[off++] = (uint8_t)((crc >> 8) & 0xFF);
        buf[off++] = (uint8_t)((crc >> 16) & 0xFF);
        buf[off++] = (uint8_t)((crc >> 24) & 0xFF);
    }
    return off;
}

int pd_decode(const uint8_t *buf, int len, pd_msg_t *m)
{
    uint16_t hdr;
    int i, off, nobj;
    uint32_t crc_rx, crc_calc;

    if (len < 6) return -1;

    hdr = (uint16_t)(buf[0] | (buf[1] << 8));
    nobj = (hdr >> 6) & 0x07;
    if (len < 2 + nobj * 4 + 4) return -1;

    crc_rx = (uint32_t)buf[len - 4] | ((uint32_t)buf[len - 3] << 8)
           | ((uint32_t)buf[len - 2] << 16) | ((uint32_t)buf[len - 1] << 24);
    crc_calc = pd_crc32(buf, len - 4);
    if (crc_rx != crc_calc) return -2;             /* CRC 校验失败 */

    memset(m, 0, sizeof(*m));
    m->msg_type = (uint8_t)(hdr & 0x1F);
    m->n_obj    = (uint8_t)nobj;
    m->id       = (uint8_t)((hdr >> 9) & 0x07);
    m->spec_rev = (uint8_t)((hdr >> 12) & 0x0F);
    m->is_data  = (nobj > 0);

    off = 2;
    for (i = 0; i < nobj && i < PB_PD_MAX_PDO; i++) {
        m->obj[i] = (uint32_t)buf[off] | ((uint32_t)buf[off + 1] << 8)
                  | ((uint32_t)buf[off + 2] << 16) | ((uint32_t)buf[off + 3] << 24);
        off += 4;
    }
    return 0;
}

/* ------------------------------------------------------------ PDO / RDO */
uint32_t pd_pack_fixed_pdo(const pd_pdo_t *p)
{
    uint32_t v = (uint32_t)(p->v / 0.05 + 0.5) & 0x3FF;      /* 50 mV 单位 */
    uint32_t i = (uint32_t)(p->i_max / 0.01 + 0.5) & 0x3FF;  /* 10 mA 单位 */
    return (v << 20) | (i << 10);                            /* 高 2 位 00 = Fixed */
}

void pd_unpack_fixed_pdo(uint32_t raw, pd_pdo_t *p)
{
    p->type  = (uint8_t)(raw >> 30);
    p->v     = ((raw >> 20) & 0x3FF) * 0.05;
    p->i_max = ((raw >> 10) & 0x3FF) * 0.01;
}

uint32_t pd_pack_request(int pdo_index, double current, double voltage)
{
    uint32_t pos, op, mx;

    /* RDO 的对象位置字段只有 3 bit（1..7，1-based）。
     * 超出范围时回绕会产生"看起来合法"的请求，这里显式夹到边界而不是静默回绕。 */
    if (pdo_index < 0)   pdo_index = 0;
    if (pdo_index > 6)   pdo_index = 6;
    pos = (uint32_t)(pdo_index + 1) & 0x07;

    op = (uint32_t)(current / 0.01 + 0.5);
    mx = (uint32_t)(voltage / 0.01 + 0.5);
    if (op > 0x3FF) op = 0x3FF;
    if (mx > 0x3FF) mx = 0x3FF;
    return (pos << 25) | (op << 10) | mx;
}

/* ---------------------------------------------------------- 策略状态机 */
static const char *PD_ST_NAME[PD_ST_COUNT] = {
    "IDLE", "SEND_CAP", "WAIT_REQUEST", "WAIT_PS_RDY", "CONTRACT",
    "SOFT_RESET", "ERROR"
};

const char *pd_state_name(pd_state_t s)
{
    if ((int)s < 0 || s >= PD_ST_COUNT) return "?";
    return PD_ST_NAME[s];
}

void pd_port_init(pd_port_t *p, const pd_pdo_t *caps, int n_caps)
{
    int i;
    memset(p, 0, sizeof(*p));
    p->state = PD_ST_IDLE;
    p->prev = PD_ST_IDLE;
    if (n_caps > PB_PD_MAX_PDO) n_caps = PB_PD_MAX_PDO;
    p->n_caps = n_caps;
    for (i = 0; i < n_caps; i++) p->src_caps[i] = caps[i];
}

int pd_pick_pdo(const pd_port_t *p, double want_v)
{
    int i, best = -1;
    double best_v = 0.0;
    for (i = 0; i < p->n_caps; i++) {
        /* 选不超过需求电压里最高的那一档；都不满足则退回最低档 */
        if (p->src_caps[i].v <= want_v + 1e-6 && p->src_caps[i].v > best_v) {
            best_v = p->src_caps[i].v;
            best = i;
        }
    }
    if (best < 0 && p->n_caps > 0) {
        best = 0; best_v = p->src_caps[0].v;
        for (i = 1; i < p->n_caps; i++)
            if (p->src_caps[i].v < best_v) { best_v = p->src_caps[i].v; best = i; }
    }
    return best;
}

static void emit(pd_msg_t *out, bool *has_out, uint8_t type, bool is_data, uint8_t id)
{
    memset(out, 0, sizeof(*out));
    out->msg_type = type;
    out->is_data = is_data;
    out->n_obj = 0;
    out->id = id;
    out->spec_rev = 3;      /* PD 3.0 */
    *has_out = true;
}

void pd_port_rx(pd_port_t *p, const pd_msg_t *in, pd_msg_t *out, bool *has_out)
{
    p->prev = p->state;
    p->tick++;
    *has_out = false;

    /* Soft_Reset 在任何状态都要处理 */
    if (in->msg_type == PD_CTRL_SOFT_RESET && !in->is_data) {
        p->state = PD_ST_SEND_CAP;
        p->contract_valid = false;
        return;
    }

    switch (p->state) {
    case PD_ST_IDLE:
        /* 收到对端能力或任意消息 -> 立即发送自己的能力并转入等请求
         * （真机上是 PHY 检测到连接后触发，这里用"收到首个消息"替代） */
        emit(out, has_out, PD_DATA_SOURCE_CAP, true, p->msg_id++);
        out->n_obj = (uint8_t)p->n_caps;
        {
            int i;
            for (i = 0; i < p->n_caps; i++)
                out->obj[i] = pd_pack_fixed_pdo(&p->src_caps[i]);
        }
        p->state = PD_ST_WAIT_REQUEST;
        break;

    case PD_ST_SEND_CAP:
        emit(out, has_out, PD_DATA_SOURCE_CAP, true, p->msg_id++);
        out->n_obj = (uint8_t)p->n_caps;
        {
            int i;
            for (i = 0; i < p->n_caps; i++)
                out->obj[i] = pd_pack_fixed_pdo(&p->src_caps[i]);
        }
        p->state = PD_ST_WAIT_REQUEST;
        break;

    case PD_ST_WAIT_REQUEST:
        if (in->is_data && in->msg_type == PD_DATA_REQUEST && in->n_obj >= 1) {
            uint32_t rdo = in->obj[0];
            int pos = (int)((rdo >> 25) & 0x07);
            double i_req = ((rdo >> 10) & 0x3FF) * 0.01;
            if (pos >= 1 && pos <= p->n_caps
                && i_req <= p->src_caps[pos - 1].i_max + 1e-6) {
                p->active_pdo = pos - 1;
                p->contract_v = p->src_caps[pos - 1].v;
                p->contract_i = i_req;
                emit(out, has_out, PD_CTRL_ACCEPT, false, p->msg_id++);
                p->state = PD_ST_WAIT_PS_RDY;
            } else {
                /* 档位越界或电流超限 -> Reject，回到等请求 */
                p->rejects++;
                emit(out, has_out, PD_CTRL_REJECT, false, p->msg_id++);
                p->state = PD_ST_WAIT_REQUEST;
            }
        }
        break;

    case PD_ST_WAIT_PS_RDY:
        if (in->msg_type == PD_CTRL_PS_RDY && !in->is_data) {
            p->state = PD_ST_CONTRACT;
            p->contract_valid = true;
            p->handshakes++;
        } else if (in->msg_type == PD_CTRL_REJECT && !in->is_data) {
            p->state = PD_ST_WAIT_REQUEST;   /* 对端拒绝 -> 等新请求 */
            p->contract_valid = false;
        }
        break;

    case PD_ST_CONTRACT:
        /* 已建立合约，收到新的 Request 则重新协商 */
        if (in->is_data && in->msg_type == PD_DATA_REQUEST) {
            p->state = PD_ST_WAIT_REQUEST;
            p->contract_valid = false;
        }
        break;

    case PD_ST_SOFT_RESET:
        p->state = PD_ST_SEND_CAP;
        break;

    default:
        p->state = PD_ST_ERROR;
        break;
    }
}
