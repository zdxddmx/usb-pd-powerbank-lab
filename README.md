# usb-pd-powerbank-lab

> 快充移动电源 / 无线充电宝固件方向的**纯 C99 仿真与实践工程** —— 零依赖、可单元测试、可在 PC 上复现
>
> 把一台充电宝的主控固件从**协议层**一路做到**电池保护**与**低功耗调度**：
> **USB PD 握手 · 锂电池 BMS 保护 · SOC 估算 · 整机状态机 · 休眠唤醒**

[![ci](https://github.com/meminehobe24435-cmyk/usb-pd-powerbank-lab/actions/workflows/ci.yml/badge.svg)](https://github.com/meminehobe24435-cmyk/usb-pd-powerbank-lab/actions/workflows/ci.yml)
![language](https://img.shields.io/badge/language-C99-blue)
![deps](https://img.shields.io/badge/dependencies-none-green)
![tests](https://img.shields.io/badge/tests-61%20passed-brightgreen)

---

## 1. 为什么做这个

充电宝固件的难点不在"写一个状态机"，而在**四件事同时成立**：

| 难点 | 本项目怎么处理 |
|---|---|
| **快充协议要能握手** | 实现 **USB PD 消息层**（Header / Fixed PDO / Request RDO / CRC-32）+ **Source 侧策略状态机**，含 Reject 与 Soft_Reset 分支 |
| **电池不能出事** | **七类保护**：过压 / 欠压 / 充电过流 / 放电过流 / **短路** / 过温 / 低温，每类带**独立去抖**，短路**锁存** |
| **电量要报得准** | **OCV-SOC 查表（分段线性插值）+ 库仑计数 + 内阻压降**三件套，端电压随负载动态修正 |
| **待机要省电** | 整机状态机含 **SLEEP**，无负载累计到阈值即休眠（静态电流降到 μA 级），负载插入/输入接入即**唤醒** |

---

## 2. 分层结构（对齐真实充电宝固件）

```
inc/pb.h              统一公共接口
src/battery.c         电池模型：OCV-SOC 表 + 库仑计数 + 内阻 + 一阶热模型
src/bms.c             保护层：七类保护 + 去抖 + 短路锁存
src/pd.c              USB PD：CRC-32 + 消息编解码 + PDO/RDO + 策略状态机
src/powerbank.c       整机状态机（休眠/待机/充电/放电/保护/恢复）+ 低功耗调度
src/main_sim.c        三个场景仿真（discharge / charge / sleep）→ CSV
test/test_pb.c        61 项单元测试
tools/plot.py         CSV 波形绘图
.github/workflows/ci.yml  Ubuntu / Windows / macOS 三平台 CI
```

> `bms.c` / `pd.c` / `powerbank.c` 三层**不含任何平台相关代码**，
> 上板时把"读 ADC + 写 MOS 控制脚 + PHY 收发 BMC"替换掉即可原样复用。

---

## 3. USB PD 实现范围（**先说边界**）

| 实现了 | 没实现 |
|---|---|
| 消息层：16 位 Header（Message Type / N Data Objects / Message ID / Spec Revision） | **BMC 物理层编码（4b5b、前导码、SOP 序列）** —— 真机上由 PHY 芯片负责 |
| 数据对象：**Fixed PDO**（电压 50 mV / 电流 10 mA 单位）、**Request RDO**（档位 + 工作电流） | PPS / AVS / 电池类 PDO |
| 校验：**CRC-32**（poly `0x04C11DB7`，init 与 xorout 均 `0xFFFFFFFF`） | E-Marker 线缆、Discover Identity |
| 策略状态机：`IDLE → SEND_CAP → WAIT_REQUEST → WAIT_PS_RDY → CONTRACT` | Sink 侧策略、Alternate Mode |
| 异常分支：档位越界 / 电流超限 → **Reject**；**Soft_Reset** → 回到 SEND_CAP | 硬复位、错误恢复计时器 |

> 也就是说：**做的是协议逻辑与时序，不是模拟前端。** 面试时不要把它说成"完整实现 PD 协议栈"。

---

## 4. 电池与保护参数（典型 2 Ah 电芯）

| 项 | 值 |
|---|---|
| 额定容量 | 2.0 Ah |
| 满充 / 标称 / 放空 | 4.20 / 3.70 / 3.00 V |
| 内阻 | 60 mΩ |
| 库仑效率 | 0.99 |
| 过压 / 欠压阈值 | 4.25 / 3.10 V |
| 充电过流 / 放电过流 / 短路 | 3.0 / 5.0 / 8.0 A |
| 过温 / 低温 | 60 / −10 °C |
| 去抖 | 过压欠压 3 拍 · 过流 2 拍 · **短路 2 拍且锁存** · 温度 5 拍 |

---

## 5. 怎么跑

```bash
make test     # 编译并运行 61 项单元测试
make sim      # 跑三个场景，CSV 输出到 out/
python tools/plot.py       # 画波形 -> out/waveforms.png（需 matplotlib）
```

Windows 上没有 `make` 时：

```bat
gcc -std=c99 -O2 -Wall -Wextra -Wpedantic -Werror -Iinc src/battery.c src/bms.c src/pd.c src/powerbank.c test/test_pb.c -o build\pb_test.exe -lm
build\pb_test.exe
```

---

## 6. 实测结果（可复现）

### 单元测试

```
==== 结果：61 passed, 0 failed ====
```

覆盖：OCV 查表与插值 · 库仑计数与 SOC 边界 · 端电压内阻压降 ·
七类保护去抖与短路锁存 · MOS 关断不判定 · CRC-32 自检 · 消息编解码往返与 CRC 破坏检测 ·
PDO 往返 · 选档策略 · **PD 握手全流程** · Reject 分支 · Soft_Reset ·
整机状态迁移 · 休眠与唤醒 · 保护与自动恢复 · 整机放电到保护

### 三个场景

| 场景 | 实测输出 |
|---|---|
| `discharge` | PD 握手 → **CONTRACT · 合约 12.0 V / 2.00 A**；4 A 放电到 **CELL_UV 保护**（用时 **650.6 s**，SOC 0.0378，端压 3.3400 V）；插输入后 **CHARGING 恢复**（SOC 0.0585） |
| `charge` | 0.5 A 恒流充电：400 s 后 SOC 0.10 → 0.1275，端压 3.6320 V |
| `sleep` | 300 次空闲后进 **SLEEP**（静态电流 **0.05 mA**）；插入 2 A 负载 → **唤醒**（唤醒计数 1），静态电流回到 8.00 mA |

---

## 7. 开发过程中被测试抓出来的真问题

| # | 问题 | 定位方式 | 修复 |
|---|---|---|---|
| 1 | **保护晚一拍生效** | `-Wunused-but-set-variable` 提示 `flt` 未被使用 | 调整 `pb_dev_step` 顺序：**先跑 BMS 再跑状态机** |
| 2 | **PD 收到对端消息后不发送能力**，握手全部失败 | 握手流程用例在第一步就卡住 | `IDLE` 分支补上"发送 Source_Capabilities 并转入 WAIT_REQUEST" |
| 3 | **RDO 档位字段静默回绕**：传 9 会被 `& 0x07` 变成 2，变成"合法请求" | 越界档位用例未按预期 Reject | `pd_pack_request` 加显式范围夹取，不再静默回绕 |

> 另有 2 处是**测试自己写错**（CRC-32 期望值用错口径、保护后断言端压方向写反），
> 也一并修正 —— 这两条恰好说明"先定义清楚期望值"比"跑绿"更重要。

---

## 8. 边界说明（重要）

- 本项目是**仿真与自建实践**，**没有在真实充电宝硬件上跑过**，不构成产品经验
- **无线充（Qi）未包含** —— 那需要线圈、谐振与调频调压的模拟侧工作，本项目只做有线快充协议
- 电池模型是**单节集总模型**，不含电芯一致性、循环老化、温度对容量的影响
- **BMC 物理层、E-Marker、Sink 侧策略未实现**
- 未做量产相关的 EMC、认证与老化测试

**下一步可做**：把 PHY 层接到真实 PD 控制器（如 FUSB302 类）验证握手时序；
增加 QC/SCP/UFCS 协议分支；把 BMS 与状态机移植到 MCU 上跑真实 MOS 与采样。

---

## 9. 许可

MIT
