# NS2-Waker (nRF52810)

超低功耗 BLE「唤醒包克隆器」固件：先嗅探并克隆目标设备（Joy-Con 2）的厂商 beacon，
之后每次按键以**对端的 MAC 地址和原始广播数据**发射一包非连接广播，唤醒目标设备；
广播 1 秒后进入 System OFF（典型 ~0.4 µA）。

硬件：`NSWakerV31.epro2`，嘉立创EDA工程，复刻的话把金手指连接器改成标准2.54更方便连JLink。

> 有问题可以提[Issue](issues)，当然也可以[给我发邮件](mailto:wangyw.dev@outlook.com)。

---

## 1. 工作原理与双模式流程

目标设备（Joy-Con 2）周期性发送 public 地址、厂商数据前缀 `53 05` 的非连接广播。
本设备CR2032电池供电，绝大多数时间处于 System OFF，两种模式由**开机/复位瞬间采样唤醒键电平**区分：

```
复位 (上电 / P0.12 拉低 / RESET 键)
  │
  ├─ Flash 无有效配对记录 ──────────────> 【配对模式】
  ├─ 唤醒键按住 ≥ 1.5s (PAIR_HOLD_MS) ──> 【配对模式】
  └─ 短按 / 复位, 且 Flash 有记录 ───────> 【常规模式】

【常规模式】
  └─ 读配对记录 -> GAP 地址与广播数据原样克隆 -> ADV_NONCONN_IND 广播 1 s (LED 亮) -> System OFF

【配对模式】
  │ Radio Timeslot 嗅探 10 s
  │ 首片锁定 37 信道, EXTEND 续片, 匹配 AdvA 前缀 78:81:8C (public)
  ├─ 命中 -> MAC+原始广播数据落盘 -> LED 常亮 1 s -> System OFF
  └─ 超时 -> LED 快闪 5 次 -> System OFF
```

- System OFF 的唤醒源：`WAKEUP_BUTTON_PIN` (P0.12) 低电平 SENSE，或 RESET 引脚。
  两者本质都是复位，程序从头执行，因此模式判定必须在复位后尽早完成（GPIO 先于 SoftDevice 初始化）。

## 2. 硬件

| 项 | 值 |
|---|---|
| SoC | Nordic nRF52810 (Cortex-M4, 192 KB Flash / 24 KB RAM) |
| SoftDevice | **S112 7.2.0**（nRF52810 官方唯一支持的 SoftDevice） |
| SDK | nRF5 SDK **17.1.1**（路径见 Makefile `SDK_ROOT`，自己下载） |
| 唤醒键 | **P0.12**，低有效，内部上拉（宏 `WAKEUP_BUTTON_PIN`，便于改板） |
| LED | **P0.06**，开漏驱动（`NRF_GPIO_PIN_S0D1`），低电平点亮 |
| LF 时钟 | **片内 RC**（PCB 无 32.768 kHz 晶振；`CLOCK_CONFIG_LF_SRC=0` 等） |
| 电源 | 电池直连 VDD；**禁用 DC-DC，使用内部 LDO**（DCC 引脚无外部 LC 电路，以后要修改成DCDC） |

> 调试器（J-Link SWD）连接时芯片无法真正进入 System OFF，测休眠电流前必须断开调试器。

## 3. 软件架构

四个自包含模块，射频 / 持久化 / 广播 / UI 显式解耦：

| 文件 | 职责 |
|---|---|
| `main.c` | 入口、模式判定、GPIO/LED、协议栈初始化、流程编排、System OFF |
| `bt_probe.c/.h` | **Radio Timeslot 裸射频嗅探**（S112 无 Observer 角色的唯一路径，见下） |
| `bt_advertising.c/.h` | 克隆广播：对端 MAC + 原始广播字节，ADV_NONCONN_IND，1 秒定时 |
| `persistence.c/.h` | 配对记录 Flash 存取：CRC16 + 三步写入掉电半写保护 |
| `sdk_config.h` | SDK 配置（各开关有 `#ifndef` 保护，可由 Makefile `-D` 覆盖） |
| `ble_app_beacon_gcc_nrf52.ld` | 链接脚本：Flash 末页让给配对记录 |
| `esp32c3_arduino.c` | ESP32-C3 原型，**仅参考不参与编译** |

### 3.1 内存与 Flash 布局

```
0x00000000  MBR + SoftDevice S112 (0x19000)
0x00019000  应用区起始; FLASH LENGTH=0x16000 -> 应用结束于 0x2F000
0x0002F000  配对记录页 (页 47, 4 KB), 仅存 48 字节 pair_record_t
0x20001118  应用 RAM 起始 (SoftDevice 占用其下空间)
```

`make flash` 用 `--sectorerase` 只擦应用 hex 覆盖的扇区，**升级固件不会丢配对记录**。
手动清除配对记录（强制重新配对）：

```bash
nrfjprog -f nrf52 --erasepage 0x2f000
```

`pair_record_t`（48 字节，4 字节对齐，可直接按字写 Flash）：
`addr[6] + addr_type + data_len + data[31] + pad + crc16 + magic("NS2W")`。
写入顺序：页擦除 → 数据+CRC（11 字）→ **magic 最后单独写**。任何时刻掉电 magic 都不会就位，
上电校验失败即自动回到配对模式。

### 3.2 嗅探为什么用 Radio Timeslot（重要背景）

S112 的 `ble_gap.h` 中**不存在** `sd_ble_gap_scan_start` / `BLE_GAP_EVT_ADV_REPORT`
（S112 只有外设/广播角色，无 Observer），换 SoftDevice 版本也解决不了。
因此嗅探走 SoftDevice 并存的 **Radio Timeslot API**（`sd_radio_session_open` / `sd_radio_request`），
在 SoftDevice 让出射频的时间片里直接操作 RADIO 寄存器收包。寄存器配置全部取自 SDK
`ble_dtm.c`（与 BLE 空口格式同源）：

- 1 Mbps LE，广播接入地址 `0x8E89BED6`（BASE0/PREFIX0 小端拆分）
- S0=1 字节 / LFLEN=8bit / 前导码 8bit / BALEN=3
- CRC24，poly `0x65B`，初值 `0x555555`，SKIPADDR=1（不含接入地址）
- 白化使能，种子 `DATAWHITEIV = 信道号`
- 短包连续接收：`READY_START` + `END_START`；收包走 RADIO 中断（SD 转发为
  `SIGNAL_TYPE_RADIO` 回调），回调内做匹配，无主循环轮询的漏包窗口

### 3.2.1 收片机制（三个必须知道的坑，实测确立）

Timeslot 的 signal callback **不只 START 时被调用**：SD 会把 RADIO / TIMER0 中断转发为
`SIGNAL_TYPE_RADIO` / `SIGNAL_TYPE_TIMER0` 回调。当前实现：

1. **START 回调**：配置 RADIO（首片锁定 37 信道）+ 武装 TIMER0（片尾前 1.5 ms 触发），
   返回 `NONE` 让片满 100 ms 持续收包。
2. **TIMER0 回调**：**先清 `NRF_TIMER0->EVENTS_COMPARE[0]`**，再返回 `EXTEND`（每次延 60 ms）
   续片——RADIO 归属在整段 10 s 窗口内不交接给 SD；命中后返回 `NONE` 让片自然到期。
3. **RADIO 回调**：处理 `EVENTS_END` 收包 + 匹配（优先级 0 内只做寄存器读取/内存拷贝）。
4. **退路**：`EXTEND_FAILED` 时返回 `REQUEST_AND_END` + `NORMAL` 链式下一片；链式被
   `BLOCKED` 时主循环以 `EARLIEST` 重新发起。

**坑 1**：不能在 START 回调返回 `REQUEST_AND_END`——它会**立即结束当前片**，radio 只活回调
执行那几微秒，实测 `rx` 恒为 0。

**坑 2**：TIMER0 的 `EVENTS_COMPARE[0]` **必须显式清除**。该标志一旦置位、不清除就一直是 1，
SD 会据此反复回调 `SIGNAL_TYPE_TIMER0`（实测 **49013 次/秒**），海量 EXTEND 请求迫使 SD
不断重新协商射频资源、频繁接管 RADIO，把连续接收配置冲掉（`STATE=Disabled` / `SHORTS=0` /
`DATAWHITEIV` 被覆盖成 SD 的 `0x40`），表现为「第一片收几十包后永久冻结」。

**坑 3**：「每片独立开会话 / 每片重配 RADIO」在片边界会失效（SD 收回 RADIO 后 `STATE` 与
`SHORTS` 被清），务必用 EXTEND 保持 RADIO 归属不交接。

匹配规则（`bt_probe.c` 的 `radio_irq_process`，实测确认）：
PDU 类型 ∈ {ADV_IND(0), ADV_NONCONN_IND(2), ADV_SCAN_IND(6)}；
**TxAdd=0（public 地址）**；`len` 6~37；AdvA 高 3 字节 = `78 81 8C`。
实测 Joy-Con 2 的地址为 `78 81 8C D6 B7 38`（与 `esp32c3_arduino.c` 原型中的
baseMac 末字节 −2 的旁注一致）。

### 3.3 克隆广播参数

- PDU 固定 `ADV_NONCONN_IND`（源 beacon 即使是 ADV_IND，空口仅 PDU 头 1 字节差异）
- 广播间隔 100 ms（非连接广播协议最小值），时长 1 s（`duration=100`，**单位 10 ms**）
- 当前发射功率 **-4 dBm**（S112 仅支持 9 档：-40/-20/-16/-12/-8/-4/0/+3/+4，其他值 INVALID_PARAM；
  nRF52810 上限 +4 dBm）
- 广播数据 = 嗅探到的**原始字节直接 memcpy**，不重新组包

## 4. 开发环境

- macOS（当前开发机）；arm-none-eabi-gcc 15.3（实测可用；老版本亦可）
- nrfjprog 10.x + J-Link（板载或独立 J-Link，SWD）
- 日志：**RTT backend**（`NRF_LOG_BACKEND_RTT_ENABLED=1`，UART backend 关闭），用 J-Link RTT Viewer 查看
- IDE：`.vscode/c_cpp_properties.json` 已提交（含 SDK include 路径与交叉编译器路径，解决 IntelliSense 红线）；
  `.clangd` 已提交。`compile_commands.json` 被 gitignore
- 若 SDK 不在默认路径，改 Makefile 的 `SDK_ROOT`

## 5. 构建与烧录

```bash
cd …/Firmware

make                 # DEBUG 版, 带 RTT 日志, 产物 _build/
make RELEASE=1       # 量产版, NRF_LOG 全裁剪为 no-op, 产物 _build_release/ (Flash/RAM 大幅缩小)

make flash           # 烧应用 (--sectorerase, 不动 SoftDevice 与配对记录页)
make erase           # 全片擦除
make flash_softdevice  # 单独烧 S112 (components/softdevice/s112/hex/s112_nrf52_7.2.0_softdevice.hex)
```

**烧录顺序铁律（踩过坑）**：`make erase` 之后必须先 `make flash_softdevice` 再 `make flash`。
应用 hex 只含 0x19000 起的内容，不会写 MBR/SoftDevice；只烧应用会导致芯片复位后
PC=0xFFFFFFFE、SP=0xFFFFFFD8、寄存器全零（从地址 0/4 读到全 F），表现为「完全没运行」。

## 6. RTT 调试方法

J-Link RTT Viewer 配置：Device **手输** `nRF52810_xxAA`，接口 SWD（4000 kHz），
RTT Control Block 用 Auto Detection（失败时可从 `.map` 文件查 `_SEGGER_RTT` 地址手动指定）。

注意：

- RTT 控制块在 `.bss`，其 "SEGGER RTT" 标识在**运行时第一条日志写入后**才填充；
  程序必须先跑起来 Viewer 才能找到控制块。
- `sdk_config.h` 默认 `NRF_LOG_DEFERRED=1`（日志入队，主循环 flush）。
  若怀疑程序在初始化中途卡死，可临时用 `-DNRF_LOG_DEFERRED=0`（直写模式）：
  每条日志立即落 RTT，即使随后 HardFault 也保留现场。
- 快速定位 SoftDevice API 失败的方法：在每个 `sd_*` 调用的 `APP_ERROR_CHECK` 前
  先打印返回码（`[gatt] xxx ret=0x%x` 风格），第一个非 0 即故障点。

## 7. 抓包 / 配对操作说明

- 手机扫描工具：nRF Connect（推荐 **Android**；iOS 因系统限制拿不到扫描设备 MAC）。
- Joy-Con 2 MAC 形如 `78:81:8C:XX:XX:XX`（`78:81:8C` 为 Nintendo 注册 OUI，可能还有其他前缀）。
- 厂商广播数据形态示例：`02 01 06 1B FF 53 05 …`（Flags=0x06，厂商数据以 `53 05` 开头）。
  正常流程下该数据由嗅探自动抓取克隆，**无需手工硬编码**；`esp32c3_arduino.c` 里保留了
  一份手工逆向的样例（含「base MAC 末字节 = BT MAC 末字节 − 2」的旁注）供对照。

## 8. 已知坑位汇总（开发期实际踩过）

- `ble_gap_adv_params_t.duration` 单位是 **10 ms**：1500 = 15 s 而非 1.5 s。
- `sd_ble_gatts_characteristic_add`：`p_value=NULL` 时 `init_len` 必须为 0，否则 INVALID_PARAM；手写 GATT 元数据易错，优先用 SDK `characteristic_add` helper。（S112 无 DLE API。）
- nrf_log 直写模式下 `%s` 偶发截断/乱码，关键日志优先用纯数字格式。
- Timeslot：START 回调返回 `REQUEST_AND_END` 会**立即结束当前片**（radio 收不到包）；续片应在 TIMER0 回调里做。
- Timeslot：signal callback 运行在优先级 0，**不能调用 `sd_*`，也不能打日志**
  （`NRF_LOG_PROCESS` 只能在主循环调用）；诊断快照需拷出后由主循环打印。
- RTT 上行缓冲过小（默认 512 B）时大量诊断日志会 "Logs dropped" 丢现场，调试期可把 `sdk_config.h` 的 `SEGGER_RTT_CONFIG_BUFFER_SIZE_UP` 调大（如 4096）。

---

*Vibe Coding: TRAE Agent 协助开发；CodeBuddy Agent 协助开发*
