/**
 * @file    bt_probe.c
 * @brief   Radio Timeslot 嗅探配对 —— S112 上接收 BLE 广播包的唯一路径.
 *
 * 设计决策:
 *   1. S112 没有 Observer/扫描角色: 其 ble_gap.h 中不存在 sd_ble_gap_scan_start
 *      / BLE_GAP_EVT_ADV_REPORT 等任何扫描 API; 且 nRF52810 官方仅支持 S112,
 *      换 SoftDevice 版本解决不了. 因此"扫描 beacon"只能走 Radio Timeslot API
 *      (nrf_soc.h 提供, 是 Nordic 为 S112 准备的正规射频通道).
 *   2. 射频收包寄存器参数全部取自 SDK ble_dtm.c 与 BLE Core Spec (两者空口包
 *      格式同源): S0=1B(PDU头)/LENGTH=8bit/前导码 8bit/BALEN=3+PREFIX=1 组成
 *      4 字节接入地址 0x8E89BED6/小端/白化使能(种子=信道号)/CRC24 poly 0x65B
 *      初值 0x555555 不含接入地址.
 *   3. Timeslot 信号回调运行在 ARM 中断优先级 0 且禁止调用 sd_* API, 因此回调
 *      里只做射频寄存器配置 (START 信号); 收包由主循环 1 ms 轮询 EVENTS_END,
 *      slot 到期由 SoftDevice 强制收回, 会话转 IDLE 事件后由主循环请求下一片.
 *      1 ms 轮询的漏包窗口对本场景足够: 目标 beacon 每个 ADV 事件在 37/38/39
 *      三信道各发一包, 10 s 窗口内命中概率接近 1.
 *   4. LED 指示/诊断日志等 UI 逻辑不进本模块: 主循环每 1 ms 回调一次 tick
 *      (线程上下文), 应用层自行决定怎么闪灯/打日志, 诊断计数经
 *      bt_probe_stats_get 读取 —— 保持射频逻辑与 UI 解耦.
 */
#include "bt_probe.h"

#include <string.h>
#include "nrf.h"          // NRF_RADIO 寄存器访问
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_soc.h"      // sd_radio_* Timeslot API
#include "nrf_sdh_soc.h"  // SOC 事件观察者
#include "ble_gap.h"      // BLE_GAP_ADDR_TYPE_PUBLIC / BLE_GAP_ADDR_LEN

// ---------------- 嗅探配置 ----------------
#define SCAN_WINDOW_MS      10000u     // 配对扫描超时: 10 s
#define SLOT_LEN_US         100000u    // 单个 Radio Timeslot 时长 (API 上限 100 ms)
#define VENDOR_OUI_B0       0x78       // 目标 public MAC 前缀 78:81:8C (MSB 侧三字节)
#define VENDOR_OUI_B1       0x81
#define VENDOR_OUI_B2       0x8C
#define BLE_ADV_ACCESS_ADDR 0x8E89BED6u // BLE 广播信道固定接入地址

// 广播信道 37/38/39 -> 频率寄存器值 (MHz 偏移): 2402/2426/2480
static const uint8_t m_adv_ch_freq[3] = {2, 26, 80};

// ---------------- 嗅探运行状态 ----------------
static uint8_t m_rx_buf[48] __attribute__((aligned(4))); // [S0][LEN][payload<=37][CRC3], RADIO DMA 直接写入
static volatile uint8_t m_sniff_ch = 37;    // 当前监听的广播信道, 每 slot 轮换 (prio-0 回调读取)
static volatile bool    m_sniff_done;       // 命中目标 (主循环读写)
static volatile bool    m_slot_idle;        // SOC: slot 结束, 会话空闲, 可请求下一片
static volatile bool    m_slot_blocked;     // SOC: 请求被拒/被取消, 需重试
static volatile bool    m_session_closed;   // SOC: 会话已关闭, 射频归还完毕
static uint32_t         m_rx_total;         // 诊断: 收到的包总数 (含 CRC 错)
static uint32_t         m_rx_crc_ok;        // 诊断: CRC 正确的包数
static uint32_t         m_rx_match;         // 诊断: 前缀命中数
static pair_record_t    m_pair_cand;        // 命中的候选记录 (RAM)

// Timeslot 请求参数: EARLIEST + 100 ms + 保证 16M 晶振. 回调返回 REQUEST_AND_END
// 需要请求参数指针, 本设计不在回调里链式请求, 故只需这一个静态请求.
static nrf_radio_request_t m_ts_request = {
    .request_type   = NRF_RADIO_REQ_TYPE_EARLIEST,
    .params.earliest = {
        .hfclk      = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
        .priority   = NRF_RADIO_PRIORITY_NORMAL,
        .length_us  = SLOT_LEN_US,
        .timeout_us = NRF_RADIO_EARLIEST_TIMEOUT_MAX_US,
    },
};

// 回调固定返回"维持当前 slot" (配置完射频后一切交给主循环)
static nrf_radio_signal_callback_return_param_t m_ts_ret_none = {
    .callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,
};

// SOC 事件 (Timeslot 会话状态), SWI2 中断上下文 -> 只置标志
static void soc_evt_handler(uint32_t evt_id, void * p_context) {
    switch (evt_id) {
        case NRF_EVT_RADIO_SESSION_IDLE: m_slot_idle = true; break;
        case NRF_EVT_RADIO_SESSION_CLOSED: m_session_closed = true; break;
        case NRF_EVT_RADIO_BLOCKED: m_slot_blocked = true; break;
        case NRF_EVT_RADIO_CANCELED: m_slot_blocked = true; break;
        default: break;
    }
}

NRF_SDH_SOC_OBSERVER(m_soc_observer, 0, soc_evt_handler, NULL);

static void sniff_radio_configure(uint8_t ch); // 前置声明, 实现见下

// Timeslot 信号回调 —— 运行在 ARM 中断优先级 0:
//   - 严禁调用任何 sd_* API;
//   - 只做寄存器配置, 不做日志/耗时操作;
//   - 收包不走 RADIO 中断 (S112 对 RADIO IRQ 转发的语义未经验证), 由主循环
//     轮询 EVENTS_END; slot 到期 SoftDevice 自动收回射频.
static nrf_radio_signal_callback_return_param_t * radio_signal_callback(uint8_t signal_type) {
    if (signal_type == NRF_RADIO_CALLBACK_SIGNAL_TYPE_START) {
        sniff_radio_configure(m_sniff_ch);
    }
    // 其他信号 (TIMER0/RADIO/EXTEND_*) 本设计不会产生, 统一无动作返回.
    return &m_ts_ret_none;
}

// 配置 RADIO 在指定广播信道 (37/38/39) 连续接收 BLE legacy 广播包.
// 参数来源: SDK ble_dtm.c (与 BLE 空口格式同源) + BLE Core Spec:
//   接入地址 0x8E89BED6 按小端上电序 D6,BE,89,8E 对应 BASE0=AA<<8, PREFIX0=AA>>24;
//   CRC24: poly 0x65B (x^24 隐含), 初值 0x555555, 不含接入地址 (SKIPADDR=1);
//   白化: PCNF1.WHITEEN=1, 种子 DATAWHITEIV=信道号.
static void sniff_radio_configure(uint8_t ch) { // ch: 37/38/39
    NRF_RADIO->TASKS_DISABLE  = 1;
    NRF_RADIO->SHORTS         = 0;
    NRF_RADIO->EVENTS_READY   = 0;
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_END     = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    for (volatile uint32_t i = 0; i < 100; i++) {} // 等状态机回到 DISABLED (~us 级)

    NRF_RADIO->MODE        = RADIO_MODE_MODE_Ble_1Mbit;
    NRF_RADIO->FREQUENCY   = m_adv_ch_freq[ch - 37];
    NRF_RADIO->DATAWHITEIV = ch;
    NRF_RADIO->BASE0       = BLE_ADV_ACCESS_ADDR << 8;  // 0x89BED600
    NRF_RADIO->PREFIX0     = BLE_ADV_ACCESS_ADDR >> 24; // AP0 = 0x8E
    NRF_RADIO->RXADDRESSES = 0x1;                       // 只匹配逻辑地址 0

    NRF_RADIO->PCNF0 = (1UL << RADIO_PCNF0_S0LEN_Pos)
                     | (8UL << RADIO_PCNF0_LFLEN_Pos)
                     | (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);
    NRF_RADIO->PCNF1 = (42UL << RADIO_PCNF1_MAXLEN_Pos)
                     | (3UL  << RADIO_PCNF1_BALEN_Pos)
                     | (RADIO_PCNF1_ENDIAN_Little   << RADIO_PCNF1_ENDIAN_Pos)
                     | (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three       << RADIO_CRCCNF_LEN_Pos)
                       | (RADIO_CRCCNF_SKIPADDR_Skip   << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCPOLY = 0x0000065B;
    NRF_RADIO->CRCINIT = 0x00555555;
    NRF_RADIO->PACKETPTR = (uint32_t)m_rx_buf;
    NRF_RADIO->INTENCLR  = 0xFFFFFFFF; // 不使用无线电中断, 主循环轮询

    // 连续接收: READY->START (ramp 完成自动开收), END->START (收完一包立即重新武装)
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos)
                      | (RADIO_SHORTS_END_START_Enabled   << RADIO_SHORTS_END_START_Pos);
    NRF_RADIO->TASKS_RXEN = 1;
}

// 处理一个收到的广播包 (主循环 1 ms 轮询调用).
// m_rx_buf 布局: [0]=S0(PDU头) [1]=LENGTH [2..]=payload [..]=CRC3(硬件校验后附加).
// AdvA 位于 payload[0..5] 小端, MSB 在 buf[7]; 广播数据在 buf[8..].
static void sniff_packet_process(void) {
    if (NRF_RADIO->EVENTS_END == 0) return;
    NRF_RADIO->EVENTS_END = 0;
    m_rx_total++;
    if (NRF_RADIO->CRCSTATUS == 0) return; // CRC 错误直接丢弃
    m_rx_crc_ok++;
    uint8_t const pdu_type = m_rx_buf[0] & 0x0F;
    // 只认带 AdvA 且非定向的广播 PDU: ADV_IND(0)/ADV_NONCONN_IND(2)/ADV_SCAN_IND(6).
    // 排除: 定向包 (AdvA 不在 payload 头部), 扫描响应 (数据语义不同), 扩展广播.
    if (pdu_type != 0x00 && pdu_type != 0x02 && pdu_type != 0x06) return;
    uint8_t const len = m_rx_buf[1]; // PDU payload 长度 (含 AdvA)
    if (len < 6 || len > 37) return;
    if (m_rx_buf[0] & 0x40) return; // TxAdd=1 -> 对端为随机地址; 本需求只配 public
    // 前缀匹配: AdvA 小端, 78:81:8C 三字节位于 buf[7],buf[6],buf[5]
    if (m_rx_buf[7] != VENDOR_OUI_B0
        || m_rx_buf[6] != VENDOR_OUI_B1
        || m_rx_buf[5] != VENDOR_OUI_B2
    ) return;
    m_rx_match++;
    // 命中: 地址与广播数据原样拷贝, 首个命中即锁定
    memcpy(m_pair_cand.addr, &m_rx_buf[2], BLE_GAP_ADDR_LEN);
    m_pair_cand.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    m_pair_cand.data_len  = (uint8_t)(len - 6);
    memcpy(m_pair_cand.data, &m_rx_buf[8], m_pair_cand.data_len);
    m_sniff_done = true;
}

void bt_probe_stats_get(bt_probe_stats_t * p_stats) {
    p_stats->rx_total  = m_rx_total;
    p_stats->rx_crc_ok = m_rx_crc_ok;
    p_stats->rx_match  = m_rx_match;
    p_stats->cur_ch    = m_sniff_ch;
}

// 配对嗅探主流程: 开会话 -> 循环 (收包/链式请求新 slot/每 ms 回调 tick) ->
// 关会话. 返回 true = 命中, *p_out 为候选记录 (尚未落盘, 持久化由调用方决定,
// 这样 Flash 策略与射频逻辑解耦; 会话已关闭, 落盘安全).
bool bt_probe_run(pair_record_t * p_out, bt_probe_tick_fn_t tick) {
    ret_code_t err_code;
    uint32_t elapsed_ms = 0, blocked_cnt = 0;
    memset(&m_pair_cand, 0, sizeof(m_pair_cand));
    m_rx_total = m_rx_crc_ok = m_rx_match = 0;
    m_sniff_done     = false;
    m_slot_idle      = false;
    m_slot_blocked   = false;
    m_session_closed = false;
    m_sniff_ch       = 37;
    err_code = sd_radio_session_open(radio_signal_callback);
    APP_ERROR_CHECK(err_code);
    err_code = sd_radio_request(&m_ts_request);
    APP_ERROR_CHECK(err_code);
    while (!m_sniff_done && elapsed_ms < SCAN_WINDOW_MS) {
        sniff_packet_process();
        if (m_slot_idle || m_slot_blocked) { // slot 结束或请求被拒 -> 轮换信道请求下一片
            bool was_blocked = m_slot_blocked;
            m_slot_idle    = false;
            m_slot_blocked = false;
            if (was_blocked) {
                if (++blocked_cnt > 20) break; // 持续调度失败 (BLE 空闲时不应发生)
                nrf_delay_ms(5);
            }
            m_sniff_ch = 37 + (uint8_t)(((m_sniff_ch - 37) + 1) % 3);
            err_code = sd_radio_request(&m_ts_request);
            if (err_code == NRF_ERROR_FORBIDDEN) {}
            // 事件竞态: 会话仍忙, 等下一次 IDLE 事件再试, 不算错误
            else APP_ERROR_CHECK(err_code);
        }

        nrf_delay_ms(1); // 1 ms 轮询: 漏包窗口对 10 s 配对窗口而言可忽略
        elapsed_ms++;
        if (tick != NULL) tick(elapsed_ms);
    }

    // 关闭会话 (SD 会先结束当前 slot); 等到 CLOSED 事件后射频才完全归还,
    // 之后调用方才允许做 Flash 擦写 (NVMC 与射频互斥).
    m_session_closed = false;
    err_code = sd_radio_session_close();
    APP_ERROR_CHECK(err_code);
    uint32_t wait_ms = 300;
    while (!m_session_closed) {
        nrf_delay_ms(1);
        if (--wait_ms == 0) return false;
    }

    if (!m_sniff_done) {
        NRF_LOG_INFO("scan timeout, no vendor beacon found");
        NRF_LOG_FINAL_FLUSH();
        return false;
    }

    NRF_LOG_INFO(
        "matched peer: %x %x %x %x %x %x",
        m_pair_cand.addr[5], m_pair_cand.addr[4], m_pair_cand.addr[3],
        m_pair_cand.addr[2], m_pair_cand.addr[1], m_pair_cand.addr[0]
    );
    NRF_LOG_INFO("peer data_len=%u", m_pair_cand.data_len);
    *p_out = m_pair_cand;
    return true;
}
