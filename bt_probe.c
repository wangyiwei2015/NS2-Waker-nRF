/**
 * @file    bt_probe.c
 * @brief   Radio Timeslot 嗅探配对 —— S112 上接收 BLE 广播包的唯一路径.
 *
 * 设计决策:
 *   1. S112 没有 Observer/扫描角色: 且 nRF52810 官方仅支持 S112, 因此扫描 beacon
 *      走 Radio Timeslot API(nrf_soc.h 提供, 是 Nordic 为 S112 准备的正规射频通道).
 *   2. 射频收包寄存器参数取自 SDK ble_dtm.c 与 BLE Core Spec (两者空口包格式
 *      同源): S0=1B(PDU头)/LENGTH=8bit/前导码 8bit/BALEN=3+PREFIX=1 组成 4 字节
 *      接入地址 0x8E89BED6/小端/白化使能(种子=信道号)/CRC24 poly 0x65B
 *      初值 0x555555 不含接入地址.
 *   3. 收片架构 (关键, 实测确立): signal callback 不只 START 时调用, SD 会把
 *      RADIO/TIMER0 中断转发为 SIGNAL_TYPE_RADIO / SIGNAL_TYPE_TIMER0 回调.
 *      - START 回调: 配置射频 + 武装 TIMER0, 返回 NONE 让片满 100 ms 持续收包.
 *        不能在 START 返回 REQUEST_AND_END —— 它会立即结束当前片, radio 只活
 *        回调执行那几微秒, 实测 rx 恒为 0.
 *      - TIMER0 回调 (片尾 - TS_TIMER0_MARGIN_US) 返回 EXTEND 延长当前片, 使
 *        RADIO 归属在整段嗅探窗口内不交接给 SoftDevice; 实测"每片重配"在片边界
 *        会失效 (SD 收回 RADIO 后 STATE=Disabled/SHORTS 被清/DATAWHITEIV 被覆盖),
 *        故 EXTEND 是唯一可靠的连续收包方式. 命中后返回 NONE 让片自然到期.
 *      - TIMER0 的 COMPARE0 事件标志必须显式清除! 不清则标志常驻, SD 会每
 *        ~20 us 重复回调 (实测 EXTEND 风暴 49013 次/秒), 进而 SD 频繁接管 RADIO.
 *      - EXTEND_FAILED (SD 需要射频时拒绝延长) 退回 REQUEST_AND_END 链式下一片;
 *        链式被 BLOCKED 时 SD 发事件并回 IDLE, 主循环以 EARLIEST 重新发起恢复.
 *      - RADIO 回调: 处理 EVENTS_END 收包 + 匹配 (优先 0 中断内只做寄存器读取/
 *        内存拷贝, 不打日志不调 sd_*). 中断收包无主循环轮询的漏包窗口.
 *   4. LED 指示/日志等 UI 逻辑不进本模块: 主循环每 1 ms 回调一次 tick (线程
 *      上下文), 运行统计经 bt_probe_stats_get 读取, 射频与 UI 解耦.
 */
#include "bt_probe.h"

#include <string.h>
#include "nrf.h"          // NRF_RADIO/NRF_TIMER0 寄存器访问, NVIC
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
#define TS_TIMER0_MARGIN_US 1500u      // 片尾提前触发 TIMER0 的余量 (EXTEND/续片准备)
#define TS_EXTEND_LEN_US    60000u     // 每次 EXTEND 的延长量 (>= NRF_RADIO_MINIMUM_TIMESLOT_LENGTH_EXTENSION_TIME_US=200)
#define VENDOR_OUI_B0       0x78       // 目标 public MAC 前缀 78:81:8C (MSB 侧三字节)
#define VENDOR_OUI_B1       0x81
#define VENDOR_OUI_B2       0x8C
#define BLE_ADV_ACCESS_ADDR 0x8E89BED6u // BLE 广播信道固定接入地址

// 广播信道 37/38/39 -> 频率寄存器值 (MHz 偏移): 2402/2426/2480
static const uint8_t m_adv_ch_freq[3] = {2, 26, 80};

// ---------------- 嗅探运行状态 ----------------
static uint8_t m_rx_buf[48] __attribute__((aligned(4))); // [S0][LEN][payload<=37][CRC3], RADIO DMA 直接写入
static volatile uint8_t m_sniff_ch = 37;    // 当前监听的广播信道 (片轮换, prio-0 回调读写)
static volatile bool    m_sniff_done;       // 命中目标
static volatile bool    m_slot_blocked;     // SOC: 链式请求被拒, 主循环恢复
static volatile bool    m_session_closed;   // SOC: 会话已关闭, 射频归还完毕
static uint32_t         m_rx_total;         // 统计: 收到的包总数 (含 CRC 错)
static uint32_t         m_rx_crc_ok;        // 统计: CRC 正确的包数
static uint32_t         m_rx_match;         // 统计: 命中数
static pair_record_t    m_pair_cand;        // 命中的候选记录 (RAM)

// Timeslot 请求参数.
// 首个请求: EARLIEST (会话内首请求强制). 正常路径由 TIMER0 回调 EXTEND 续片;
// EXTEND 被拒时退回链式 NORMAL 下一片; 链式被 BLOCKED 后主循环用 EARLIEST 恢复.
static nrf_radio_request_t m_first_request = {
    .request_type   = NRF_RADIO_REQ_TYPE_EARLIEST,
    .params.earliest = {
        .hfclk      = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
        .priority   = NRF_RADIO_PRIORITY_NORMAL,
        .length_us  = SLOT_LEN_US,
        .timeout_us = NRF_RADIO_EARLIEST_TIMEOUT_MAX_US,
    },
};

// 链式续片: 相对上一片起点 distance_us 后开始, = 片长度即无缝衔接.
static nrf_radio_request_t m_next_request = {
    .request_type   = NRF_RADIO_REQ_TYPE_NORMAL,
    .params.normal = {
        .hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
        .priority    = NRF_RADIO_PRIORITY_NORMAL,
        .distance_us = SLOT_LEN_US,
        .length_us   = SLOT_LEN_US,
    },
};

// 回调返回参数必须为静态 (SoftDevice 在回调返回后仍会引用该指针, 见 nrf_soc.h).
static nrf_radio_signal_callback_return_param_t m_ts_ret_none = {
    .callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,
};

static nrf_radio_signal_callback_return_param_t m_ts_ret_request_and_end = {
    .callback_action        = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
    .params.request.p_next = &m_next_request,
};

// EXTEND: 延长当前片而不结束它, RADIO 归属不交接, 收包零断流.
static nrf_radio_signal_callback_return_param_t m_ts_ret_extend = {
    .callback_action         = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,
    .params.extend.length_us = TS_EXTEND_LEN_US,
};

// SOC 事件 (Timeslot 会话状态), SWI2 中断上下文 -> 只置标志
static void soc_evt_handler(uint32_t evt_id, void * p_context) {
    switch (evt_id) {
        case NRF_EVT_RADIO_SESSION_CLOSED: m_session_closed = true; break;
        case NRF_EVT_RADIO_BLOCKED:        m_slot_blocked   = true; break;
        case NRF_EVT_RADIO_CANCELED:       m_slot_blocked   = true; break;
        default: break;
    }
}

NRF_SDH_SOC_OBSERVER(m_soc_observer, 0, soc_evt_handler, NULL);

static void sniff_radio_configure(uint8_t ch); // 前置声明, 实现见下
static void radio_irq_process(void); // 前置声明, 实现见下

// 武装 TIMER0: 在片尾 (SLOT_LEN_US - TS_TIMER0_MARGIN_US) 触发 COMPARE0 中断,
// SD 将其转发为 SIGNAL_TYPE_TIMER0 回调, 在那里 EXTEND 续片.
// TIMER0 在片开始时被 SD 复位, 此处显式保证 32bit 计数 + 1 MHz (16 MHz / 2^4).
static void ts_timer0_arm(void) {
    NRF_TIMER0->TASKS_STOP  = 1;
    NRF_TIMER0->BITMODE     = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER0->PRESCALER   = 4; // 16 MHz / 2^4 = 1 MHz, 计数单位 1 us
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->CC[0]       = SLOT_LEN_US - TS_TIMER0_MARGIN_US;
    // 清 COMPARE0 事件标志! 它一旦置位不清除会一直是 1, SD 会据此反复回调
    // SIGNAL_TYPE_TIMER0, 造成 EXTEND 风暴 (实测 49013 次/秒), 进而 SD 频繁接管
    // RADIO 把连续接收配置冲掉 (STATE=Disabled / SHORTS=0 / DATAWHITEIV 被覆盖).
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->INTENCLR    = 0xFFFFFFFF;
    NRF_TIMER0->INTENSET    = TIMER_INTENSET_COMPARE0_Msk;
    NVIC_EnableIRQ(TIMER0_IRQn);
    NRF_TIMER0->TASKS_START = 1;
}

// Timeslot 信号回调 —— 运行在 ARM 中断优先级 0:
//   - 严禁调用任何 sd_* API 与 NRF_LOG (deferred flush 不允许在中断上下文);
//   - START: 配置射频 + 武装 TIMER0, 返回 NONE 让片满时长收包;
//   - TIMER0: 清事件标志; 未命中则 EXTEND 续片, 命中后返回 NONE 让片自然到期;
//   - EXTEND_SUCCEEDED: CC0 顺延一个延长量, 维持恒定 margin;
//   - EXTEND_FAILED: 退回链式下一片 (小概率退路);
//   - RADIO: 处理收包事件与匹配.
static nrf_radio_signal_callback_return_param_t * radio_signal_callback(uint8_t signal_type) {
    switch (signal_type) {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            sniff_radio_configure(m_sniff_ch);
            // 轮换到下一信道 (EXTEND 正常路径下不再有 START, 此值仅链式退路使用)
            m_sniff_ch = 37 + (uint8_t)(((m_sniff_ch - 37) + 1) % 3);
            ts_timer0_arm();
            return &m_ts_ret_none;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            NRF_TIMER0->EVENTS_COMPARE[0] = 0; // 必须清, 否则 EXTEND 风暴 (见上方注释)
            if (!m_sniff_done) {
                return &m_ts_ret_extend;
            }
            return &m_ts_ret_none;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
            NRF_TIMER0->CC[0] += TS_EXTEND_LEN_US; // 片尾顺延, margin 恒定
            return &m_ts_ret_none;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
            if (!m_sniff_done) {
                return &m_ts_ret_request_and_end;
            }
            return &m_ts_ret_none;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            radio_irq_process();
            return &m_ts_ret_none;
        default:
            return &m_ts_ret_none;
    }
}

// 配置 RADIO 在指定广播信道 (37/38/39) 连续接收 BLE legacy 广播包.
// 参数来源: SDK ble_dtm.c (与 BLE 空口格式同源) + BLE Core Spec:
//   接入地址 0x8E89BED6 按小端上电序 D6,BE,89,8E 对应 BASE0=AA<<8, PREFIX0=AA>>24;
//   CRC24: poly 0x65B (x^24 隐含), 初值 0x555555, 不含接入地址 (SKIPADDR=1);
//   白化: PCNF1.WHITEEN=1, 种子 DATAWHITEIV=信道号.
static void sniff_radio_configure(uint8_t ch) { // ch: 37/38/39
    // 先确保 RADIO 回到 DISABLED. 注意: 若 RADIO 已处于 DISABLED, 触发 TASKS_DISABLE
    // 不会产生 EVENTS_DISABLED (无状态转换), 用 while(EVENTS_DISABLED==0) 忙等会死
    // 循环 (该回调运行在优先级 0, 会把整个系统卡死). 这里以 STATE 寄存器判断状态机
    // 是否回到 DISABLED, 带有限超时兜底.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;
    for (volatile uint32_t i = 0; i < 100000; i++) {
        if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled) break;
    }
    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_READY    = 0;
    NRF_RADIO->EVENTS_ADDRESS  = 0;
    NRF_RADIO->EVENTS_END      = 0;
    NRF_RADIO->EVENTS_CRCOK    = 0;
    NRF_RADIO->EVENTS_CRCERROR = 0;
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
    // 收包走 RADIO 中断 (SD 转发为 SIGNAL_TYPE_RADIO 回调): 每 END 事件中断一次,
    // 回调内按 CRCSTATUS 区分 CRC OK/错. 不再主循环轮询, 无漏包窗口.
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    NVIC_EnableIRQ(RADIO_IRQn);
    // 连续接收: READY->START (ramp 完成自动开收), END->START (收完一包立即重新武装)
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos)
                      | (RADIO_SHORTS_END_START_Enabled   << RADIO_SHORTS_END_START_Pos);
    NRF_RADIO->TASKS_RXEN = 1;
}

// RADIO 中断回调里的收包处理 (优先级 0, 无日志/无 sd_* 调用).
// m_rx_buf 布局: [0]=S0(PDU头) [1]=LENGTH [2..]=payload [..]=CRC3(硬件校验后附加).
// AdvA 位于 payload[0..5] 小端, MSB 在 buf[7]; 广播数据在 buf[8..].
static void radio_irq_process(void) {
    if (NRF_RADIO->EVENTS_END == 0) return;
    NRF_RADIO->EVENTS_END = 0;
    m_rx_total++;
    if (NRF_RADIO->CRCSTATUS == 0) return; // CRC 错误直接丢弃
    m_rx_crc_ok++;
    // 匹配规则: 目标为 public 地址 (txadd=0) 且 OUI 前缀 78:81:8C (Nintendo 注册
    // OUI, Joy-Con 2). AdvA 小端, 78:81:8C 三字节位于 buf[7],buf[6],buf[5].
    // PDU 只认带 AdvA 且非定向的广播类型: ADV_IND(0)/ADV_NONCONN_IND(2)/
    // ADV_SCAN_IND(6), 排除定向包/扫描响应/扩展广播.
    if (m_sniff_done) return; // 首个命中即锁定
    uint8_t const pdu_type = m_rx_buf[0] & 0x0F;
    if (pdu_type != 0x00 && pdu_type != 0x02 && pdu_type != 0x06) return;
    uint8_t const len = m_rx_buf[1]; // PDU payload 长度 (含 AdvA)
    if (len < 6 || len > 37) return;
    if (m_rx_buf[0] & 0x40) return;  // TxAdd=1 (随机地址) 直接排除
    if (m_rx_buf[7] != VENDOR_OUI_B0
        || m_rx_buf[6] != VENDOR_OUI_B1
        || m_rx_buf[5] != VENDOR_OUI_B2
    ) return;
    m_rx_match++;
    // 命中: 地址与广播数据原样拷贝.
    memcpy(m_pair_cand.addr, &m_rx_buf[2], BLE_GAP_ADDR_LEN);
    m_pair_cand.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    m_pair_cand.data_len  = (uint8_t)(len - 6);
    memcpy(m_pair_cand.data, &m_rx_buf[8], m_pair_cand.data_len);
    m_sniff_done = true;
}

void bt_probe_stats_get(bt_probe_stats_t * p_stats) {
    p_stats->rx_total = m_rx_total;
    p_stats->rx_crc_ok = m_rx_crc_ok;
    p_stats->rx_match = m_rx_match;
    p_stats->cur_ch   = m_sniff_ch;
}

// 配对嗅探主流程: 单会话, 首个 EARLIEST 请求, 之后 TIMER0 回调 EXTEND 续片 (退路:
// 链式 NORMAL + BLOCKED 后 EARLIEST 恢复); 收包在 RADIO 中断内完成, 主循环只做
// 计时与 BLOCKED 恢复.
// 返回 true = 命中, *p_out 为候选记录 (尚未落盘, 持久化由调用方决定, 会话已关闭,
// 落盘安全).
bool bt_probe_run(pair_record_t * p_out, bt_probe_tick_fn_t tick) {
    ret_code_t err_code;
    uint32_t elapsed_ms = 0;
    memset(&m_pair_cand, 0, sizeof(m_pair_cand));
    m_rx_total = m_rx_crc_ok = m_rx_match = 0;
    m_sniff_done     = false;
    m_slot_blocked   = false;
    m_session_closed = false;
    m_sniff_ch       = 37;
    err_code = sd_radio_session_open(radio_signal_callback);
    APP_ERROR_CHECK(err_code);
    err_code = sd_radio_request(&m_first_request);
    APP_ERROR_CHECK(err_code);
    // 收包在 RADIO 中断内完成; 主循环只负责计时与 BLOCKED 恢复.
    while (!m_sniff_done && elapsed_ms < SCAN_WINDOW_MS) {
        if (m_slot_blocked) { // EXTEND/链式续片被拒: 退避后以 EARLIEST 重新发起
            m_slot_blocked = false;
            nrf_delay_ms(10);
            err_code = sd_radio_request(&m_first_request);
            APP_ERROR_CHECK(err_code);
        }
        nrf_delay_ms(1);
        elapsed_ms++;
        if (tick != NULL) tick(elapsed_ms);
    }
    // 关闭会话 (SD 结束当前片并取消已排队的续片); 等到 CLOSED 事件后射频才完全
    // 归还, 之后调用方才允许做 Flash 擦写 (NVMC 与射频互斥).
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
