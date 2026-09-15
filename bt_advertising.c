/**
 * @file    bt_advertising.c
 * @brief   BLE 克隆广播实现.
 *
 * 设计决策:
 *   1. 克隆语义: sd_ble_gap_addr_set 设为抓到的 public MAC, 广播数据直接
 *      memcpy 抓到的原样字节 (目标包的 AD 结构), 不做任何重新组包编码.
 *   2. PDU 类型由广播参数固定为 ADV_NONCONN_IND(0x02, 非连接不可扫描), 与典型
 *      beacon 一致; 若源 beacon 是可连接的 ADV_IND, 空中仅 PDU 头一个字节不同,
 *      地址与数据完全一致.
 *   3. 发射功率必须是 S112 支持的 9 个档位之一 (-40/-20/-16/-12/-8/-4/0/+3/+4),
 *      否则 sd_ble_gap_tx_power_set 返回 NRF_ERROR_INVALID_PARAM;
 *      nRF52810 上限 +4 dBm.
 */
#include "bt_advertising.h"

#include <string.h>
#include "app_error.h"
#include "app_util.h"
#include "nrf_log.h"
#include "nrf_sdh_ble.h"

/*
 发射功率距离对照 (参考):
 +4 0dB MAX <30m
 +3  -1dB  9-27m
  0  -4dB  6-20m
 -4  -8dB  4-12m
 -8 -12dB   3-8m
-12 -16dB   2-5m
-16 -20dB   1-3m
*/
#define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(100, UNIT_0_625_MS) // 广播间隔 100 ms = 不可连接广播按蓝牙协议的最小值
#define ADV_DURATION_10MS 100 // 广播时长单位: 10ms, 即 1 s, 到期后停止广播并上报 BLE_GAP_EVT_ADV_SET_TERMINATED
#define TX_POWER_LEVEL -4 // 发射功率 (dBm, S112 支持档位之一)

static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  // 广播数据缓冲区 (31 字节)
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
static ble_gap_adv_params_t m_adv_params;
static ble_gap_adv_data_t m_adv_data = {
    .adv_data = {
        .p_data = m_enc_advdata, .len = 0
    }, .scan_rsp_data = {
        .p_data = NULL, .len = 0
    }
};

static volatile bool m_advertising_done = false; // 广播结束标志, 在 BLE 事件回调中置位

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            NRF_LOG_INFO("Advertising terminated after 1 s");
            m_advertising_done = true;
            break;
        default: break; // 无需处理其它事件
    }
}

NRF_SDH_BLE_OBSERVER(m_ble_observer, 3, ble_evt_handler, NULL);

void bt_advertising_init(pair_record_t const * p_rec) {
    ret_code_t     err_code;
    ble_gap_addr_t gap_addr;
    // 1. GAP 地址 = 配对时抓到的对端 MAC (public, 原样).
    memset(&gap_addr, 0, sizeof(gap_addr));
    gap_addr.addr_type = p_rec->addr_type;
    memcpy(gap_addr.addr, p_rec->addr, BLE_GAP_ADDR_LEN);
    err_code = sd_ble_gap_addr_set(&gap_addr);
    APP_ERROR_CHECK(err_code);
    // 2. 广播数据 = 抓到的原样字节 (含对方 Flags/厂商数据等完整 AD 结构).
    memcpy(m_enc_advdata, p_rec->data, p_rec->data_len);
    m_adv_data.adv_data.len = p_rec->data_len;
    // 3. 广播参数: 非连接不可扫描无向 (PDU = ADV_NONCONN_IND, 典型 beacon 形态).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL; // 无向广播.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = ADV_DURATION_10MS;
    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

void bt_advertising_start(void) {
    ret_code_t err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}

bool bt_advertising_is_done(void) { return m_advertising_done; }
