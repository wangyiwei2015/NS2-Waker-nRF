/**
 * @file    main.c
 * @brief   NS2_Waker —— JC 设备唤醒广播发送器 (自 ESP32-C3 Arduino 版本移植).
 *   - 广播后由 SoftDevice 自动停止 (BLE_GAP_EVT_ADV_SET_TERMINATED 事件), 随后进入休眠;
 *   - 电池直连 VDD (VDD 即内部稳压器输入, 电压须保持在 1.7 ~ 3.6 V);
 *   - 硬件 DCC 引脚未设计外部 LC 电路, 固件禁用 DC-DC, 使用内部 LDO 供电;
 *   - 休眠采用 System OFF 模式 (典型电流 ~0.4 uA @ 3 V), 通过 WAKEUP_BUTTON_PIN 拉低或 RESET 复位唤醒.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
// #include "nordic_common.h"
#include "app_util.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*
 +4 0dB MAX <30m
 +3  -1dB  9-27m
  0  -4dB  6-20m
 -4  -8dB  4-12m
 -8 -12dB   3-8m
-12 -16dB   2-5m
-16 -20dB   1-3m
*/

#define APP_BLE_CONN_CFG_TAG 1 // 标识 SoftDevice BLE 配置的 tag
#define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(100, UNIT_0_625_MS) // 广播间隔 100 ms = 不可连接广播按蓝牙协议的最小值
#define ADV_DURATION_10MS 100 // 广播时长单位: 10ms, 1s, 到期后停止广播并上报 BLE_GAP_EVT_ADV_SET_TERMINATED
#define TX_POWER_LEVEL -4 // 发射功率
#define WAKEUP_BUTTON_PIN 12 // 唤醒引脚低电平有效, 内部上拉; RESET 引脚复位同样可以唤醒 (已定义 CONFIG_GPIO_AS_PINRESET)
#define LED_PIN 6
#define DEAD_BEEF 0xDEADBEEF // 栈转储时用作错误码的值, 可用于定位栈回溯位置

// 蓝牙地址,小端格式: [0] = LSB ... [5] = MSB
static const uint8_t target_ble_addr[BLE_GAP_ADDR_LEN] = {0x38, 0xB7, 0xD6, 0x8C, 0x81, 0x78};
#define WAKE_COMPANY_ID 0x0553 // 小端编码后线上字节顺序一致
static uint8_t m_wake_packet[] = {
    0x01, 0x00, 0x03, 0x7E, 0x05, 0x66, 0x20, 0x00,
    0x01, 0x81, 0x4F, 0xF4, 0xE0, 0x8C, 0x81, 0x78,
    0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static ble_gap_adv_params_t m_adv_params;
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  // 广播数据缓冲区 31字节恰好装满

static ble_gap_adv_data_t m_adv_data = {
    .adv_data = {
        .p_data = m_enc_advdata, .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    }, .scan_rsp_data = {
        .p_data = NULL, .len = 0
    }
};

static volatile bool m_advertising_done = false; // 广播结束标志, 在 BLE 事件回调中置位
ret_code_t err_code;

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            NRF_LOG_INFO("Advertising terminated after 1.5 s");
            m_advertising_done = true;
            break;
        default: break; // 无需处理其它事件
    }
}

NRF_SDH_BLE_OBSERVER(m_ble_observer, 3, ble_evt_handler, NULL);

static void advertising_init(void) {
    ret_code_t               err_code;
    ble_advdata_t            advdata;
    ble_advdata_manuf_data_t manuf_specific_data;
    ble_gap_addr_t           gap_addr;
    // 1. 蓝牙地址必须在启动广播前设置.
    memset(&gap_addr, 0, sizeof(gap_addr));
    gap_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    memcpy(gap_addr.addr, target_ble_addr, BLE_GAP_ADDR_LEN);
    err_code = sd_ble_gap_addr_set(&gap_addr);
    APP_ERROR_CHECK(err_code);
    // 2. 厂商数据: 公司 ID + 24 字节唤醒报文.
    manuf_specific_data.company_identifier = WAKE_COMPANY_ID;
    manuf_specific_data.data.p_data        = m_wake_packet;
    manuf_specific_data.data.size          = sizeof(m_wake_packet);
    // 3. 广播数据: Flags 0x06 (LE General Discoverable + BR/EDR Not Supported)
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data = &manuf_specific_data;
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL; // 无向广播.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = ADV_DURATION_10MS;
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

static void ble_stack_init(void) {
    ret_code_t err_code;
    uint32_t   ram_start = 0;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    // 使用默认配置, 并取得应用 RAM 起始地址.
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

static void enter_system_off(void) {
    NRF_LOG_INFO("Entering System OFF, wake by P0.%u low or RESET", WAKEUP_BUTTON_PIN);
    NRF_LOG_FINAL_FLUSH();
    // 等待唤醒键释放, 避免仍按住时 DETECT 信号立即再次唤醒.
    while (nrf_gpio_pin_read(WAKEUP_BUTTON_PIN) == 0) nrf_delay_ms(10);
    // 关灯并重置引脚到高阻悬空态（断开所有驱动和输入缓冲）
    nrf_gpio_cfg_default(LED_PIN);
    // 配置唤醒引脚: 上拉输入 + 低电平 SENSE, System OFF 下拉低即唤醒.
    nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    sd_power_system_off(); // 进入 System OFF, 本调用不会返回.
    for (;;) {} // 不应执行到这里.
}

int main(void) {
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    ble_stack_init();
    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
    APP_ERROR_CHECK(err_code);
    advertising_init();
    nrf_gpio_cfg_input(WAKEUP_BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
    // 配置为开漏模式（无上拉、输出、高电平断开）
    nrf_gpio_cfg(LED_PIN,
        NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, // 即开漏模式
        NRF_GPIO_PIN_NOSENSE
    ); nrf_gpio_pin_clear(LED_PIN); // 点亮 LED
    NRF_LOG_INFO("NS2 waker started, advertising");
    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    for (;;) { // 主循环: 事件驱动, 广播结束后进入 System OFF.
        if (m_advertising_done) enter_system_off();
        if (!NRF_LOG_PROCESS()) nrf_pwr_mgmt_run();
    }
}
