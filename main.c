/**
 * @file    main.c
 * @brief   NS2_Waker —— JC 设备唤醒广播克隆发送器 (自 ESP32-C3 Arduino 版本移植).
 *          应用入口: 模式判定与流程编排. 功能模块:
 *            - persistence.c    配对记录 Flash 存取 (CRC16 + 掉电半写保护)
 *            - bt_probe.c       Radio Timeslot 嗅探配对 (S112 无扫描角色的唯一替代路径)
 *            - bt_advertising.c 克隆广播 (对端 MAC + 原样广播包)
 *
 * 双模式流程:
 *   - 配对模式 (开机按住唤醒键 >=1.5 s 进入; Flash 无有效记录时自动进入):
 *       嗅探 10 s 找 vendor 前缀 (78:81:8C, public) beacon, 命中后记录落盘.
 *       LED: 扫描中慢闪 (500 ms 周期) / 成功常亮 1 s / 失败快闪 5 次.
 *   - 常规模式 (短按唤醒或复位开机):
 *       读 Flash 配对记录, 克隆广播 1 s, 随后进入 System OFF.
 *
 * 硬件约束 (不可变更):
 *   - 电池直连 VDD (VDD 即内部稳压器输入, 电压须保持在 1.7 ~ 3.6 V);
 *   - 硬件 DCC 引脚未设计外部 LC 电路, 固件禁用 DC-DC, 使用内部 LDO 供电;
 *   - 休眠采用 System OFF 模式 (典型电流 ~0.4 uA @ 3 V), 通过 WAKEUP_BUTTON_PIN
 *     拉低或 RESET 复位唤醒 (两者本质都是复位, 程序从头执行, 开机采样按键电平
 *     区分模式).
 */

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"             // sd_power_*
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "persistence.h"
#include "bt_probe.h"
#include "bt_advertising.h"

#define WAKEUP_BUTTON_PIN 12 // 唤醒引脚低电平有效, 内部上拉; RESET 引脚复位同样可以唤醒 (已定义 CONFIG_GPIO_AS_PINRESET)
#define LED_PIN 6
#define DEAD_BEEF 0xDEADBEEF // 栈转储时用作错误码的值, 可用于定位栈回溯位置
#define PAIR_HOLD_MS 1500u   // 开机长按唤醒键超过该时长进入配对模式

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

// ============================================================================
// 协议栈初始化
// ============================================================================

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

// ============================================================================
// LED / 模式判定 / 休眠 (应用层 UI, 与射频/BLE 模块解耦)
// ============================================================================

// 失败提示: 快闪 5 次后由调用方进入 System OFF
static void led_error_blink(void) {
    for (uint8_t i = 0; i < 5; i++) {
        nrf_gpio_pin_clear(LED_PIN);
        nrf_delay_ms(80);
        nrf_gpio_pin_set(LED_PIN);
        nrf_delay_ms(80);
    }
}

// 嗅探主循环的 1 ms tick (bt_probe 回调, 线程上下文):
// LED 慢闪 + 每秒一条诊断日志, 经 bt_probe_stats_get 区分射频/CRC/过滤哪层出问题.
static void probe_tick(uint32_t elapsed_ms) {
    static uint32_t blink_div = 0;
    static uint32_t log_div   = 0;

    if (++blink_div >= 250) { // LED 慢闪, 500 ms 周期
        blink_div = 0;
        nrf_gpio_pin_toggle(LED_PIN);
    }
    if (++log_div >= 1000) {
        log_div = 0;
        bt_probe_stats_t stats;
        bt_probe_stats_get(&stats);
        NRF_LOG_INFO(
            "scan %u ms: rx=%u crcok=%u match=%u ch=%u", elapsed_ms,
            stats.rx_total, stats.rx_crc_ok, stats.rx_match, stats.cur_ch
        );
        NRF_LOG_PROCESS();
    }
}

// 模式判定 (须在 GPIO 上拉配置后调用):
//   无有效记录(首次上电/记录损坏) -> 配对;
//   唤醒键未按下 -> 常规;
//   按住持续满 1.5 s -> 配对 (中途松开立即返回常规, 常规唤醒只多等一次松键的时间).
static bool detect_pairing_mode(void) {
    pair_record_t rec;
    if (!persistence_load(&rec)) return true;
    if (nrf_gpio_pin_read(WAKEUP_BUTTON_PIN) != 0) return false;
    for (uint32_t i = 0; i < (PAIR_HOLD_MS / 10); i++) {
        nrf_delay_ms(10);
        if (nrf_gpio_pin_read(WAKEUP_BUTTON_PIN) != 0) return false;
    }
    return true;
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

// ============================================================================
// 主流程
// ============================================================================

int main(void) {
    ret_code_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    // GPIO 先于协议栈初始化: 复位后第一时间采样唤醒键电平判定模式.
    nrf_gpio_cfg_input(WAKEUP_BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
    // LED 开漏模式（无上拉、输出、高电平断开）, 初始熄灭
    nrf_gpio_cfg(LED_PIN,
        NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE
    );
    nrf_gpio_pin_set(LED_PIN);
    nrf_delay_ms(10); // 内部上拉稳定 + 按键去抖
    bool pairing = detect_pairing_mode();
    NRF_LOG_INFO("NS2 waker started, mode=%u", (uint32_t)pairing);
    ble_stack_init();
    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
    APP_ERROR_CHECK(err_code);
    if (pairing) {
        // 嗅探 -> 落盘: bt_probe_run 返回时会话已关闭, 之后写 Flash 安全
        // (三步写入掉电保护见 persistence.c).
        pair_record_t rec;
        bool paired = bt_probe_run(&rec, probe_tick);
        if (paired && !persistence_save(&rec)) {
            paired = false;
            NRF_LOG_INFO("flash save failed");
            NRF_LOG_FINAL_FLUSH();
        }
        if (paired) {
            NRF_LOG_INFO("pair record saved");
            nrf_gpio_pin_clear(LED_PIN); // 成功: 常亮 1 s 作为确认
            nrf_delay_ms(1000);
        } else led_error_blink(); // 超时/存储失败: 快闪 5 次
        enter_system_off();
    }

    // 常规模式: 读配对记录, 克隆地址与广播包, 广播 1 s 后休眠
    pair_record_t rec;
    if (!persistence_load(&rec)) { // 防御: 判定阶段已兜底转配对, 理论不可达
        led_error_blink();
        enter_system_off();
    }
    bt_advertising_init(&rec);
    nrf_gpio_pin_clear(LED_PIN); // 广播期间点亮 LED
    NRF_LOG_INFO("advertising cloned beacon, dlen=%u", rec.data_len);
    bt_advertising_start();
    for (;;) { // 主循环: 事件驱动, 广播结束后进入 System OFF.
        if (bt_advertising_is_done()) enter_system_off();
        if (!NRF_LOG_PROCESS()) nrf_pwr_mgmt_run();
    }
}
