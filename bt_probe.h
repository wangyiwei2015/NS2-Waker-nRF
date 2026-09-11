/**
 * @file    bt_probe.h
 * @brief   Radio Timeslot 嗅探配对: 定向找 vendor 前缀的 public 地址 beacon.
 */
#ifndef BT_PROBE_H__
#define BT_PROBE_H__

#include <stdbool.h>
#include <stdint.h>

#include "persistence.h"

/**
 * 嗅探主循环的周期回调, 约每 1 ms 调用一次 (线程上下文, 非中断).
 * 用于应用层 (main.c) 做 LED 指示/诊断日志等 UI 逻辑, 保持射频与 UI 解耦.
 */
typedef void (*bt_probe_tick_fn_t)(uint32_t elapsed_ms);

// 嗅探诊断计数 —— 用于区分射频/CRC/过滤哪一层出问题
typedef struct {
    uint32_t rx_total;  // 收到的包总数 (含 CRC 错)
    uint32_t rx_crc_ok; // CRC 正确的包数
    uint32_t rx_match;  // 前缀命中数
    uint8_t  cur_ch;    // 当前监听信道 (37/38/39)
} bt_probe_stats_t;

/**
 * 配对嗅探主流程 (含 Radio Timeslot 会话生命周期管理):
 *   打开会话 -> 循环 (收包 / 轮换信道请求新 slot / 每 ms 回调 tick) -> 关会话.
 *
 * 前提: SoftDevice 已使能.
 * 返回 true = 命中目标, *p_out 为候选记录 (尚未落盘, 持久化由调用方决定,
 * Flash 策略与射频逻辑解耦); 返回时会话必然已关闭, 之后做 Flash 擦写是安全的.
 * 返回 false = 超时未命中或会话异常.
 */
bool bt_probe_run(pair_record_t * p_out, bt_probe_tick_fn_t tick);

// 读取诊断计数 (可在 tick 回调里调用)
void bt_probe_stats_get(bt_probe_stats_t * p_stats);

#endif // BT_PROBE_H__
