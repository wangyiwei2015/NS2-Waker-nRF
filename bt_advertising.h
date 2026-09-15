/**
 * @file    bt_advertising.h
 * @brief   克隆广播: 按配对记录原样发送对端 MAC 与广播包.
 */
#ifndef BT_ADVERTISING_H__
#define BT_ADVERTISING_H__

#include <stdbool.h>
#include "persistence.h"

// 应用 BLE 配置 tag: main.c 的栈初始化 (nrf_sdh_ble_default_cfg_set) 与本模块
// 的广播启动 (sd_ble_gap_adv_start) 必须使用同一个值, 故集中定义在此.
#define APP_BLE_CONN_CFG_TAG 1

/**
 * 用配对记录初始化克隆广播:
 *   GAP 地址 = 抓到的对端 MAC (public, 原样); 广播数据 = 抓到的原样字节
 *   (含对方 Flags/厂商数据等完整 AD 结构, 不重新组包);
 *   PDU 类型固定 ADV_NONCONN_IND (非连接不可扫描, 典型 beacon 形态).
 * 前提: SoftDevice (BLE 栈) 已使能.
 */
void bt_advertising_init(pair_record_t const * p_rec);

// 启动广播 (时长 1 s, 由广播参数 duration 控制, 到期上报 ADV_SET_TERMINATED).
void bt_advertising_start(void);

// 广播是否已结束 (BLE_GAP_EVT_ADV_SET_TERMINATED 置位).
bool bt_advertising_is_done(void);

#endif // BT_ADVERTISING_H__
