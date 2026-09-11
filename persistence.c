/**
 * @file    persistence.c
 * @brief   配对记录 Flash 读写: 末页存储 + CRC16 校验 + 掉电半写保护.
 *
 * 设计决策:
 *   1. 记录放 Flash 末页 0x2F000 (页 47, 4KB): 链接脚本把应用区缩短 0x1000
 *      让出该页; make flash 的 --sectorerase 只擦应用 hex 覆盖的扇区, 记录页
 *      不在其中, 因此升级固件不会丢配对记录.
 *   2. 掉电半写保护 ("dirty flag" 的最省实现): 记录分三步写入 —— 页擦除 ->
 *      数据+CRC16 共 11 个字 -> magic 最后单独写入. 任意时刻掉电, magic 都不会
 *      就位, 上电校验必然失败并自动回到配对模式. 不做 A/B 双槽 (配对频率极低,
 *      属过度设计).
 *   3. sd_flash_* 为异步 API: 完成事件经 SOC 观察者 (SWI2 中断上下文) 置标志,
 *      flash_wait 忙等收拢. 擦写期间 NVMC 被占用, 与 Radio Timeslot 射频互斥,
 *      调用方须保证嗅探会话已关闭 (bt_probe_run 返回后即满足).
 */
#include "persistence.h"

#include <string.h>
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_sdh_soc.h"  // SOC 事件观察者
#include "nrf_soc.h"      // sd_flash_* API
#include "ble_gap.h"      // BLE_GAP_ADDR_TYPE_PUBLIC
#include "crc16.h"

#define PAIR_RECORD_ADDR   0x2F000u                   // Flash 末页 (应用区之外, 见链接脚本)
#define PAIR_PAGE_NUM      (PAIR_RECORD_ADDR / 4096u) // sd_flash_page_erase 参数是页号: 47
#define PAIR_MAGIC         0x4E533257u                // "NS2W" 提交标记, 最后写入

static volatile bool m_flash_success; // SOC: sd_flash_* 操作完成
static volatile bool m_flash_error;   // SOC: sd_flash_* 操作失败

// SOC 事件 (Flash 操作结果), SWI2 中断上下文 -> 只置标志
static void soc_evt_handler(uint32_t evt_id, void * p_context) {
    switch (evt_id) {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        m_flash_success = true; break;
        case NRF_EVT_FLASH_OPERATION_ERROR:
        m_flash_error = true; break;
        default: break;
    }
}

NRF_SDH_SOC_OBSERVER(m_soc_observer, 0, soc_evt_handler, NULL);

// 等待当前 sd_flash_* 操作的完成事件 (SOC 事件在 SWI2 中断上下文置位标志).
// 超时按最坏情况取值: 页擦除 <=500 ms, 字写入 <=200 ms.
static bool flash_wait(uint32_t timeout_ms) {
    while (!m_flash_success && !m_flash_error) {
        nrf_delay_ms(1);
        if (--timeout_ms == 0) return false;
    }
    return m_flash_success;
}

// 三步写入: 页擦除 -> 数据+CRC (11 字) -> magic (1 字, 最后).
// 掉电半写保护: magic 不落盘则记录必无效, 上电自动回配对模式.
bool persistence_save(pair_record_t const * p_rec) {
    ret_code_t     err_code;
    pair_record_t  rec;
    uint32_t       magic_word = PAIR_MAGIC;

    memcpy(&rec, p_rec, sizeof(rec));
    rec.pad0  = 0;
    memset(rec.pad1, 0, sizeof(rec.pad1));
    rec.crc16 = crc16_compute((uint8_t const *)&rec, 40, NULL);
    rec.magic = PAIR_MAGIC;

    m_flash_success = false;
    m_flash_error   = false;
    err_code = sd_flash_page_erase(PAIR_PAGE_NUM);
    APP_ERROR_CHECK(err_code);
    if (!flash_wait(500)) return false;

    // 数据+CRC 共 44 字节 (magic 字段不在本次写入范围内)
    m_flash_success = false;
    m_flash_error   = false;
    err_code = sd_flash_write((uint32_t *)PAIR_RECORD_ADDR, (uint32_t const *)&rec, 11);
    APP_ERROR_CHECK(err_code);
    if (!flash_wait(200)) return false;

    // magic 恒最后单独写入 —— 掉电半写保护的关键
    m_flash_success = false;
    m_flash_error   = false;
    err_code = sd_flash_write((uint32_t *)(PAIR_RECORD_ADDR + 44), &magic_word, 1);
    APP_ERROR_CHECK(err_code);
    if (!flash_wait(200)) return false;
    return true;
}

// 有效性 = magic 就位 + 类型/长度合法 + CRC16 通过.
// magic 缺失即覆盖"从未配对"与"写入中途掉电"两种情况.
bool persistence_load(pair_record_t * p_rec) {
    pair_record_t const * p_flash = (pair_record_t const *)PAIR_RECORD_ADDR;
    if (p_flash->magic != PAIR_MAGIC) return false;
    if (p_flash->addr_type != BLE_GAP_ADDR_TYPE_PUBLIC) return false;
    if (p_flash->data_len > sizeof(p_flash->data)) return false;
    if (p_flash->crc16 != crc16_compute((uint8_t const *)p_flash, 40, NULL)) return false;
    memcpy(p_rec, p_flash, sizeof(pair_record_t));
    return true;
}
