/**
 * @file    persistence.h
 * @brief   配对记录 (对端 MAC + 原样广播包) 的 Flash 持久化接口.
 */
#ifndef PERSISTENCE_H__
#define PERSISTENCE_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * 配对记录 (48 字节 = 12 个 32bit 字, 天然 4 字节对齐).
 * 布局刻意保证各成员自然对齐 (crc16 在偶数偏移, magic 在 4 倍数偏移),
 * 使结构体可直接按字数组写给 sd_flash_write.
 */
typedef struct {
    uint8_t  addr[6];   // 对端 MAC, 小端, 与 ble_gap_addr_t.addr 一致
    uint8_t  addr_type; // BLE_GAP_ADDR_TYPE_PUBLIC (本方案只配 public 地址)
    uint8_t  data_len;  // 广播数据长度 (PDU payload 去掉 AdvA, 0~31)
    uint8_t  data[31];  // 广播数据原样克隆
    uint8_t  pad0;      // 填充, 参与 CRC, 写入前清零
    uint16_t crc16;     // 覆盖前 40 字节
    uint8_t  pad1[2];   // 填充
    uint32_t magic;     // 提交标记, 恒最后写入 —— 掉电半写保护的关键
} pair_record_t;

// 读记录: 有效性 = magic 就位 + 类型/长度合法 + CRC16 通过.
// magic 缺失即覆盖"从未配对"与"写入中途掉电"两种情况.
// 直接读内存映射 Flash, SoftDevice 未使能时也可调用 (开机模式判定用).
bool persistence_load(pair_record_t * p_rec);

// 写记录 (三步写入掉电保护, 见 persistence.c).
// 须 SoftDevice 已使能, 且无 Radio Timeslot 会话 (NVMC 擦写与射频互斥).
bool persistence_save(pair_record_t const * p_rec);

#endif // PERSISTENCE_H__
