/**
 * @file pi_frame.h
 * @brief 帧解析器接口
 * @author Bubble
 * @date 2026-01-27
 */

#ifndef PI_FRAME_H
#define PI_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "pi_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 帧结构
// ============================================================================

typedef struct {
    uint8_t  seq;                           // 序列号
    uint8_t  cmd;                           // 命令码
    uint8_t  data[PI_FRAME_MAX_DATA_SIZE];  // 数据
    uint16_t data_len;                      // 数据长度
    uint16_t crc;                           // CRC
} pi_frame_t;

// ============================================================================
// 帧解析器状态
// ============================================================================

typedef enum {
    PARSER_STATE_WAIT_HEADER_0,
    PARSER_STATE_WAIT_HEADER_1,
    PARSER_STATE_WAIT_LEN,
    PARSER_STATE_WAIT_DATA,
    PARSER_STATE_WAIT_CRC_H,
    PARSER_STATE_WAIT_CRC_L,
} parser_state_t;

typedef struct {
    parser_state_t state;
    uint8_t  buf[PI_FRAME_MAX_SIZE];
    uint16_t buf_idx;
    uint16_t expected_len;
} pi_frame_parser_t;

// ============================================================================
// 函数声明
// ============================================================================

/**
 * @brief 初始化帧解析器
 */
void pi_frame_parser_init(pi_frame_parser_t *parser);

/**
 * @brief 重置帧解析器
 */
void pi_frame_parser_reset(pi_frame_parser_t *parser);

/**
 * @brief 向解析器输入一个字节
 * @param parser 解析器
 * @param byte 输入字节
 * @param frame 输出帧 (当解析完成时)
 * @return true 如果解析出一个完整帧
 */
bool pi_frame_parser_feed(pi_frame_parser_t *parser, uint8_t byte, pi_frame_t *frame);

/**
 * @brief 向解析器输入多个字节
 * @param parser 解析器
 * @param data 数据
 * @param len 长度
 * @param frame 输出帧
 * @return 解析出的帧数量
 */
int pi_frame_parser_feed_bytes(pi_frame_parser_t *parser, const uint8_t *data, 
                                size_t len, pi_frame_t *frame);

/**
 * @brief 构建帧
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @param seq 序列号
 * @param cmd 命令码
 * @param data 数据
 * @param data_len 数据长度
 * @return 帧长度，0 表示失败
 */
size_t pi_frame_build(uint8_t *buf, size_t buf_size, uint8_t seq, uint8_t cmd,
                      const uint8_t *data, size_t data_len);

/**
 * @brief 计算 CRC16-CCITT
 * @param data 数据
 * @param len 长度
 * @return CRC16 值
 */
uint16_t pi_crc16_ccitt(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // PI_FRAME_H
