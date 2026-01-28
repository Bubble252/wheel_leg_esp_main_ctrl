/**
 * @file pi_frame.c
 * @brief 帧解析器实现
 * @author Bubble
 * @date 2026-01-27
 */

#include "pi_frame.h"
#include <string.h>

// ============================================================================
// CRC16-CCITT 计算
// ============================================================================

uint16_t pi_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

// ============================================================================
// 帧解析器
// ============================================================================

void pi_frame_parser_init(pi_frame_parser_t *parser) {
    memset(parser, 0, sizeof(pi_frame_parser_t));
    parser->state = PARSER_STATE_WAIT_HEADER_0;
}

void pi_frame_parser_reset(pi_frame_parser_t *parser) {
    parser->state = PARSER_STATE_WAIT_HEADER_0;
    parser->buf_idx = 0;
    parser->expected_len = 0;
}

bool pi_frame_parser_feed(pi_frame_parser_t *parser, uint8_t byte, pi_frame_t *frame) {
    switch (parser->state) {
        case PARSER_STATE_WAIT_HEADER_0:
            if (byte == PI_FRAME_HEADER_0) {
                parser->buf[0] = byte;
                parser->buf_idx = 1;
                parser->state = PARSER_STATE_WAIT_HEADER_1;
            }
            break;
            
        case PARSER_STATE_WAIT_HEADER_1:
            if (byte == PI_FRAME_HEADER_1) {
                parser->buf[1] = byte;
                parser->buf_idx = 2;
                parser->state = PARSER_STATE_WAIT_LEN;
            } else if (byte == PI_FRAME_HEADER_0) {
                // 可能是新帧的开始
                parser->buf[0] = byte;
                parser->buf_idx = 1;
            } else {
                pi_frame_parser_reset(parser);
            }
            break;
            
        case PARSER_STATE_WAIT_LEN:
            // LEN = SEQ(1) + CMD(1) + DATA(0-250) 的长度
            if (byte >= 2 && byte <= 252) {
                parser->buf[2] = byte;
                parser->buf_idx = 3;
                parser->expected_len = byte;  // 需要接收的字节数
                parser->state = PARSER_STATE_WAIT_DATA;
            } else {
                pi_frame_parser_reset(parser);
            }
            break;
            
        case PARSER_STATE_WAIT_DATA:
            parser->buf[parser->buf_idx++] = byte;
            if (parser->buf_idx >= (size_t)(3 + parser->expected_len)) {
                parser->state = PARSER_STATE_WAIT_CRC_H;
            }
            break;
            
        case PARSER_STATE_WAIT_CRC_H:
            parser->buf[parser->buf_idx++] = byte;
            parser->state = PARSER_STATE_WAIT_CRC_L;
            break;
            
        case PARSER_STATE_WAIT_CRC_L:
            parser->buf[parser->buf_idx++] = byte;
            
            // 解析帧
            uint8_t len = parser->buf[2];
            uint16_t recv_crc = ((uint16_t)parser->buf[3 + len] << 8) | 
                                parser->buf[3 + len + 1];
            
            // 计算 CRC (对 SEQ + CMD + DATA)
            uint16_t calc_crc = pi_crc16_ccitt(&parser->buf[3], len);
            
            if (recv_crc == calc_crc) {
                // CRC 正确，填充帧结构
                frame->seq = parser->buf[3];
                frame->cmd = parser->buf[4];
                frame->data_len = len - 2;  // 减去 SEQ 和 CMD
                if (frame->data_len > 0) {
                    memcpy(frame->data, &parser->buf[5], frame->data_len);
                }
                frame->crc = recv_crc;
                
                pi_frame_parser_reset(parser);
                return true;
            } else {
                // CRC 错误
                pi_frame_parser_reset(parser);
            }
            break;
    }
    
    return false;
}

int pi_frame_parser_feed_bytes(pi_frame_parser_t *parser, const uint8_t *data, 
                                size_t len, pi_frame_t *frame) {
    int count = 0;
    
    for (size_t i = 0; i < len; i++) {
        if (pi_frame_parser_feed(parser, data[i], frame)) {
            count++;
            // 注意：这里只返回最后一个解析出的帧
            // 如果需要处理多帧，应该使用回调
        }
    }
    
    return count;
}

// ============================================================================
// 帧构建
// ============================================================================

size_t pi_frame_build(uint8_t *buf, size_t buf_size, uint8_t seq, uint8_t cmd,
                      const uint8_t *data, size_t data_len) {
    // 计算总长度: HEAD(2) + LEN(1) + SEQ(1) + CMD(1) + DATA + CRC(2)
    size_t total_len = 2 + 1 + 1 + 1 + data_len + 2;
    
    if (buf_size < total_len || data_len > PI_FRAME_MAX_DATA_SIZE) {
        return 0;
    }
    
    // 帧头
    buf[0] = PI_FRAME_HEADER_0;
    buf[1] = PI_FRAME_HEADER_1;
    
    // 长度 (SEQ + CMD + DATA)
    buf[2] = 2 + data_len;
    
    // SEQ + CMD
    buf[3] = seq;
    buf[4] = cmd;
    
    // 数据
    if (data_len > 0 && data != NULL) {
        memcpy(&buf[5], data, data_len);
    }
    
    // 计算 CRC (对 SEQ + CMD + DATA)
    uint16_t crc = pi_crc16_ccitt(&buf[3], 2 + data_len);
    
    // CRC (大端序)
    buf[5 + data_len] = (crc >> 8) & 0xFF;
    buf[5 + data_len + 1] = crc & 0xFF;
    
    return total_len;
}
