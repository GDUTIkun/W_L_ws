#ifndef __UART_PROTOCOL_TEST_H
#define __UART_PROTOCOL_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t rx_bytes;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t sync_losses;
    uint32_t rx_seq_gaps;
    uint32_t uart_errors;
    uint32_t tx_frames;
    uint32_t tx_bytes;
    uint32_t tx_errors;
    uint32_t tx_busy_errors;
    uint32_t tx_timeout_errors;
    uint32_t tx_hal_error_errors;
    uint16_t last_seq;
    uint16_t last_tx_seq;
    uint8_t last_type;
    uint8_t last_len;
    uint32_t last_frame_tick_ms;
    uint32_t min_frame_gap_ms;
    uint32_t max_frame_gap_ms;
    uint32_t tx_skip_in_flight;
} UartProtocolTestStats;

extern volatile UartProtocolTestStats uart2_protocol_test_stats;

void UartProtocolTest_Init(void);
void UartProtocolTest_Process(void);
void UartProtocolTest_GetStats(UartProtocolTestStats *out);
void UartProtocolTest_ResetStats(void);
void UartProtocolTest_FillActuatorCommand(float *out_efforts, uint32_t effort_count);
uint8_t UartProtocolTest_GetSafetyState(void);

#ifdef __cplusplus
}
#endif

#endif
