#pragma once
#include <cstdint>
#include <cstddef>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef int uart_port_t;
typedef enum { UART_DATA_8_BITS } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE } uart_parity_t;
typedef enum { UART_STOP_BITS_1 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE } uart_hw_flowcontrol_t;
typedef enum { UART_SCLK_DEFAULT } uart_sclk_t;
typedef struct {
    int baud_rate;
    uart_word_length_t data_bits;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_hw_flowcontrol_t flow_ctrl;
    uint8_t rx_flow_ctrl_thresh;
    uart_sclk_t source_clk;
} uart_config_t;
#define UART_PIN_NO_CHANGE (-1)
esp_err_t uart_driver_install(uart_port_t, int, int, int, void*, int);
esp_err_t uart_param_config(uart_port_t, const uart_config_t*);
esp_err_t uart_set_pin(uart_port_t, int, int, int, int);
int uart_read_bytes(uart_port_t, void*, uint32_t, TickType_t);
int uart_write_bytes(uart_port_t, const void*, size_t);
