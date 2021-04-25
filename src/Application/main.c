/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "app_uart.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "board_spi.h"
#include "GUI_Paint.h"
#include "multi_button.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define SPI_INSTANCE  0 /**< SPI instance index. */
// static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
// static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
// static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

// e-Paper pin
#define EPD_DC_PIN 42
#define EPD_CS_PIN 43
#define EPD_RST_PIN 38
#define EPD_BUSY_PIN 36
// Display resolution
#define EPD_1IN54_V2_WIDTH       200
#define EPD_1IN54_V2_HEIGHT      200

// uart
#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

const unsigned char gImage_1in54[5000] = { /* 0X00,0X01,0XC8,0X00,0XC8,0X00, */
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFE,0X01,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF1,0XF8,0X00,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,
0X3F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XF0,0X00,
0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE1,0XFE,0X1F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XF0,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE3,0XFF,0X1F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,
0XE0,0X00,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE3,0XFF,0X9F,0XFF,0XFF,0XFF,0XFF,
0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XE1,0XE3,0X00,0X00,0X00,0X00,0X01,
0XFF,0XFF,0XE7,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF1,0XC3,0XF3,0X0F,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0XE3,0XFF,0X8F,0XFF,0XFF,
0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XC3,0X3B,0X0F,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFF,0XE3,0XFF,0X1F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF1,0XC3,0X0F,0X0F,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XE1,0XFE,0X1F,
0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XC3,0X0F,0X0F,
0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XF0,0XFC,0X1F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XE3,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XC3,0X87,0X0F,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XF0,
0X00,0X3F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XE0,0X7F,0XFF,0XFF,0XFF,0XFF,0XF1,0XC1,
0X03,0X0F,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XF8,0X00,0X7F,0XFF,0XFF,0XFF,0XFF,0X8F,
0XFF,0XE0,0X0F,0XFF,0XFF,0XFF,0XFF,0XF1,0XE0,0X00,0X1F,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFF,0XFE,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFC,0X03,0XFF,0XFF,0XFF,0XFF,
0XF1,0XE0,0X00,0X1F,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0X8F,0XFF,0XFF,0X80,0X7F,0XFF,0XFF,0XFF,0XF1,0XF0,0X00,0X3F,0XFF,0XFF,0XFF,
0X7E,0XFF,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XF0,0X1F,0XFF,
0XFF,0XFF,0XF1,0XF8,0X00,0X7F,0XFF,0XFF,0XFC,0X3E,0XFF,0XFF,0XE0,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0X00,0X1F,0XFF,0XFF,0XFF,0XF1,0XFE,0X00,0XFF,0XFF,
0XFF,0XF0,0X3E,0XFF,0XFF,0XC6,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XF8,0X07,
0X8F,0XFF,0XFF,0XFF,0XF1,0XFE,0X87,0XFF,0XFF,0XFF,0XE0,0XCE,0XFF,0XFF,0XCE,0X7F,
0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XE0,0X1F,0X8F,0XC0,0X07,0XFF,0XF1,0XFE,0XFF,
0XFF,0XFF,0XFF,0X81,0X86,0XFF,0XFF,0XCE,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,
0XE0,0XFF,0XCF,0X80,0X07,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0X02,0X06,0XFF,0XFF,
0XC6,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XE3,0XFF,0XEF,0X00,0X07,0XFF,0XF1,
0XFE,0XFF,0XFF,0XFF,0XFE,0X0C,0X02,0XFF,0XFF,0XE0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X8F,0XFF,0XFF,0XFF,0XFF,0X1F,0XFF,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFE,0X18,0X02,
0XFF,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0X1F,0XFF,
0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFE,0X20,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0X8F,0XFF,0XFE,0X01,0XFF,0X1F,0XFF,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFE,
0XC0,0X02,0XFF,0XFF,0XFF,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XF0,0X01,0XFF,
0X9F,0XFF,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0X00,0X02,0XFF,0XFF,0XF8,0X00,0X7F,
0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XE0,0X01,0XE0,0X80,0X0F,0XFF,0XF1,0XFE,0XFF,0XFF,
0XFF,0XFF,0X00,0X02,0XFF,0XFF,0XF0,0X00,0X3F,0XFF,0XE7,0XFF,0XFF,0X8F,0XFF,0XE0,
0X03,0XE0,0X00,0X07,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0X80,0X06,0XFF,0XFF,0XE0,
0XCC,0X1F,0XFF,0XE0,0XFF,0XFF,0X8F,0XFF,0XE2,0X71,0XE0,0X00,0X07,0XFF,0XF1,0XFE,
0XFF,0XFF,0XFF,0XFF,0XC0,0X0E,0XFF,0XFF,0XE3,0XC7,0X1F,0XFF,0XE0,0X3F,0XFF,0X8F,
0XFF,0XE7,0X39,0XE0,0X00,0X0F,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0XC0,0X0E,0XFF,
0XFF,0XE7,0XE7,0X8F,0XFF,0XF0,0X07,0XFF,0X8F,0XFF,0XE7,0X38,0XFF,0XFF,0XFF,0XFF,
0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0X00,0X1E,0XFF,0XFF,0XE7,0XE7,0X8F,0XFF,0XFE,0X01,
0XFF,0X8F,0XFF,0XE3,0X10,0XFF,0XFF,0XFF,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFC,0X00,
0X3E,0XFF,0XFF,0XE3,0XC7,0X8F,0XFF,0XFF,0XC0,0X7F,0X8F,0XFF,0XE3,0X01,0XFF,0X1F,
0XC7,0XFF,0XF1,0XFE,0XFF,0X83,0XFF,0XF0,0X00,0X7E,0XFF,0XFF,0XE0,0X07,0X1F,0XFF,
0XFF,0XF8,0X3F,0X8F,0XFF,0XF3,0X81,0XFF,0X1F,0XC7,0XFF,0XF1,0XFE,0XFC,0X01,0XFF,
0XE0,0X00,0XFE,0XFF,0XFF,0XF0,0X0F,0X1F,0XFF,0XFF,0XC0,0X1F,0X8F,0XFF,0XFF,0XC7,
0XFF,0X1F,0XC7,0XFF,0XF1,0XFE,0XF0,0X07,0XFF,0X80,0X01,0XFE,0XFF,0XFF,0XF8,0X1F,
0XBF,0XFF,0XFE,0X00,0X0F,0X8F,0XFF,0XFF,0XFF,0XF8,0X00,0X07,0XFF,0XF1,0XFE,0XC0,
0X18,0XFE,0X00,0X03,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X07,0X87,0X8F,0XFF,
0XFF,0XFF,0XF8,0X00,0X0F,0XFF,0XF1,0XFE,0XC0,0XE0,0XF8,0X00,0X0F,0XFE,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XE0,0X1F,0XC7,0X8F,0XFF,0XFF,0XFF,0XF8,0X00,0X0F,0XFF,0XF1,
0XFE,0XC7,0X00,0XE0,0X00,0X1F,0XFE,0XFF,0XFF,0XFF,0XFF,0X9F,0XFF,0XE0,0XFF,0XC7,
0X8F,0XFE,0X00,0X01,0XFF,0X1E,0XFF,0XFF,0XF1,0XFE,0XDC,0X00,0XC0,0X00,0X3F,0XFE,
0XFF,0XFF,0XF8,0X3F,0X1F,0XFF,0XE3,0XFF,0XC7,0X8F,0XFE,0X00,0X01,0XFF,0X1F,0XFF,
0XFF,0XF1,0XFE,0XE0,0X00,0X00,0X00,0XFF,0XFE,0XFF,0XFF,0XF0,0X0F,0X1F,0XFF,0XFF,
0XFF,0XC7,0X8F,0XFE,0X00,0X01,0XFF,0XFF,0XFF,0XFF,0XF1,0XFE,0XC0,0X00,0X00,0X03,
0XFF,0XFE,0XFF,0XFF,0XE0,0X03,0X1F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFE,0X03,0XFF,0XF8,
0X7F,0XFF,0XFF,0XF1,0XFE,0XE0,0X00,0X00,0X07,0XFF,0XFE,0XFF,0XFF,0XE3,0XC1,0X9F,
0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XC0,0X7F,0XF8,0X1F,0XFF,0XFF,0XF1,0XFE,0XE0,0X00,
0X00,0X1F,0XFF,0XFE,0XFF,0XFF,0XE7,0XE0,0X9F,0XFF,0XF8,0X00,0XFF,0X8F,0XFF,0XF8,
0X1F,0XF8,0X07,0XFF,0XFF,0XF1,0XFE,0XF0,0X00,0X00,0XFF,0XFF,0XFE,0XFF,0XFF,0XE7,
0XF8,0X1F,0XFF,0XF0,0X00,0XFF,0X8F,0XFF,0XFE,0X03,0XF8,0X01,0XFF,0XFF,0XF1,0XFE,
0XF8,0X00,0X03,0XFF,0XFF,0XFE,0XFF,0XFF,0XE7,0XFC,0X1F,0XFF,0XE0,0X00,0XFF,0X8F,
0XFF,0XFF,0X83,0XF8,0XC0,0X7F,0XFF,0XF1,0XFE,0XFC,0X00,0X3F,0XFF,0XFF,0XFE,0XFF,
0XFF,0XE3,0XFE,0X1F,0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFC,0X07,0XF8,0XF0,0X1F,0XFF,
0XF1,0XFE,0XFF,0XB7,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XF1,0XFF,0X1F,0XFF,0XE3,0X31,
0XFF,0X8F,0XFF,0XE0,0X3F,0XF8,0XFC,0X07,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFF,0XF3,0XFF,0X9F,0XFF,0XE3,0X38,0XFF,0X8F,0XFF,0X00,0XFF,0XF8,0XFF,
0X03,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE3,0X38,0XFF,0X8F,0XFE,0X07,0XFF,0XF8,0XFF,0XC3,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE3,0X18,0XFF,0X8F,0XFE,0X00,0X01,
0XF8,0XFF,0XF3,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,
0XFF,0XFF,0XE3,0X00,0XFF,0X8F,0XFE,0X00,0X01,0XF8,0XFF,0XFF,0XFF,0XF1,0XFE,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XE3,0X80,0XFF,0X8F,0XFE,
0X00,0X01,0XF8,0XFF,0XFF,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,
0XFF,0X87,0XFF,0XFF,0XF1,0X81,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,
0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XC3,0XFF,0XFF,0XFF,0XC3,0XFF,
0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,
0XFF,0XFF,0XFF,0XE3,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF1,0XFE,0X7F,0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XF3,0XFF,0XFF,0XFF,
0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0X00,0X00,0X00,0X00,
0X00,0X01,0XFF,0XFF,0XFF,0XE3,0XFF,0XFC,0X00,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE3,0XFF,
0XFC,0X00,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC3,0XFF,0XFC,0X00,0X00,0XFF,0X8F,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X87,0XFF,0XFC,0X00,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XE3,0XF1,0XFF,0X8F,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0X1F,0XFF,0XFF,0XE3,0XF9,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0XFF,0XFF,0XE3,0XF8,
0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,0XFF,0XE3,0XF8,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X3F,0XFF,0XFF,
0XE1,0XF0,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0XFF,0XFF,0XF0,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,
0XFF,0XFF,0XF0,0X01,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X0F,0XFF,0XFF,0XF8,0X01,0XFF,0X8F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0X87,0XFF,0XFF,0XFE,0X07,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XCF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,
0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0XFF,0XFF,0XF0,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF8,0X70,0X3F,
0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,
0XFF,0XFA,0XFF,0XFF,0XFF,0XFF,0XF0,0X20,0X1F,0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XC0,0X1F,0XFF,0XFF,0XFF,0XE0,
0X02,0X1F,0XFF,0XE3,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0X0F,0XFF,0XFF,0XF1,0XFF,
0XFF,0XFF,0XFF,0X00,0X0F,0XFF,0XFF,0XFF,0XE3,0X87,0X1F,0XFF,0XE3,0XFF,0XFF,0X8F,
0XFF,0XFF,0XFF,0XFF,0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFE,0X00,0X07,0XFF,0XFF,
0XFF,0XE7,0X8F,0X8F,0XFF,0XE3,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0X0F,0XFF,0XFF,
0XF1,0XFF,0XFF,0XFF,0XFE,0X00,0X02,0X7F,0XFF,0XFF,0XE7,0XCF,0X8F,0XFF,0XF0,0XFF,
0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFC,0X00,0X00,
0X7F,0XFF,0XFF,0XE7,0XCF,0X9F,0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFE,0X00,0X00,0X00,
0X3F,0XFF,0XF1,0XFF,0XFF,0XFF,0XF8,0X00,0X00,0XFF,0XFF,0XFF,0XE3,0XFF,0X1F,0XFF,
0XE0,0X00,0XFF,0X8F,0XFF,0XFE,0X00,0X00,0X00,0X3F,0XFF,0XF1,0XFF,0XFF,0XFF,0XF8,
0X30,0X00,0XFF,0XFF,0XFF,0XF3,0XFE,0X1F,0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFE,0X00,
0X00,0X00,0X3F,0XFF,0XF1,0XFF,0XFF,0XFF,0XF8,0X30,0X00,0XFF,0XFF,0XFF,0XF7,0XFF,
0X3F,0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFE,0X00,0X00,0X00,0X3F,0XFF,0XF1,0XFF,0XFF,
0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,
0XFE,0X00,0X00,0X00,0X3F,0XFF,0XF1,0XFF,0XFF,0XFF,0XF0,0X00,0X00,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0X01,0XFF,0X0F,0XFF,0XFF,0XF1,
0XFF,0XFF,0XFF,0X90,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0XFF,0XFF,0XFF,0XFF,
0X8F,0XFF,0XFF,0XC0,0X7F,0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0X10,0X00,0X00,0XFF,
0XFF,0XFF,0XF8,0X1F,0X1F,0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFF,0XE0,0X3F,0X0F,0XFF,
0XFF,0XF1,0XFF,0XFF,0XFC,0X10,0X00,0X00,0XFF,0XFF,0XFF,0XF0,0X0F,0X1F,0XFF,0XE0,
0X00,0XFF,0X8F,0XFF,0XFF,0XF8,0X1F,0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XF8,0X18,0X30,
0X00,0XFF,0XFF,0XFF,0XE0,0X03,0X1F,0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFF,0XFC,0X07,
0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XF8,0X08,0X38,0X00,0XFF,0XFF,0XFF,0XE3,0XC1,0X9F,
0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0X03,0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XF0,
0X08,0X30,0X01,0XFF,0XFF,0XFF,0XE7,0XF0,0X9F,0XFF,0XFF,0XE1,0XFF,0X8F,0XFF,0XFF,
0XFF,0X81,0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XE0,0X08,0X00,0X01,0XFF,0XFF,0XFF,0XE7,
0XF8,0X1F,0XFF,0XFF,0XF9,0XFF,0X8F,0XFF,0XFF,0XFF,0XE0,0X0F,0XFF,0XFF,0XF1,0XFF,
0XFF,0XE0,0XE4,0X00,0X01,0XFF,0XFF,0XFF,0XE7,0XFC,0X1F,0XFF,0XFF,0XF8,0XFF,0X8F,
0XFF,0XFF,0XFF,0XF0,0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XE0,0XE2,0X00,0X03,0XFF,0XFF,
0XFF,0XE3,0XFE,0X1F,0XFF,0XFF,0XF8,0XFF,0X8F,0XFF,0XFF,0XFF,0XF8,0X0F,0XFF,0XFF,
0XF1,0XFF,0XFF,0XC0,0XE2,0X00,0X07,0XFF,0XFF,0XFF,0XF1,0XFF,0X1F,0XFF,0XE0,0X00,
0XFF,0X8F,0XFF,0XFF,0XFF,0XFC,0X0F,0XFF,0XFF,0XF1,0XFF,0XFF,0XC0,0X41,0X00,0X0F,
0XFF,0XFF,0XFF,0XF3,0XFF,0X9F,0XFF,0XE0,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFE,0X0F,
0XFF,0XFF,0XF1,0XFF,0XFF,0XC0,0X00,0XC0,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XE0,0X01,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XC0,0X00,
0X30,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE0,0X03,0XFF,0X8F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XC0,0X00,0X07,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,
0XC0,0X00,0X00,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,
0XFF,0XFF,0XFF,0X07,0XFF,0XFF,0XF1,0XFF,0XFF,0XC0,0X00,0X00,0X7F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XE0,0XFC,0X01,0XFF,0XFF,0XF1,
0XFF,0XFF,0XC0,0X40,0X00,0X7F,0XFF,0XFF,0XFF,0XFF,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X8F,0XFF,0XFF,0XC0,0X78,0X00,0XFF,0XFF,0XF1,0XFF,0XFF,0XC0,0XE0,0X00,0X7F,0XFF,
0XFF,0XFF,0XFF,0X3F,0XFF,0XFF,0XFF,0X87,0XFF,0X8F,0XFF,0XFF,0X80,0X38,0X00,0XFF,
0XFF,0XF1,0XFF,0XFF,0XE0,0XE0,0X00,0X7F,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,0XFF,0X1E,
0X03,0XFF,0X8F,0XFF,0XFF,0X00,0X10,0X00,0X7F,0XFF,0XF1,0XFF,0XFF,0XE0,0XE0,0X00,
0X7F,0XFF,0XFF,0XFE,0XFF,0XBF,0XCF,0XFE,0X3E,0X01,0XFF,0X8F,0XFF,0XFF,0X00,0X10,
0X00,0X7F,0XFF,0XF1,0XFF,0XFF,0XE0,0X00,0X00,0X3F,0XFF,0XFF,0XFE,0X3F,0XFF,0X8F,
0XFE,0X3C,0X01,0XFF,0X8F,0XFF,0XFE,0X00,0X00,0XF0,0X7F,0XFF,0XF1,0XFF,0XFF,0XF0,
0X00,0X00,0X3F,0XFF,0XFF,0XFF,0X3C,0X0F,0X9F,0XFE,0X3C,0X30,0XFF,0X8F,0XFF,0XFE,
0X0F,0X01,0XF8,0X3F,0XFF,0XF1,0XFF,0XFF,0XF8,0X00,0X01,0X1F,0XFF,0XFF,0XFF,0XF0,
0X03,0XFF,0XFE,0X3C,0X78,0XFF,0X8F,0XFF,0XFE,0X0F,0X81,0XFC,0X3F,0XFF,0XF1,0XFF,
0XFF,0XF8,0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XE0,0X00,0XFF,0XFE,0X38,0X78,0XFF,0X8F,
0XFF,0XFE,0X1F,0XC3,0XFC,0X3F,0XFF,0XF1,0XFF,0XFF,0XFC,0X00,0X07,0XFF,0XFF,0XFF,
0XFF,0XC0,0X00,0XFF,0XFE,0X18,0X78,0XFF,0X8F,0XFF,0XFE,0X1F,0XC3,0XFC,0X3F,0XFF,
0XF1,0XFF,0XFF,0XFF,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,0XC0,0X00,0X7F,0XFE,0X00,0XF8,
0XFF,0X8F,0XFF,0XFE,0X1F,0XC3,0XFC,0X3F,0XFF,0XF1,0XFF,0XFF,0XFF,0X80,0X3F,0XFF,
0XFF,0XFF,0XFF,0X80,0X00,0X3F,0XFF,0X00,0XF8,0XFF,0X8F,0XFF,0XFE,0X1F,0XC3,0XFC,
0X3F,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X3F,0XFF,
0X01,0XF0,0XFF,0X8F,0XFF,0XFE,0X1F,0XC3,0XFC,0X3F,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X3F,0XFF,0XC3,0XF1,0XFF,0X8F,0XFF,0XFF,0X0F,
0XC3,0XFC,0X3F,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0X00,0X00,
0X31,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0X0F,0XFF,0XF8,0X3F,0XFF,0XF1,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0X00,0X00,0X31,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,
0XFF,0X07,0XFF,0XF8,0X7F,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X80,0X00,0X3F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XF0,0X7F,0XFF,0XF1,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X3F,0XFF,0XFF,0XFF,0XFF,
0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0X80,0X00,0X7F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0X00,0X7F,0XFF,0XFF,
0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XE0,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X01,0XFF,
0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X78,0X03,0XDF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X3F,
0X1F,0X8F,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XCF,0XFF,0XFF,0XFF,0XFF,0X8F,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,0XFF,0XFF,0XFF,
0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0X3F,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0X00,0X00,
0X00,0X01,0X7F,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFD,0X80,0X00,0X00,0X03,0X1F,0XFF,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0X0F,0XF8,0X7F,0XFF,0XF1,0XFF,0XF8,
0X80,0X00,0X00,0X02,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,
0XFF,0XFE,0X07,0XF8,0X3F,0XFF,0XF1,0XFF,0XF0,0X40,0X00,0X00,0X04,0X0F,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFE,0X07,0XF0,0X3F,0XFF,0XF1,
0XFF,0XF0,0X60,0X00,0X00,0X0C,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X8F,0XFF,0XFF,0XFE,0X07,0XF0,0X3F,0XFF,0XF1,0XFF,0XF0,0X20,0X00,0X00,0X08,0X07,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFE,0X07,0XF0,0X3F,
0XFF,0XF1,0XFF,0XF0,0X30,0X00,0X00,0X10,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0X07,0XF8,0X7F,0XFF,0XF1,0XFF,0XF0,0X10,0X00,0X00,
0X30,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0X0F,
0XF8,0X7F,0XFF,0XF1,0XFF,0XF0,0X18,0X00,0X00,0X20,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X08,
0X00,0X00,0X60,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X04,0X00,0X00,0X40,0X07,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,
0XF0,0X06,0X00,0X00,0X80,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X02,0X00,0X01,0X80,0X07,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF1,0XFF,0XF0,0X03,0X00,0X01,0X00,0X07,0XFF,0XFF,0XFF,0XFF,0XE0,0X00,0X00,0X00,
0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X01,0X00,0X02,0X00,
0X07,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF1,0XFF,0XF0,0X01,0X80,0X06,0X00,0X07,0XFF,0XFF,0XFF,0XFC,0X00,0X00,
0X00,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X80,
0X04,0X00,0X07,0XFF,0XFF,0XFF,0XF0,0X00,0X38,0X00,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X40,0X08,0X00,0X07,0XFF,0XFF,0XFF,0XE0,
0X00,0XFE,0X00,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,
0X00,0X40,0X08,0X00,0X07,0XFF,0XFF,0XFF,0XC0,0X03,0XFF,0X80,0X00,0XFF,0X8F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X20,0X10,0X00,0X07,0XFF,0XFF,
0XFF,0X80,0X0F,0XFF,0XF8,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,
0XFF,0XF0,0X00,0X30,0X30,0X00,0X07,0XFF,0XFF,0XFF,0X00,0X3F,0XFF,0XFC,0X00,0XFF,
0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X10,0X20,0X00,0X07,
0XFF,0XFF,0XFE,0X00,0XFF,0XE7,0XFE,0X00,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF1,0XFF,0XF0,0X00,0X18,0X40,0X00,0X07,0XFF,0XFF,0XFC,0X01,0XFF,0X81,0XFF,
0X00,0XFF,0X8F,0XFF,0XFF,0XF0,0X3F,0XFC,0X3F,0XFF,0XF1,0XFF,0XF0,0X00,0X08,0XC0,
0X00,0X07,0XFF,0XFF,0XFC,0X01,0XFF,0X00,0X7F,0X00,0XFF,0X8F,0XFF,0XFF,0XC0,0X0F,
0XFC,0X3F,0XFF,0XF1,0XFF,0XF0,0X00,0X04,0X80,0X00,0X07,0XFF,0XFF,0XF8,0X01,0XFF,
0XC0,0X7F,0X80,0XFF,0X8F,0XFF,0XFF,0X80,0X07,0XFC,0X3F,0XFF,0XF1,0XFF,0XF0,0X00,
0X05,0X00,0X00,0X07,0XFF,0XFF,0XF8,0X00,0XFF,0XF8,0X00,0X00,0XFF,0X8F,0XFF,0XFF,
0X00,0X03,0XFC,0X3F,0XFF,0XF1,0XFF,0XF0,0X00,0X03,0X00,0X00,0X07,0XFF,0XFF,0XF0,
0X00,0X1F,0XFE,0X00,0X00,0XFF,0X8F,0XFF,0XFF,0X00,0X01,0XFC,0X3F,0XFF,0XF1,0XFF,
0XF0,0X00,0X03,0X00,0X00,0X07,0XFF,0XFF,0XF0,0X00,0X07,0XFF,0X80,0X00,0XFF,0X8F,
0XFF,0XFE,0X00,0X00,0XFC,0X3F,0XFF,0XF1,0XFF,0XF0,0X00,0X01,0X00,0X00,0X07,0XFF,
0XFF,0XF1,0XFF,0X81,0XFF,0X80,0X00,0XFF,0X8F,0XFF,0XFE,0X07,0X80,0XFC,0X3F,0XFF,
0XF1,0XFF,0XF0,0X00,0X01,0X80,0X00,0X07,0XFF,0XFF,0XE0,0XFF,0X00,0X7F,0X80,0X00,
0XFF,0X8F,0XFF,0XFE,0X0F,0XE0,0X7C,0X3F,0XFF,0XF1,0XFF,0XF0,0X47,0X80,0X80,0X00,
0X07,0XFF,0XFF,0XE0,0X7F,0XC1,0X3F,0X80,0X00,0XFF,0X8F,0XFF,0XFE,0X1F,0XF0,0X3C,
0X3F,0XFF,0XF1,0XFF,0XF0,0X4F,0XC0,0X40,0X00,0X07,0XFF,0XFF,0XE0,0X3F,0XF9,0X4F,
0X00,0X00,0XFF,0X8F,0XFF,0XFE,0X1F,0XF8,0X1C,0X3F,0XFF,0XF1,0XFF,0XF0,0X4C,0X40,
0X40,0X00,0X07,0XFF,0XFF,0XE0,0X1F,0XFE,0X78,0X00,0X00,0XFF,0X8F,0XFF,0XFE,0X1F,
0XF8,0X0C,0X3F,0XFF,0XF1,0XFF,0XF0,0X64,0XC0,0X40,0X00,0X07,0XFF,0XFF,0XE0,0X0F,
0XFF,0X80,0X00,0X00,0XFF,0X8F,0XFF,0XFE,0X1F,0XFC,0X04,0X3F,0XFF,0XF1,0XFF,0XF0,
0X7F,0XC0,0X40,0X00,0X07,0XFF,0XFF,0XE0,0X05,0XFF,0XF0,0X00,0X00,0XFF,0X8F,0XFF,
0XFE,0X1F,0XFE,0X00,0X3F,0XFF,0XF1,0XFF,0XF0,0X1F,0X00,0XC0,0X00,0X07,0XFF,0XFF,
0XE0,0X00,0X3F,0XFC,0X00,0X00,0XFF,0X8F,0XFF,0XFF,0X0F,0XFF,0X00,0X3F,0XFF,0XF1,
0XFF,0XF0,0X00,0X00,0X80,0X00,0X07,0XFF,0XFF,0XE0,0X00,0X0F,0XFF,0X00,0X00,0XFF,
0X8F,0XFF,0XFF,0X0F,0XFF,0X00,0X3F,0XFF,0XF1,0XFF,0XF0,0X00,0X01,0X00,0X00,0X07,
0XFF,0XFF,0XE0,0X00,0X03,0XFF,0X80,0X00,0XFF,0X8F,0XFF,0XFF,0X07,0XFF,0XC0,0X3F,
0XFF,0XF1,0XFF,0XF0,0X00,0X01,0X00,0X00,0X07,0XFF,0XFF,0XE0,0X00,0X07,0XFF,0X80,
0X01,0XFF,0X8F,0XFF,0XFF,0X87,0XFF,0XE0,0X3F,0XFF,0XF1,0XFF,0XF0,0X00,0X02,0X00,
0X00,0X07,0XFF,0XFF,0XE0,0X00,0X3F,0XFF,0X00,0X01,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,
0XF8,0X3F,0XFF,0XF1,0XFF,0XF0,0X00,0X06,0X00,0X00,0X07,0XFF,0XFF,0XE0,0X00,0XFF,
0XFC,0X00,0X01,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,
0X05,0X00,0X00,0X07,0XFF,0XFF,0XE0,0X01,0XFF,0XE0,0X00,0X01,0XFF,0X8F,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X0D,0X80,0X00,0X07,0XFF,0XFF,0XE0,
0X01,0XFF,0X00,0X00,0X03,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,
0XF0,0X00,0X08,0X80,0X00,0X07,0XFF,0XFF,0XE0,0X01,0XFF,0X80,0X00,0X03,0XFF,0X8F,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X10,0X40,0X00,0X07,0XFF,
0XFF,0XE0,0X00,0XFF,0XF0,0X00,0X03,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF1,0XFF,0XF0,0X00,0X10,0X40,0X00,0X07,0XFF,0XFF,0XE0,0X00,0X1F,0XFE,0X00,0X07,
0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X20,0X20,0X00,
0X07,0XFF,0XFF,0XE0,0X00,0X03,0XFF,0X80,0X07,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X60,0X30,0X00,0X07,0XFF,0XFF,0XE0,0X00,0X07,0XFF,
0X80,0X0F,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0X40,
0X10,0X00,0X07,0XFF,0XFF,0XE0,0X00,0X3F,0XFF,0X80,0X1F,0XFF,0X8F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X00,0XC0,0X08,0X00,0X07,0XFF,0XFF,0XE0,0X01,
0XFF,0XFF,0X00,0X1F,0XFF,0X8F,0XFF,0XFE,0X00,0X00,0X00,0X3F,0XFF,0XF1,0XFF,0XF0,
0X00,0X80,0X0C,0X00,0X07,0XFF,0XFF,0XE0,0X01,0XFF,0XF8,0X00,0X3F,0XFF,0X8F,0XFF,
0XFE,0X00,0X00,0X00,0X3F,0XFF,0XF1,0XFF,0XF0,0X01,0X80,0X04,0X00,0X07,0XFF,0XFF,
0XE0,0X01,0XFF,0XE0,0X00,0X7F,0XFF,0X8F,0XFF,0XFE,0X00,0X00,0X00,0X3F,0XFF,0XF1,
0XFF,0XF0,0X01,0X00,0X02,0X00,0X07,0XFF,0XFF,0XE0,0X01,0XFE,0X00,0X00,0XFF,0XFF,
0X8F,0XFF,0XFE,0X00,0X00,0X00,0X3F,0XFF,0XF1,0XFF,0XF0,0X02,0X00,0X03,0X00,0X07,
0XFF,0XFF,0XE0,0X01,0XF0,0X00,0X03,0XFF,0XFF,0X8F,0XFF,0XFE,0X00,0X00,0X00,0X3F,
0XFF,0XF1,0XFF,0XF0,0X06,0X00,0X01,0X00,0X07,0XFF,0XFF,0XE0,0X01,0X80,0X00,0X07,
0XFF,0XFF,0X8F,0XFF,0XFF,0X00,0X00,0X00,0X3F,0XFF,0XF1,0XFF,0XF0,0X04,0X00,0X01,
0X80,0X07,0XFF,0XFF,0XE0,0X00,0X00,0X00,0X1F,0XFF,0XFF,0X8F,0XFF,0XFF,0X83,0XFF,
0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X0C,0X00,0X00,0X80,0X07,0XFF,0XFF,0XE0,0X00,0X00,
0X00,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0X81,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X08,
0X00,0X00,0X40,0X07,0XFF,0XFF,0XE0,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,
0XC1,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X18,0X00,0X00,0X60,0X07,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XC1,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,
0XF0,0X10,0X00,0X00,0X20,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,
0XFF,0XFF,0XE0,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X20,0X00,0X00,0X10,0X07,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XE0,0XFF,0XFF,0XFF,0XFF,
0XF1,0XFF,0XF0,0X60,0X00,0X00,0X18,0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF0,0X40,0X00,0X00,0X08,
0X07,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF1,0XFF,0XF8,0XC0,0X00,0X00,0X04,0X0F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XF8,0X80,0X00,
0X00,0X04,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0X00,0X00,0X00,0X02,0X3F,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X8F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,};


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

void gpio_init(void){
		nrf_gpio_cfg_output(EPD_RST_PIN);
	  nrf_gpio_cfg_output(EPD_DC_PIN);
	  nrf_gpio_cfg_output(EPD_CS_PIN);
		nrf_gpio_cfg_input(EPD_BUSY_PIN,NRF_GPIO_PIN_NOPULL);
}

int DEV_Module_Init(void)
{
    nrf_gpio_pin_write(EPD_DC_PIN, 0);
    nrf_gpio_pin_write(EPD_CS_PIN, 0);
    nrf_gpio_pin_write(EPD_RST_PIN, 1);
		return 0;
}

/******************************************************************************
function :	Software reset
parameter:
******************************************************************************/
static void EPD_1IN54_V2_Reset(void)
{
    nrf_gpio_pin_write(EPD_RST_PIN, 1);
    nrf_delay_ms(200);
    nrf_gpio_pin_write(EPD_RST_PIN, 0);
    nrf_delay_ms(10);
    nrf_gpio_pin_write(EPD_RST_PIN, 1);
    nrf_delay_ms(200);
}

/******************************************************************************
function :	Wait until the busy_pin goes LOW
parameter:
******************************************************************************/
static void EPD_1IN54_V2_ReadBusy(void)
{
    while(nrf_gpio_pin_read(EPD_BUSY_PIN) == 1) {      //LOW: idle, HIGH: busy
        nrf_delay_ms(100);
    }
}

/******************************************************************************
function :	send command
parameter:
     Reg : Command register
******************************************************************************/
static void EPD_1IN54_V2_SendCommand(uint8_t Reg)
{
    nrf_gpio_pin_write(EPD_DC_PIN, 0);
//    nrf_gpio_pin_write(EPD_CS_PIN, 0);
		SPI_ReadWriteData(&Reg, m_rx_buf, sizeof(uint8_t));
//    nrf_drv_spi_transfer(&spi, &Reg, 1, m_rx_buf, m_length);
//    nrf_gpio_pin_write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void EPD_1IN54_V2_SendData(uint8_t Data)
{
    nrf_gpio_pin_write(EPD_DC_PIN, 1);
//    nrf_gpio_pin_write(EPD_CS_PIN, 0);
		SPI_ReadWriteData(&Data, m_rx_buf, sizeof(uint8_t));
//    nrf_drv_spi_transfer(&spi, &Data, 1, m_rx_buf, m_length);
//    nrf_gpio_pin_write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	Turn On Display full
parameter:
******************************************************************************/
static void EPD_1IN54_V2_TurnOnDisplay(void)
{
    EPD_1IN54_V2_SendCommand(0x22);
    EPD_1IN54_V2_SendData(0xF7);
    EPD_1IN54_V2_SendCommand(0x20);
    EPD_1IN54_V2_ReadBusy();
}

/******************************************************************************
function :	Initialize the e-Paper register
parameter:
******************************************************************************/
void EPD_1IN54_V2_Init(void)
{
    EPD_1IN54_V2_Reset();
    EPD_1IN54_V2_ReadBusy();
    EPD_1IN54_V2_SendCommand(0x12);  //SWRESET
    EPD_1IN54_V2_ReadBusy();

    EPD_1IN54_V2_SendCommand(0x01); //Driver output control
    EPD_1IN54_V2_SendData(0xC7);
    EPD_1IN54_V2_SendData(0x00);
    EPD_1IN54_V2_SendData(0x01);

    EPD_1IN54_V2_SendCommand(0x11); //data entry mode
    EPD_1IN54_V2_SendData(0x01);

    EPD_1IN54_V2_SendCommand(0x44); //set Ram-X address start/end position
    EPD_1IN54_V2_SendData(0x00);
    EPD_1IN54_V2_SendData(0x18);    //0x0C-->(18+1)*8=200

    EPD_1IN54_V2_SendCommand(0x45); //set Ram-Y address start/end position
    EPD_1IN54_V2_SendData(0xC7);   //0xC7-->(199+1)=200
    EPD_1IN54_V2_SendData(0x00);
    EPD_1IN54_V2_SendData(0x00);
    EPD_1IN54_V2_SendData(0x00);

    EPD_1IN54_V2_SendCommand(0x3C); //BorderWavefrom
    EPD_1IN54_V2_SendData(0x01);

    EPD_1IN54_V2_SendCommand(0x18);
    EPD_1IN54_V2_SendData(0x80);

    EPD_1IN54_V2_SendCommand(0x22); // //Load Temperature and waveform setting.
    EPD_1IN54_V2_SendData(0XB1);
    EPD_1IN54_V2_SendCommand(0x20);

    EPD_1IN54_V2_SendCommand(0x4E);   // set RAM x address count to 0;
    EPD_1IN54_V2_SendData(0x00);
    EPD_1IN54_V2_SendCommand(0x4F);   // set RAM y address count to 0X199;
    EPD_1IN54_V2_SendData(0xC7);
    EPD_1IN54_V2_SendData(0x00);
    EPD_1IN54_V2_ReadBusy();
}

void EPD_1IN54_V2_Display(UBYTE *Image)
{
    UWORD Width, Height;
    Width = (EPD_1IN54_V2_WIDTH % 8 == 0)? (EPD_1IN54_V2_WIDTH / 8 ): (EPD_1IN54_V2_WIDTH / 8 + 1);
    Height = EPD_1IN54_V2_HEIGHT;

    int Addr = 0;
    EPD_1IN54_V2_SendCommand(0x24);
    for (UWORD j = 0; j < Height; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Addr = i + j * Width;
            EPD_1IN54_V2_SendData(Image[Addr]);
        }
    }
    EPD_1IN54_V2_TurnOnDisplay();
}

/******************************************************************************
function :	Clear screen
parameter:
******************************************************************************/
void EPD_1IN54_V2_Clear(void)
{
    uint16_t Width, Height;
    Width = (EPD_1IN54_V2_WIDTH % 8 == 0)? (EPD_1IN54_V2_WIDTH / 8 ): (EPD_1IN54_V2_WIDTH / 8 + 1);
    Height = EPD_1IN54_V2_HEIGHT;

    EPD_1IN54_V2_SendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_1IN54_V2_SendData(0X00);
        }
    }
    EPD_1IN54_V2_TurnOnDisplay();
}

uint8_t read_button1_GPIO(void)
{
    nrf_gpio_cfg_input(3,NRF_GPIO_PIN_PULLUP);
    return nrf_gpio_pin_read(3);
}

uint8_t read_button2_GPIO(void)
{
    nrf_gpio_cfg_input(4,NRF_GPIO_PIN_PULLUP);
    return nrf_gpio_pin_read(4);
}

uint8_t read_button3_GPIO(void)
{
    nrf_gpio_cfg_input(35,NRF_GPIO_PIN_PULLUP);
    return nrf_gpio_pin_read(35);
}

void BTN1_PRESS_DOWN_Handler(void *btn)
{
    bsp_board_led_on(0);
}


void BTN1_PRESS_UP_Handler(void *btn)
{
    bsp_board_led_off(0);
}

void BTN2_PRESS_DOWN_Handler(void *btn)
{
    bsp_board_led_on(1);
}


void BTN2_PRESS_UP_Handler(void *btn)
{
    bsp_board_led_off(1);
}

void BTN3_PRESS_DOWN_Handler(void *btn)
{
    bsp_board_led_on(2);
}


void BTN3_PRESS_UP_Handler(void *btn)
{
    bsp_board_led_off(2);
}

int main(void)
{	
    bsp_board_init(BSP_INIT_LEDS);

    gpio_init();
    bsp_board_leds_on();

    SPI_Init();
    SPI_Disable();
    SPI_Enable();
    DEV_Module_Init(); //Init Dev module
    EPD_1IN54_V2_Init();
    EPD_1IN54_V2_Clear();
    nrf_delay_ms(500);
		
	unsigned char *BlackImage;
    /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
    unsigned short Imagesize = ((EPD_1IN54_V2_WIDTH % 8 == 0)? (EPD_1IN54_V2_WIDTH / 8 ): (EPD_1IN54_V2_WIDTH / 8 + 1)) * EPD_1IN54_V2_HEIGHT;
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        return -1;
    }
    Paint_NewImage(BlackImage, EPD_1IN54_V2_WIDTH, EPD_1IN54_V2_HEIGHT, 270, WHITE);
 
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);
    Paint_DrawBitMap(gImage_1in54);

    EPD_1IN54_V2_Display(BlackImage);
    nrf_delay_ms(2000);
		
	struct button btn1,btn2,btn3;
		
    button_init(&btn1, read_button1_GPIO, 0);
    button_init(&btn2, read_button2_GPIO, 0);
    button_init(&btn3, read_button3_GPIO, 0);
		
    button_attach(&btn1, PRESS_DOWN, BTN1_PRESS_DOWN_Handler);
    button_attach(&btn1, PRESS_UP,   BTN1_PRESS_UP_Handler);
    button_attach(&btn2, PRESS_DOWN, BTN2_PRESS_DOWN_Handler);
    button_attach(&btn2, PRESS_UP,   BTN2_PRESS_UP_Handler);
    button_attach(&btn3, PRESS_DOWN, BTN3_PRESS_DOWN_Handler);
    button_attach(&btn3, PRESS_UP,   BTN3_PRESS_UP_Handler);
    button_start(&btn1);
    button_start(&btn2);
    button_start(&btn3);
		
    // bsp_board_leds_off();
    while (1)
    {
		button_ticks();
        bsp_board_led_invert(0);
        nrf_delay_ms(3000);
    }
}
