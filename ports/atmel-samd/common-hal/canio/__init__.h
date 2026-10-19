// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2020 Jeff Epler for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "hal/utils/include/utils.h"
#include "component/can.h"

#define COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH (64)
#define COMMON_HAL_CANIO_RX_FIFO_SIZE (3)
#define COMMON_HAL_CANIO_RX_FILTER_SIZE (4)
#define COMMON_HAL_CANIO_TX_FIFO_SIZE (1)
#define COMMON_HAL_CANIO_FD_SUPPORTED (1)

// This appears to be a typo (transposition error) in the ASF4 headers
// It's called the "Extended ID Filter Entry"
typedef CanMramXifde CanMramXidfe;

typedef struct canio_listener canio_listener_t;
typedef struct canio_can canio_can_t;

typedef struct {
    CAN_TXBE_0_Type txb0;
    CAN_TXBE_1_Type txb1;
    COMPILER_ALIGNED(4)
    uint8_t data[COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH];
} canio_can_tx_buffer_t;

typedef struct {
    CAN_RXF0E_0_Type rxf0;
    CAN_RXF0E_1_Type rxf1;
    COMPILER_ALIGNED(4)
    uint8_t data[COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH];
} canio_can_rx_fifo_t;

typedef uint32_t canio_can_filter_t;

typedef struct {
    canio_can_tx_buffer_t tx_buffer[COMMON_HAL_CANIO_TX_FIFO_SIZE];
    canio_can_rx_fifo_t rx0_fifo[COMMON_HAL_CANIO_RX_FIFO_SIZE];
    canio_can_rx_fifo_t rx1_fifo[COMMON_HAL_CANIO_RX_FIFO_SIZE];
    CanMramSidfe standard_rx_filter[COMMON_HAL_CANIO_RX_FILTER_SIZE];
    CanMramXifde extended_rx_filter[COMMON_HAL_CANIO_RX_FILTER_SIZE];
} canio_can_state_t;

// CAN FD DLC <-> byte length
static inline uint8_t canio_dlc_to_len(uint8_t dlc) {
    static const uint8_t lens[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return lens[dlc & 0xf];
}

// Smallest DLC whose length is >= len
static inline uint8_t canio_len_to_dlc(size_t len) {
    if (len <= 8) {
        return (uint8_t)len;
    }
    if (len <= 12) {
        return 9;
    }
    if (len <= 16) {
        return 10;
    }
    if (len <= 20) {
        return 11;
    }
    if (len <= 24) {
        return 12;
    }
    if (len <= 32) {
        return 13;
    }
    if (len <= 48) {
        return 14;
    }
    return 15;
}