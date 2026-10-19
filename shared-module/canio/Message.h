// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2020 Jeff Epler for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

#include "py/obj.h"

#if defined(__has_include)
#if __has_include("common-hal/canio/__init__.h")
#include "common-hal/canio/__init__.h"
#endif
#endif

#ifndef COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH
#define COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH (8)
#endif

typedef struct {
    mp_obj_base_t base;
    int id;
    uint8_t data[COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH];
    size_t size;
    bool extended;
    bool fd;
    bool brs;
} canio_message_obj_t;