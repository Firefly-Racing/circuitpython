// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2020 Jeff Epler for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/canio/Message.h"

#include "py/obj.h"
#include "py/objproperty.h"
#include "py/runtime.h"

#ifndef COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH
#define COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH (8)
#endif

//| class Message:
//|     def __init__(self, id: int, data: bytes, *, extended: bool = False, fd: bool = False, brs: bool = False) -> None:
//|         """Construct a Message to send on a CAN bus.
//|
//|         :param int id: The numeric ID of the message
//|         :param bytes data: The content of the message
//|         :param bool extended: True if the message has an extended identifier, False if it has a standard identifier
//|         :param bool fd: True for a CAN FD frame. Implied if ``data`` is longer than 8 bytes.
//|         :param bool brs: True to send the data phase at the faster data baudrate (FD only).
//|
//|         Classic CAN messages have 0 to 8 bytes. CAN FD messages have 0 to 8, 12, 16, 20, 24, 32, 48 or 64 bytes; other lengths are zero-padded up to the next valid size on transmit.
//|
//|         In CAN, messages can have a length from 0 to 8 bytes.
//|         """
//|         ...
//|
static mp_obj_t canio_message_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_id, ARG_data, ARG_extended, ARG_fd, ARG_brs, NUM_ARGS };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id, MP_ARG_INT | MP_ARG_REQUIRED },
        { MP_QSTR_data, MP_ARG_OBJ | MP_ARG_REQUIRED },
        { MP_QSTR_extended, MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_fd, MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_brs, MP_ARG_BOOL, {.u_bool = false} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    MP_STATIC_ASSERT(MP_ARRAY_SIZE(allowed_args) == NUM_ARGS);

    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t data;
    mp_get_buffer_raise(args[ARG_data].u_obj, &data, MP_BUFFER_READ);

    mp_arg_validate_length_range(data.len, 0, COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH, MP_QSTR_data);

    canio_message_obj_t *self = mp_obj_malloc(canio_message_obj_t, &canio_message_type);
    common_hal_canio_message_construct(self, args[ARG_id].u_int, data.buf, data.len, args[ARG_extended].u_bool);
    bool fd = args[ARG_fd].u_bool || data.len > 8;
    common_hal_canio_message_set_fd(self, fd);
    common_hal_canio_message_set_brs(self, fd && args[ARG_brs].u_bool);

    return MP_OBJ_FROM_PTR(self);
}

//|     id: int
//|     """The numeric ID of the message"""
static mp_obj_t canio_message_id_get(const mp_obj_t self_in) {
    canio_message_obj_t *self = self_in;
    return MP_OBJ_NEW_SMALL_INT(common_hal_canio_message_get_id(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(canio_message_id_get_obj, canio_message_id_get);

static mp_obj_t canio_message_id_set(const mp_obj_t self_in, const mp_obj_t id) {
    canio_message_obj_t *self = self_in;
    common_hal_canio_message_set_id(self, mp_obj_get_int(id));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(canio_message_id_set_obj, canio_message_id_set);

MP_PROPERTY_GETSET(canio_message_id_obj,
    (mp_obj_t)&canio_message_id_get_obj,
    (mp_obj_t)&canio_message_id_set_obj);

//|     data: bytes
//|     """The content of the message"""
static mp_obj_t canio_message_data_get(const mp_obj_t self_in) {
    canio_message_obj_t *self = self_in;
    return mp_obj_new_bytes((const byte *)common_hal_canio_message_get_data(self), common_hal_canio_message_get_length(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(canio_message_data_get_obj, canio_message_data_get);

static mp_obj_t canio_message_data_set(const mp_obj_t self_in, const mp_obj_t data_in) {
    canio_message_obj_t *self = self_in;
    mp_buffer_info_t data;
    mp_get_buffer_raise(data_in, &data, MP_BUFFER_READ);

    mp_arg_validate_length_range(data.len, 0, COMMON_HAL_CANIO_MAX_MESSAGE_LENGTH, MP_QSTR_data);

    common_hal_canio_message_set_data(self, data.buf, data.len);
    if (data.len > 8) {
        common_hal_canio_message_set_fd(self, true);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(canio_message_data_set_obj, canio_message_data_set);


MP_PROPERTY_GETSET(canio_message_data_obj,
    (mp_obj_t)&canio_message_data_get_obj,
    (mp_obj_t)&canio_message_data_set_obj);


//|     extended: bool
//|     """True if the message's id is an extended id"""
//|
//|
static mp_obj_t canio_message_extended_get(const mp_obj_t self_in) {
    canio_message_obj_t *self = self_in;
    return mp_obj_new_bool(common_hal_canio_message_get_extended(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(canio_message_extended_get_obj, canio_message_extended_get);

static mp_obj_t canio_message_extended_set(const mp_obj_t self_in, const mp_obj_t extended) {
    canio_message_obj_t *self = self_in;
    common_hal_canio_message_set_extended(self, mp_obj_is_true(extended));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(canio_message_extended_set_obj, canio_message_extended_set);


MP_PROPERTY_GETSET(canio_message_extended_obj,
    (mp_obj_t)&canio_message_extended_get_obj,
    (mp_obj_t)&canio_message_extended_set_obj);

//|     fd: bool
//|     """True if the message is a CAN FD frame"""
static mp_obj_t canio_message_fd_get(const mp_obj_t self_in) {
    canio_message_obj_t *self = self_in;
    return mp_obj_new_bool(common_hal_canio_message_get_fd(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(canio_message_fd_get_obj, canio_message_fd_get);

static mp_obj_t canio_message_fd_set(const mp_obj_t self_in, const mp_obj_t fd) {
    canio_message_obj_t *self = self_in;
    common_hal_canio_message_set_fd(self, mp_obj_is_true(fd));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(canio_message_fd_set_obj, canio_message_fd_set);

MP_PROPERTY_GETSET(canio_message_fd_obj,
    (mp_obj_t)&canio_message_fd_get_obj,
    (mp_obj_t)&canio_message_fd_set_obj);

//|     brs: bool
//|     """True if the data phase uses (or used) the faster data bit rate (FD only)"""
static mp_obj_t canio_message_brs_get(const mp_obj_t self_in) {
    canio_message_obj_t *self = self_in;
    return mp_obj_new_bool(common_hal_canio_message_get_brs(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(canio_message_brs_get_obj, canio_message_brs_get);

static mp_obj_t canio_message_brs_set(const mp_obj_t self_in, const mp_obj_t brs) {
    canio_message_obj_t *self = self_in;
    common_hal_canio_message_set_brs(self, mp_obj_is_true(brs));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(canio_message_brs_set_obj, canio_message_brs_set);

MP_PROPERTY_GETSET(canio_message_brs_obj,
    (mp_obj_t)&canio_message_brs_get_obj,
    (mp_obj_t)&canio_message_brs_set_obj);

static const mp_rom_map_elem_t canio_message_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_id), MP_ROM_PTR(&canio_message_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_data), MP_ROM_PTR(&canio_message_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_extended), MP_ROM_PTR(&canio_message_extended_obj) },
    { MP_ROM_QSTR(MP_QSTR_fd), MP_ROM_PTR(&canio_message_fd_obj) },
    { MP_ROM_QSTR(MP_QSTR_brs), MP_ROM_PTR(&canio_message_brs_obj) },
};
static MP_DEFINE_CONST_DICT(canio_message_locals_dict, canio_message_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    canio_message_type,
    MP_QSTR_Message,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    make_new, canio_message_make_new,
    locals_dict, &canio_message_locals_dict
    );
