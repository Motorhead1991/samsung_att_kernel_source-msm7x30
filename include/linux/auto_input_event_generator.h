/*
 * include/linux/auto_input_event_generator.h - auto event generator
 *
 * Copyright (C) 2011 Samsung electronices
 * Jaecheol Kim <jc22.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __AUTO_INPUT_EVENT_GENERATOR__
#define __AUTO_INPUT_EVENT_GENERATOR__

#include <linux/input.h>

/* 
 * Auto event generator type
 */
enum auto_genevent_type {
    NO_AUTO_EVENT           = 0,
    AUTO_EVENT_KEY          = 0x1,        // Key
    AUTO_EVENT_TOUCHKEY     = 0x2,        // Touchkey
    AUTO_EVENT_TOUCH        = 0x4,        // Touch
    AUTO_EVENT_ALL          = AUTO_EVENT_KEY | AUTO_EVENT_TOUCH | AUTO_EVENT_TOUCHKEY,        // all event
};

/*
 * default event value
 */
enum auto_genevent_default_value {

    /* touch */
    DEFAULT_TOUCH_X = 480,
    DEFAULT_TOUCH_Y = 800,
    DEFAULT_TOUCH_STR = 110,

    /* touch key */
    DEFAULT_TOUCHKEY_EVENT = KEY_BACK,

    /* key */
    DEFAULT_KEY_EVENT = KEY_HOME,
};

enum auto_genevent_max_loop_value {
    /* TOUCHKEY event loop count */
    DEFAULT_EVENT_COUNT_KEY = 3,

    /* KEY event loop count */
    DEFAULT_EVENT_COUNT_TOUCHKEY = 3,
};

/* 
 * samsung debug monitor platform data
 */
struct auto_input_event_generator_platform_data {

    /* monitoring event type */
    unsigned int event;

    /* event generation time : micro seconds */
    unsigned int repeat_time_key;
    unsigned int repeat_time_touchkey;
    unsigned int repeat_time_touch;

    /* start event after boot : micro seconds */
    unsigned int auto_start;
    unsigned int auto_start_time;

    /* key info */
    unsigned int key_code[DEFAULT_EVENT_COUNT_KEY];

    /* touch info */
    unsigned int touch_matrix_x;
    unsigned int touch_matrix_y;
    unsigned int touch_matrix_str;

    /* touchkey info */
    unsigned int touchkey_code[DEFAULT_EVENT_COUNT_TOUCHKEY];

    /* event handler */
    void (*key_event_handler)(unsigned int keyevt);
    void (*touchkey_event_handler)(unsigned int keyevt);
    void (*touch_event_handler)(int pos_x, int pos_y, int pos_str);

};
#endif  //__AUTO_INPUT_EVENT_GENERATOR__
