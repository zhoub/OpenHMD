/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* Internal Interface for Platform Specific Functions */

#ifndef PLATFORM_H
#define PLATFORM_H

#include "openhmd.h"

OHMD_APIENTRYDLL double OHMD_APIENTRY ohmd_get_tick();
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_sleep(double seconds);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_toggle_ovr_service(int state);

typedef struct ohmd_thread ohmd_thread;
typedef struct ohmd_mutex ohmd_mutex;

OHMD_APIENTRYDLL ohmd_mutex* OHMD_APIENTRY ohmd_create_mutex(ohmd_context* ctx);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_destroy_mutex(ohmd_mutex* mutex);

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_lock_mutex(ohmd_mutex* mutex);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_unlock_mutex(ohmd_mutex* mutex);

OHMD_APIENTRYDLL ohmd_thread* OHMD_APIENTRY ohmd_create_thread(ohmd_context* ctx, unsigned int (*routine)(void* arg), void* arg);
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_destroy_thread(ohmd_thread* thread);

/* String functions */

int findEndPoint(char* path, int endpoint);

#endif
