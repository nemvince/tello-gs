#pragma once

#include <stdbool.h>

/*
 * Global running flag.
 * Set to false to signal all threads to exit gracefully.
 * Defined in main.c.
 */
extern volatile bool g_running;
