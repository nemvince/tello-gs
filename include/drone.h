#pragma once

#include <stdbool.h>

/*
 * drone_connected is set true as soon as the first telemetry packet
 * arrives, and is read by the HUD to show connection status.
 */
extern volatile bool drone_connected;

/* Set up the UDP command socket. Returns 0 on success, -1 on error. */
int  drone_init(void);

/* Send a raw SDK command string (fire-and-forget). */
void drone_send(const char *cmd);

/* Send an RC command; values are clamped to [-100, 100]. */
void drone_rc(int roll, int pitch, int throttle, int yaw);

/* Close the command socket. */
void drone_cleanup(void);
