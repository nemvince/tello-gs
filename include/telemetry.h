#pragma once

#include <stdbool.h>

typedef struct {
    int bat, height, flight_time;
    int pitch, roll, yaw;
    int vgx, vgy, vgz;
    int templ, temph, tof;
    bool valid;
} Telemetry;

/* Launch the background state-receiver thread. */
void telemetry_start(void);

/* Signal the thread to stop and join it. */
void telemetry_stop(void);

/* Thread-safe copy of the latest telemetry snapshot. */
void telemetry_get(Telemetry *out);
