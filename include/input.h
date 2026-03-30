#pragma once

#include <SDL2/SDL.h>

/* Apply deadzone and expo curve to a raw axis value in [-1, 1].
 * Returns a drone RC value in [-100, 100]. */
int input_apply_stick(float raw);

/* Read all four axes from a gamepad. */
void input_read_gamepad(SDL_GameController *gc,
                        int *roll, int *pitch,
                        int *throttle, int *yaw);

/* Read WASD + arrow keys as discrete RC values. */
void input_read_keyboard(int *roll, int *pitch,
                         int *throttle, int *yaw);
