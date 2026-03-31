#include "input.h"
#include "config.h"
#include "settings.h"

#include <math.h>

int input_apply_stick(float raw)
{
    float dz = g_settings.deadzone;
    float expo = g_settings.expo;
    if (fabsf(raw) < dz)
        return 0;
    float sign = raw > 0 ? 1.0f : -1.0f;
    float norm = (fabsf(raw) - dz) / (1.0f - dz);
    return (int)(sign * powf(norm, expo) * 100.0f);
}

void input_read_gamepad(SDL_GameController *gc,
                        int *roll, int *pitch,
                        int *throttle, int *yaw)
{
    *roll = input_apply_stick(
        SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTX) / 32767.0f);
    *pitch = -input_apply_stick(
        SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTY) / 32767.0f);
    *yaw = input_apply_stick(
        SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_RIGHTX) / 32767.0f);
    *throttle = -input_apply_stick(
        SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_RIGHTY) / 32767.0f);
}

void input_read_keyboard(int *roll, int *pitch,
                         int *throttle, int *yaw)
{
    const Uint8 *k = SDL_GetKeyboardState(NULL);
    int spd = 60;
    *roll = k[SDL_SCANCODE_D] ? spd : (k[SDL_SCANCODE_A] ? -spd : 0);
    *pitch = k[SDL_SCANCODE_W] ? spd : (k[SDL_SCANCODE_S] ? -spd : 0);
    *throttle = k[SDL_SCANCODE_UP] ? spd : (k[SDL_SCANCODE_DOWN] ? -spd : 0);
    *yaw = k[SDL_SCANCODE_RIGHT] ? spd : (k[SDL_SCANCODE_LEFT] ? -spd : 0);
}
