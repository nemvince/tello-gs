/*
 * Tello Ground Station - main entry point
 * Deps: SDL2, SDL2_ttf, FFmpeg (libavcodec, libavutil, libswscale)
 * Build: make
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "app.h"
#include "config.h"
#include "config_menu.h"
#include "drone.h"
#include "gesture.h"
#include "hud.h"
#include "input.h"
#include "settings.h"
#include "telemetry.h"
#include "tracker.h"
#include "video.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <stdbool.h>
#include <stdio.h>

volatile bool g_running = true;

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  /* ----- Load persistent settings ----- */
  settings_load();

  /* ----- SDL init ----- */
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0)
  {
    fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
    return 1;
  }
  if (TTF_Init() < 0)
  {
    fprintf(stderr, "TTF_Init: %s\n", TTF_GetError());
    return 1;
  }

  SDL_Window *win = SDL_CreateWindow("Tello Ground Station",
                                     SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                     WIN_W, WIN_H, SDL_WINDOW_RESIZABLE);
  SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
                                         SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!win || !ren)
  {
    fprintf(stderr, "SDL window/renderer: %s\n", SDL_GetError());
    return 1;
  }

  /* ----- Fonts ----- */
  const char *fpath = hud_find_font();
  if (!fpath)
    fprintf(stderr, "Warning: no monospace font found, HUD text disabled\n");
  else
    printf("Using font: %s\n", fpath);

  TTF_Font *font = NULL, *font_sm = NULL;
  int prev_ww = 0, prev_wh = 0;

  /* ----- Drone comms ----- */
  if (drone_init() < 0)
  {
    fprintf(stderr, "Failed to initialise drone socket\n");
    return 1;
  }
  drone_send("command");
  SDL_Delay(100);
  drone_send("streamon");

  /* ----- Background threads ----- */
  tracker_init();
  video_set_frame_callback(tracker_process_frame);
  video_start();
  telemetry_start();

  /* ----- Video texture ----- */
  SDL_Texture *vid_tex = NULL;
  int tex_w = 0, tex_h = 0;

  /* ----- Game controller ----- */
  SDL_GameController *gc = NULL;
  for (int i = 0; i < SDL_NumJoysticks(); i++)
  {
    if (SDL_IsGameController(i))
    {
      gc = SDL_GameControllerOpen(i);
      if (gc)
      {
        printf("Gamepad: %s\n", SDL_GameControllerName(gc));
        break;
      }
    }
  }

  Uint32 last_rc = 0;

  /* ===== Main loop ===== */
  while (g_running)
  {

    /* --- Events --- */
    SDL_Event ev;
    while (SDL_PollEvent(&ev))
    {
      switch (ev.type)
      {
      case SDL_QUIT:
        g_running = false;
        break;

      case SDL_KEYDOWN:
        /* Feed event to config menu first - it consumes keys when open */
        if (config_menu_handle_event(&ev))
          break;
        switch (ev.key.keysym.sym)
        {
        case SDLK_ESCAPE:
          g_running = false;
          break;
        case SDLK_F1:
          config_menu_toggle();
          break;
        case SDLK_t:
          drone_send("takeoff");
          break;
        case SDLK_l:
          drone_send("land");
          break;
        case SDLK_SPACE:
          drone_send("emergency");
          break;
        case SDLK_TAB:
          tracker_set_enabled(!tracker_is_enabled());
          printf("Tracking: %s\n", tracker_is_enabled() ? "ON" : "OFF");
          break;
        default:
          break;
        }
        break;

      case SDL_CONTROLLERBUTTONDOWN:
        if (config_menu_handle_event(&ev))
          break;
        switch (ev.cbutton.button)
        {
        case SDL_CONTROLLER_BUTTON_A:
          drone_send("takeoff");
          break;
        case SDL_CONTROLLER_BUTTON_B:
          drone_send("land");
          break;
        case SDL_CONTROLLER_BUTTON_X:
          drone_send("emergency");
          break;
        case SDL_CONTROLLER_BUTTON_Y:
          tracker_set_enabled(!tracker_is_enabled());
          printf("Tracking: %s\n", tracker_is_enabled() ? "ON" : "OFF");
          break;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
          drone_send("flip l");
          break;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
          drone_send("flip r");
          break;
        default:
          break;
        }
        break;

      case SDL_CONTROLLERDEVICEADDED:
        if (!gc && SDL_IsGameController(ev.cdevice.which))
          gc = SDL_GameControllerOpen(ev.cdevice.which);
        break;

      case SDL_CONTROLLERDEVICEREMOVED:
        if (gc)
        {
          SDL_GameControllerClose(gc);
          gc = NULL;
        }
        break;
      }
    }

    /* --- RC at 20 Hz --- */
    Uint32 now = SDL_GetTicks();
    if (!config_menu_is_open() && now - last_rc >= (Uint32)g_settings.rc_rate_ms)
    {
      /* --- Gesture commands (debounced) --- */
      static Uint32 last_gesture_cmd = 0;
      TrackerDebug dbg = tracker_get_debug();
      if (dbg.gesture == GESTURE_THUMBS_DOWN &&
          now - last_gesture_cmd >= (Uint32)g_settings.gesture_debounce_ms)
      {
        drone_send("land");
        last_gesture_cmd = now;
      }

      int roll = 0, pitch = 0, throttle = 0, yaw = 0;

      /* Tracker overrides manual input when active */
      TrackerOutput tr = tracker_get_output();
      if (tr.active)
      {
        roll = tr.roll;
        pitch = tr.pitch;
        throttle = tr.throttle;
        yaw = tr.yaw;
      }
      else if (gc)
      {
        input_read_gamepad(gc, &roll, &pitch, &throttle, &yaw);
      }
      else
      {
        input_read_keyboard(&roll, &pitch, &throttle, &yaw);
      }

      drone_rc(roll, pitch, throttle, yaw);
      last_rc = now;
    }

    /* --- Update video texture --- */
    const uint8_t *rgb;
    int fw, fh;
    if (video_lock_frame(&rgb, &fw, &fh))
    {
      if (!vid_tex || tex_w != fw || tex_h != fh)
      {
        if (vid_tex)
          SDL_DestroyTexture(vid_tex);
        vid_tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGB24,
                                    SDL_TEXTUREACCESS_STREAMING, fw, fh);
        tex_w = fw;
        tex_h = fh;
      }
      if (vid_tex)
        SDL_UpdateTexture(vid_tex, NULL, rgb, fw * 3);
      video_unlock_frame();
    }

    /* --- Render --- */
    int ww, wh;
    SDL_GetWindowSize(win, &ww, &wh);

    SDL_SetRenderDrawColor(ren, 30, 30, 45, 255);
    SDL_RenderClear(ren);

    if (vid_tex)
    {
      SDL_Rect dst = {0, 0, ww, wh};
      SDL_RenderCopy(ren, vid_tex, NULL, &dst);
    }

    /* Recreate fonts when window is resized */
    if (fpath && (ww != prev_ww || wh != prev_wh))
    {
      if (font)
        TTF_CloseFont(font);
      if (font_sm)
        TTF_CloseFont(font_sm);
      float s = ((float)ww / WIN_W + (float)wh / WIN_H) / 2.0f;
      int fs = (int)(16.0f * s);
      if (fs < 8)
        fs = 8;
      int fs_sm = (int)(13.0f * s);
      if (fs_sm < 8)
        fs_sm = 8;
      font = TTF_OpenFont(fpath, fs);
      font_sm = TTF_OpenFont(fpath, fs_sm);
      prev_ww = ww;
      prev_wh = wh;
    }

    HandLandmarks lm = tracker_get_landmarks();
    hud_draw(ren, font, font_sm, ww, wh, gc != NULL,
             &lm, tracker_is_enabled());
    config_menu_draw(ren, font, font_sm, ww, wh);
    SDL_RenderPresent(ren);
  }

  /* ===== Shutdown ===== */
  drone_send("streamoff");
  drone_send("land");
  g_running = false;

  video_stop();
  telemetry_stop();
  tracker_cleanup();

  if (vid_tex)
    SDL_DestroyTexture(vid_tex);
  if (font)
    TTF_CloseFont(font);
  if (font_sm)
    TTF_CloseFont(font_sm);
  if (gc)
    SDL_GameControllerClose(gc);

  drone_cleanup();

  TTF_Quit();
  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(win);
  SDL_Quit();
  return 0;
}
