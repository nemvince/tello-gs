/*
 * Tello Ground Station PoC
 * Single-file C implementation: video feed + HUD + Xbox controller
 * Deps: SDL2, SDL2_ttf, FFmpeg (libavcodec, libavutil, libswscale)
 * Build: make
*/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

/* ---- Config ---- */
#define TELLO_IP    "192.168.10.1"
#define CMD_PORT    8889
#define STATE_PORT  8890
#define VIDEO_PORT  11111
#define WIN_W       960
#define WIN_H       720
#define RC_RATE_MS  50
#define DEADZONE    0.15f
#define EXPO        2.0f

/* ---- Shared state ---- */
typedef struct {
    int bat, height, flight_time;
    int pitch, roll, yaw;
    int vgx, vgy, vgz;
    int templ, temph, tof;
    bool valid;
} Telemetry;

static pthread_mutex_t telem_lock = PTHREAD_MUTEX_INITIALIZER;
static Telemetry telem;

static pthread_mutex_t frame_lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t *frame_rgb = NULL;
static int frame_w = 0, frame_h = 0;
static bool frame_new = false;

static volatile bool g_running = true;
static volatile bool drone_connected = false;
static int cmd_sock = -1;
static struct sockaddr_in tello_addr;

/* ---- Drone comms ---- */

static void tello_send(const char *cmd) {
    if (cmd_sock >= 0)
        sendto(cmd_sock, cmd, strlen(cmd), 0,
               (struct sockaddr *)&tello_addr, sizeof(tello_addr));
}

static inline int clamp100(int v) { return v < -100 ? -100 : (v > 100 ? 100 : v); }

static void tello_rc(int roll, int pitch, int throttle, int yaw) {
    char buf[64];
    snprintf(buf, sizeof(buf), "rc %d %d %d %d",
             clamp100(roll), clamp100(pitch), clamp100(throttle), clamp100(yaw));
    tello_send(buf);
}

/* ---- State receiver thread ---- */

static void parse_telem(const char *raw) {
    Telemetry t = {0};
    char buf[512];
    strncpy(buf, raw, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *save = NULL;
    for (char *tok = strtok_r(buf, ";", &save); tok; tok = strtok_r(NULL, ";", &save)) {
        while (*tok == ' ') tok++;
        char *colon = strchr(tok, ':');
        if (!colon) continue;
        *colon = '\0';
        int val = atoi(colon + 1);

        if      (!strcmp(tok, "bat"))   t.bat = val;
        else if (!strcmp(tok, "h"))     t.height = val;
        else if (!strcmp(tok, "time"))  t.flight_time = val;
        else if (!strcmp(tok, "pitch")) t.pitch = val;
        else if (!strcmp(tok, "roll"))  t.roll = val;
        else if (!strcmp(tok, "yaw"))   t.yaw = val;
        else if (!strcmp(tok, "vgx"))   t.vgx = val;
        else if (!strcmp(tok, "vgy"))   t.vgy = val;
        else if (!strcmp(tok, "vgz"))   t.vgz = val;
        else if (!strcmp(tok, "templ")) t.templ = val;
        else if (!strcmp(tok, "temph")) t.temph = val;
        else if (!strcmp(tok, "tof"))   t.tof = val;
    }
    t.valid = true;

    pthread_mutex_lock(&telem_lock);
    telem = t;
    pthread_mutex_unlock(&telem_lock);
}

static void *state_thread(void *arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return NULL;

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(STATE_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("state bind");
        close(sock);
        return NULL;
    }

    struct timeval tv = {.tv_sec = 2};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char buf[1024];
    while (g_running) {
        ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0, NULL, NULL);
        if (n > 0) {
            buf[n] = '\0';
            parse_telem(buf);
            drone_connected = true;
        }
    }
    close(sock);
    return NULL;
}

/* ---- Video receiver / decoder thread ---- */

static void *video_thread(void *arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return NULL;

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    int rcvbuf = 512 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(VIDEO_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("video bind");
        close(sock);
        return NULL;
    }

    struct timeval tv = {.tv_sec = 2};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* FFmpeg decoder setup */
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) { fprintf(stderr, "H264 codec not found\n"); close(sock); return NULL; }

    AVCodecContext *ctx = avcodec_alloc_context3(codec);
    ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    ctx->thread_count = 1;
    if (avcodec_open2(ctx, codec, NULL) < 0) {
        fprintf(stderr, "Could not open H264 codec\n");
        avcodec_free_context(&ctx);
        close(sock);
        return NULL;
    }

    AVCodecParserContext *parser = av_parser_init(AV_CODEC_ID_H264);
    AVPacket *pkt = av_packet_alloc();
    AVFrame *frame = av_frame_alloc();
    struct SwsContext *sws = NULL;
    uint8_t *rgb_buf = NULL;
    int rgb_stride = 0;

    uint8_t recv_buf[2048];

    while (g_running) {
        ssize_t n = recvfrom(sock, recv_buf, sizeof(recv_buf), 0, NULL, NULL);
        if (n <= 0) continue;

        uint8_t *data = recv_buf;
        int data_size = (int)n;

        while (data_size > 0) {
            int consumed = av_parser_parse2(parser, ctx,
                &pkt->data, &pkt->size, data, data_size,
                AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
            if (consumed < 0) break;
            data += consumed;
            data_size -= consumed;

            if (pkt->size == 0) continue;
            if (avcodec_send_packet(ctx, pkt) < 0) continue;

            while (avcodec_receive_frame(ctx, frame) == 0) {
                int w = frame->width, h = frame->height;
                if (w <= 0 || h <= 0) continue;

                /* (Re)init swscale if dimensions changed */
                if (!sws || rgb_stride != w * 3) {
                    if (sws) sws_freeContext(sws);
                    sws = sws_getContext(w, h, frame->format,
                                         w, h, AV_PIX_FMT_RGB24,
                                         SWS_FAST_BILINEAR, NULL, NULL, NULL);
                    rgb_stride = w * 3;
                    free(rgb_buf);
                    rgb_buf = malloc(w * h * 3);
                }

                uint8_t *dst[1] = {rgb_buf};
                int dst_stride[1] = {rgb_stride};
                sws_scale(sws, (const uint8_t *const *)frame->data,
                          frame->linesize, 0, h, dst, dst_stride);

                pthread_mutex_lock(&frame_lock);
                if (!frame_rgb || frame_w != w || frame_h != h) {
                    free(frame_rgb);
                    frame_rgb = malloc(w * h * 3);
                }
                memcpy(frame_rgb, rgb_buf, w * h * 3);
                frame_w = w;
                frame_h = h;
                frame_new = true;
                pthread_mutex_unlock(&frame_lock);
            }
        }
    }

    free(rgb_buf);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    av_parser_close(parser);
    avcodec_free_context(&ctx);
    if (sws) sws_freeContext(sws);
    close(sock);
    return NULL;
}

/* ---- HUD drawing helpers ---- */

static void draw_text(SDL_Renderer *r, TTF_Font *font,
                      int x, int y, SDL_Color col, const char *fmt, ...) {
    if (!font) return;
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (buf[0] == '\0') return;

    SDL_Surface *s = TTF_RenderText_Blended(font, buf, col);
    if (!s) return;
    SDL_Texture *t = SDL_CreateTextureFromSurface(r, s);
    if (t) {
        SDL_Rect dst = {x, y, s->w, s->h};
        SDL_RenderCopy(r, t, NULL, &dst);
        SDL_DestroyTexture(t);
    }
    SDL_FreeSurface(s);
}

static void fill_rect(SDL_Renderer *r, int x, int y, int w, int h,
                       Uint8 cr, Uint8 cg, Uint8 cb, Uint8 ca) {
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(r, cr, cg, cb, ca);
    SDL_Rect rc = {x, y, w, h};
    SDL_RenderFillRect(r, &rc);
}

static void draw_hud(SDL_Renderer *r, TTF_Font *font, TTF_Font *font_sm,
                     int ww, int wh, bool gamepad) {
    Telemetry t;
    pthread_mutex_lock(&telem_lock);
    t = telem;
    pthread_mutex_unlock(&telem_lock);

    float sx = (float)ww / WIN_W;
    float sy = (float)wh / WIN_H;
    #define SX(v) ((int)((v) * sx))
    #define SY(v) ((int)((v) * sy))

    SDL_Color white  = {255, 255, 255, 255};
    SDL_Color green  = {0,   220, 80,  255};
    SDL_Color red    = {255, 50,  50,  255};
    SDL_Color cyan   = {0,   220, 255, 255};
    SDL_Color yellow = {255, 220, 0,   255};
    SDL_Color orange = {255, 160, 0,   255};

    /* --- Top bar --- */
    fill_rect(r, 0, 0, ww, SY(32), 0, 0, 0, 160);
    SDL_Color bat_col = t.bat > 50 ? green : (t.bat > 20 ? yellow : red);
    draw_text(r, font, SX(10), SY(6), bat_col, "BAT %d%%", t.bat);
    draw_text(r, font, ww / 2 - SX(50), SY(6), white, "TIME %02d:%02d",
              t.flight_time / 60, t.flight_time % 60);
    draw_text(r, font, ww - SX(160), SY(6), white, "TEMP %d-%dC",
              t.templ, t.temph);

    /* --- Speed panel (left) --- */
    fill_rect(r, SX(6), SY(50), SX(140), SY(55), 0, 0, 0, 160);
    float spd = sqrtf((float)(t.vgx*t.vgx + t.vgy*t.vgy + t.vgz*t.vgz));
    draw_text(r, font, SX(12), SY(52), green, "SPD %.1f", spd);
    draw_text(r, font_sm, SX(12), SY(76), white,
              "X%+d Y%+d Z%+d", t.vgx, t.vgy, t.vgz);

    /* --- Altitude bar (right) --- */
    int bar_x = ww - SX(44), bar_y = SY(50), bar_h = SY(220);
    fill_rect(r, bar_x - SX(6), bar_y - SY(6), SX(44), bar_h + SY(32), 0, 0, 0, 160);

    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(r, 255, 255, 255, 200);
    SDL_Rect alt_out = {bar_x, bar_y, SX(24), bar_h};
    SDL_RenderDrawRect(r, &alt_out);

    int fill = (int)(bar_h * fminf(1.0f, fmaxf(0.0f, t.height / 300.0f)));
    if (fill > 0)
        fill_rect(r, bar_x + 1, bar_y + bar_h - fill, SX(24) - 2, fill,
                  0, 220, 255, 200);
    draw_text(r, font_sm, bar_x - SX(4), bar_y + bar_h + SY(4), cyan,
              "%dcm", t.height);

    /* --- Attitude indicator (center) --- */
    int cx = ww / 2, cy = wh / 2, rad = SX(55);
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(r, 255, 255, 255, 100);
    for (int i = 0; i < 64; i++) {
        float a1 = i / 64.0f * 2.0f * (float)M_PI;
        float a2 = (i + 1) / 64.0f * 2.0f * (float)M_PI;
        SDL_RenderDrawLine(r, cx + (int)(cosf(a1) * rad), cy + (int)(sinf(a1) * rad),
                              cx + (int)(cosf(a2) * rad), cy + (int)(sinf(a2) * rad));
    }
    float roll_r = t.roll * (float)M_PI / 180.0f;
    int pitch_off = (int)(t.pitch * rad / 45.0f);
    int dx = (int)(cosf(-roll_r) * rad * 0.8f);
    int dy = (int)(sinf(-roll_r) * rad * 0.8f);
    SDL_SetRenderDrawColor(r, 0, 220, 80, 220);
    SDL_RenderDrawLine(r, cx - dx, cy + pitch_off - dy, cx + dx, cy + pitch_off + dy);
    /* crosshair */
    SDL_SetRenderDrawColor(r, 255, 220, 0, 220);
    SDL_RenderDrawLine(r, cx - SX(8), cy, cx + SX(8), cy);
    SDL_RenderDrawLine(r, cx, cy - SY(8), cx, cy + SY(8));

    /* --- Compass --- */
    static const char *dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
    int yaw_pos = ((t.yaw % 360) + 360) % 360;
    int di = ((yaw_pos + 22) / 45) % 8;
    fill_rect(r, ww / 2 - SX(60), wh - SY(68), SX(120), SY(24), 0, 0, 0, 160);
    draw_text(r, font, ww / 2 - SX(48), wh - SY(66), orange,
              "%03d %s", yaw_pos, dirs[di]);

    /* --- Bottom bar --- */
    fill_rect(r, 0, wh - SY(28), ww, SY(28), 0, 0, 0, 160);
    draw_text(r, font_sm, SX(10), wh - SY(24),
              drone_connected ? green : red,
              drone_connected ? "DRONE CONNECTED" : "DISCONNECTED");
    draw_text(r, font_sm, ww / 2 - SX(35), wh - SY(24),
              gamepad ? green : yellow, gamepad ? "GAMEPAD" : "KEYBOARD");
    draw_text(r, font_sm, ww - SX(110), wh - SY(24), white,
              "TOF %dcm", t.tof);

    #undef SX
    #undef SY
}

/* ---- Controller helpers ---- */

static int apply_stick(float raw) {
    if (fabsf(raw) < DEADZONE) return 0;
    float sign = raw > 0 ? 1.0f : -1.0f;
    float norm = (fabsf(raw) - DEADZONE) / (1.0f - DEADZONE);
    return (int)(sign * powf(norm, EXPO) * 100.0f);
}

/* ---- Font search ---- */

static const char *font_paths[] = {
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    "/usr/share/fonts/liberation-mono/LiberationMono-Regular.ttf",
    "/usr/share/fonts/noto/NotoSansMono-Regular.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
    "/usr/share/fonts/google-noto/NotoSansMono-Regular.ttf",
    "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
    NULL
};

static const char *find_font(void) {
    for (int i = 0; font_paths[i]; i++) {
        if (access(font_paths[i], R_OK) == 0) return font_paths[i];
    }
    return NULL;
}

/* ---- Main ---- */

int main(int argc, char **argv) {
    (void)argc; (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    if (TTF_Init() < 0) {
        fprintf(stderr, "TTF_Init: %s\n", TTF_GetError());
        return 1;
    }

    SDL_Window *win = SDL_CreateWindow("Tello Ground Station",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H, SDL_WINDOW_RESIZABLE);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!win || !ren) {
        fprintf(stderr, "SDL window/renderer: %s\n", SDL_GetError());
        return 1;
    }

    /* Find and open font */
    const char *fpath = find_font();
    if (!fpath) {
        fprintf(stderr, "Warning: no monospace font found, HUD text disabled\n");
    } else {
        printf("Using font: %s\n", fpath);
    }

    TTF_Font *font = NULL, *font_sm = NULL;
    int prev_ww = 0, prev_wh = 0;

    /* Command socket */
    cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
    int reuse = 1;
    setsockopt(cmd_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&tello_addr, 0, sizeof(tello_addr));
    tello_addr.sin_family = AF_INET;
    tello_addr.sin_port = htons(CMD_PORT);
    inet_pton(AF_INET, TELLO_IP, &tello_addr.sin_addr);

    /* Enter SDK mode and start video */
    tello_send("command");
    SDL_Delay(100);
    tello_send("streamon");

    /* Start background threads */
    pthread_t vid_tid, st_tid;
    pthread_create(&vid_tid, NULL, video_thread, NULL);
    pthread_create(&st_tid, NULL, state_thread, NULL);

    /* Video texture */
    SDL_Texture *vid_tex = NULL;
    int tex_w = 0, tex_h = 0;

    /* Game controller */
    SDL_GameController *gc = NULL;
    for (int i = 0; i < SDL_NumJoysticks(); i++) {
        if (SDL_IsGameController(i)) {
            gc = SDL_GameControllerOpen(i);
            if (gc) { printf("Gamepad: %s\n", SDL_GameControllerName(gc)); break; }
        }
    }

    Uint32 last_rc = 0;

    while (g_running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT:
                g_running = false;
                break;
            case SDL_KEYDOWN:
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: g_running = false; break;
                case SDLK_t: tello_send("takeoff"); break;
                case SDLK_l: tello_send("land"); break;
                case SDLK_SPACE: tello_send("emergency"); break;
                default: break;
                }
                break;
            case SDL_CONTROLLERBUTTONDOWN:
                switch (ev.cbutton.button) {
                case SDL_CONTROLLER_BUTTON_A: tello_send("takeoff"); break;
                case SDL_CONTROLLER_BUTTON_B: tello_send("land"); break;
                case SDL_CONTROLLER_BUTTON_X: tello_send("emergency"); break;
                case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:  tello_send("flip l"); break;
                case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: tello_send("flip r"); break;
                default: break;
                }
                break;
            case SDL_CONTROLLERDEVICEADDED:
                if (!gc && SDL_IsGameController(ev.cdevice.which))
                    gc = SDL_GameControllerOpen(ev.cdevice.which);
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                if (gc) { SDL_GameControllerClose(gc); gc = NULL; }
                break;
            }
        }

        /* RC control @ 20 Hz */
        Uint32 now = SDL_GetTicks();
        if (now - last_rc >= RC_RATE_MS) {
            int roll = 0, pitch = 0, throttle = 0, yaw = 0;
            if (gc) {
                roll     =  apply_stick(SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTX)  / 32767.0f);
                pitch    = -apply_stick(SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTY)  / 32767.0f);
                yaw      =  apply_stick(SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_RIGHTX) / 32767.0f);
                throttle = -apply_stick(SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_RIGHTY) / 32767.0f);
            } else {
                const Uint8 *k = SDL_GetKeyboardState(NULL);
                int spd = 60;
                if (k[SDL_SCANCODE_D])     roll = spd;
                if (k[SDL_SCANCODE_A])     roll = -spd;
                if (k[SDL_SCANCODE_W])     pitch = spd;
                if (k[SDL_SCANCODE_S])     pitch = -spd;
                if (k[SDL_SCANCODE_UP])    throttle = spd;
                if (k[SDL_SCANCODE_DOWN])  throttle = -spd;
                if (k[SDL_SCANCODE_RIGHT]) yaw = spd;
                if (k[SDL_SCANCODE_LEFT])  yaw = -spd;
            }
            tello_rc(roll, pitch, throttle, yaw);
            last_rc = now;
        }

        /* Update video texture from decoded frame */
        pthread_mutex_lock(&frame_lock);
        if (frame_new && frame_rgb) {
            if (!vid_tex || tex_w != frame_w || tex_h != frame_h) {
                if (vid_tex) SDL_DestroyTexture(vid_tex);
                vid_tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGB24,
                    SDL_TEXTUREACCESS_STREAMING, frame_w, frame_h);
                tex_w = frame_w;
                tex_h = frame_h;
            }
            if (vid_tex)
                SDL_UpdateTexture(vid_tex, NULL, frame_rgb, frame_w * 3);
            frame_new = false;
        }
        pthread_mutex_unlock(&frame_lock);

        /* Render */
        int ww, wh;
        SDL_GetWindowSize(win, &ww, &wh);

        SDL_SetRenderDrawColor(ren, 30, 30, 45, 255);
        SDL_RenderClear(ren);

        if (vid_tex) {
            SDL_Rect dst = {0, 0, ww, wh};
            SDL_RenderCopy(ren, vid_tex, NULL, &dst);
        }

        /* Recreate fonts if window size changed */
        if (fpath && (ww != prev_ww || wh != prev_wh)) {
            if (font)    TTF_CloseFont(font);
            if (font_sm) TTF_CloseFont(font_sm);
            float s = ((float)ww / WIN_W + (float)wh / WIN_H) / 2.0f;
            int fs = (int)(16.0f * s);
            int fs_sm = (int)(13.0f * s);
            if (fs < 8) fs = 8;
            if (fs_sm < 8) fs_sm = 8;
            font    = TTF_OpenFont(fpath, fs);
            font_sm = TTF_OpenFont(fpath, fs_sm);
            prev_ww = ww;
            prev_wh = wh;
        }

        draw_hud(ren, font, font_sm, ww, wh, gc != NULL);

        SDL_RenderPresent(ren);
    }

    /* Cleanup */
    tello_send("streamoff");
    tello_send("land");
    g_running = false;

    pthread_join(vid_tid, NULL);
    pthread_join(st_tid, NULL);

    if (vid_tex) SDL_DestroyTexture(vid_tex);
    if (font)    TTF_CloseFont(font);
    if (font_sm) TTF_CloseFont(font_sm);
    if (gc)      SDL_GameControllerClose(gc);
    free(frame_rgb);
    close(cmd_sock);

    TTF_Quit();
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
