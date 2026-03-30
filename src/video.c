#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "video.h"
#include "app.h"
#include "config.h"

#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static pthread_t          vid_tid;
static pthread_mutex_t    frame_lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t           *frame_rgb  = NULL;
static int                frame_w    = 0;
static int                frame_h    = 0;
static bool               frame_new  = false;
static VideoFrameCallback frame_cb   = NULL;

void video_set_frame_callback(VideoFrameCallback cb) {
    frame_cb = cb;
}

static void *video_thread(void *arg) {
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("video socket"); return NULL; }

    int reuse = 1, rcvbuf = 512 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse,   sizeof(reuse));
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF,   &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(VIDEO_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("video bind");
        close(sock);
        return NULL;
    }

    struct timeval tv = {.tv_sec = 2};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* FFmpeg H.264 decoder setup */
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        fprintf(stderr, "H264 codec not found\n");
        close(sock);
        return NULL;
    }

    AVCodecContext *ctx = avcodec_alloc_context3(codec);
    ctx->flags      |= AV_CODEC_FLAG_LOW_DELAY;
    ctx->thread_count = 1;
    if (avcodec_open2(ctx, codec, NULL) < 0) {
        fprintf(stderr, "Could not open H264 codec\n");
        avcodec_free_context(&ctx);
        close(sock);
        return NULL;
    }

    AVCodecParserContext *parser   = av_parser_init(AV_CODEC_ID_H264);
    AVPacket             *pkt      = av_packet_alloc();
    AVFrame              *frame    = av_frame_alloc();
    struct SwsContext    *sws      = NULL;
    uint8_t              *rgb_buf  = NULL;
    int                   rgb_stride = 0;

    uint8_t recv_buf[2048];

    while (g_running) {
        ssize_t n = recvfrom(sock, recv_buf, sizeof(recv_buf), 0, NULL, NULL);
        if (n <= 0) continue;

        uint8_t *data      = recv_buf;
        int      data_size = (int)n;

        while (data_size > 0) {
            int consumed = av_parser_parse2(parser, ctx,
                &pkt->data, &pkt->size, data, data_size,
                AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
            if (consumed < 0) break;
            data      += consumed;
            data_size -= consumed;

            if (pkt->size == 0) continue;
            if (avcodec_send_packet(ctx, pkt) < 0) continue;

            while (avcodec_receive_frame(ctx, frame) == 0) {
                int w = frame->width, h = frame->height;
                if (w <= 0 || h <= 0) continue;

                /* (Re)init swscale on dimension change */
                if (!sws || rgb_stride != w * 3) {
                    if (sws) sws_freeContext(sws);
                    sws = sws_getContext(w, h, frame->format,
                                         w, h, AV_PIX_FMT_RGB24,
                                         SWS_FAST_BILINEAR, NULL, NULL, NULL);
                    rgb_stride = w * 3;
                    free(rgb_buf);
                    rgb_buf = malloc((size_t)(w * h * 3));
                }

                uint8_t *dst[1]    = {rgb_buf};
                int dst_stride[1]  = {rgb_stride};
                sws_scale(sws, (const uint8_t *const *)frame->data,
                          frame->linesize, 0, h, dst, dst_stride);

                pthread_mutex_lock(&frame_lock);
                if (!frame_rgb || frame_w != w || frame_h != h) {
                    free(frame_rgb);
                    frame_rgb = malloc((size_t)(w * h * 3));
                }
                memcpy(frame_rgb, rgb_buf, (size_t)(w * h * 3));
                frame_w   = w;
                frame_h   = h;
                frame_new = true;

                /* Notify tracker (or any subscriber) while still under lock */
                if (frame_cb)
                    frame_cb(frame_rgb, frame_w, frame_h);

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

void video_start(void) {
    pthread_create(&vid_tid, NULL, video_thread, NULL);
}

void video_stop(void) {
    pthread_join(vid_tid, NULL);
    free(frame_rgb);
    frame_rgb = NULL;
}

bool video_lock_frame(const uint8_t **rgb, int *w, int *h) {
    pthread_mutex_lock(&frame_lock);
    if (!frame_new || !frame_rgb) {
        pthread_mutex_unlock(&frame_lock);
        return false;
    }
    *rgb = frame_rgb;
    *w   = frame_w;
    *h   = frame_h;
    return true;  /* caller must call video_unlock_frame() */
}

void video_unlock_frame(void) {
    frame_new = false;
    pthread_mutex_unlock(&frame_lock);
}
