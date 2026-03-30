#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "telemetry.h"
#include "drone.h"      /* drone_connected */
#include "app.h"
#include "config.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static pthread_t       state_tid;
static pthread_mutex_t telem_lock = PTHREAD_MUTEX_INITIALIZER;
static Telemetry       telem;

static void parse_telem(const char *raw) {
    Telemetry t = {0};
    char buf[512];
    strncpy(buf, raw, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *save = NULL;
    for (char *tok = strtok_r(buf, ";", &save); tok;
         tok = strtok_r(NULL, ";", &save)) {
        while (*tok == ' ') tok++;
        char *colon = strchr(tok, ':');
        if (!colon) continue;
        *colon = '\0';
        int val = atoi(colon + 1);

        if      (!strcmp(tok, "bat"))   t.bat         = val;
        else if (!strcmp(tok, "h"))     t.height      = val;
        else if (!strcmp(tok, "time"))  t.flight_time = val;
        else if (!strcmp(tok, "pitch")) t.pitch       = val;
        else if (!strcmp(tok, "roll"))  t.roll        = val;
        else if (!strcmp(tok, "yaw"))   t.yaw         = val;
        else if (!strcmp(tok, "vgx"))   t.vgx         = val;
        else if (!strcmp(tok, "vgy"))   t.vgy         = val;
        else if (!strcmp(tok, "vgz"))   t.vgz         = val;
        else if (!strcmp(tok, "templ")) t.templ       = val;
        else if (!strcmp(tok, "temph")) t.temph       = val;
        else if (!strcmp(tok, "tof"))   t.tof         = val;
    }
    t.valid = true;

    pthread_mutex_lock(&telem_lock);
    telem = t;
    pthread_mutex_unlock(&telem_lock);
}

static void *state_thread(void *arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("state socket"); return NULL; }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(STATE_PORT);
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

void telemetry_start(void) {
    pthread_create(&state_tid, NULL, state_thread, NULL);
}

void telemetry_stop(void) {
    pthread_join(state_tid, NULL);
}

void telemetry_get(Telemetry *out) {
    pthread_mutex_lock(&telem_lock);
    *out = telem;
    pthread_mutex_unlock(&telem_lock);
}
