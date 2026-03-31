#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "drone.h"
#include "app.h"
#include "config.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

volatile bool drone_connected = false;

static int cmd_sock = -1;
static struct sockaddr_in tello_addr;
static pthread_t resp_tid;

/* Background thread: reads "ok"/"error" responses from the drone. */
static void *response_thread(void *arg)
{
    (void)arg;

    struct timeval tv = {.tv_sec = 2};
    setsockopt(cmd_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char buf[256];
    while (g_running)
    {
        ssize_t n = recvfrom(cmd_sock, buf, sizeof(buf) - 1, 0, NULL, NULL);
        if (n > 0)
        {
            buf[n] = '\0';
            fprintf(stderr, "[drone] response: %s\n", buf);
            drone_connected = true;
        }
    }
    return NULL;
}

int drone_init(void)
{
    cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (cmd_sock < 0)
    {
        perror("[drone] socket");
        return -1;
    }

    int reuse = 1;
    setsockopt(cmd_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    /* Bind to CMD_PORT locally BEFORE any sendto(), so the OS doesn't
     * auto-bind to a random ephemeral port and the drone knows where
     * to send its "ok" responses. */
    struct sockaddr_in local = {0};
    local.sin_family = AF_INET;
    local.sin_port = htons(CMD_PORT);
    local.sin_addr.s_addr = INADDR_ANY;
    if (bind(cmd_sock, (struct sockaddr *)&local, sizeof(local)) < 0)
    {
        perror("[drone] bind cmd socket");
        /* Non-fatal: we can still send, just won't receive responses */
    }
    else
    {
        fprintf(stderr, "[drone] cmd socket bound to local port %d\n", CMD_PORT);
    }

    memset(&tello_addr, 0, sizeof(tello_addr));
    tello_addr.sin_family = AF_INET;
    tello_addr.sin_port = htons(CMD_PORT);
    inet_pton(AF_INET, TELLO_IP, &tello_addr.sin_addr);

    fprintf(stderr, "[drone] target %s:%d\n", TELLO_IP, CMD_PORT);

    pthread_create(&resp_tid, NULL, response_thread, NULL);
    return 0;
}

void drone_send(const char *cmd)
{
    if (cmd_sock < 0)
        return;
    /* Don't flood stderr with rc heartbeats - only log non-rc commands. */
    if (strncmp(cmd, "rc ", 3) != 0)
        fprintf(stderr, "[drone] send: %s\n", cmd);
    sendto(cmd_sock, cmd, strlen(cmd), 0,
           (struct sockaddr *)&tello_addr, sizeof(tello_addr));
}

static inline int clamp100(int v)
{
    return v < -100 ? -100 : (v > 100 ? 100 : v);
}

void drone_rc(int roll, int pitch, int throttle, int yaw)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "rc %d %d %d %d",
             clamp100(roll), clamp100(pitch),
             clamp100(throttle), clamp100(yaw));
    drone_send(buf);
}

void drone_cleanup(void)
{
    pthread_join(resp_tid, NULL);
    if (cmd_sock >= 0)
    {
        close(cmd_sock);
        cmd_sock = -1;
    }
}
