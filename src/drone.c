#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "drone.h"
#include "config.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

volatile bool drone_connected = false;

static int               cmd_sock  = -1;
static struct sockaddr_in tello_addr;

int drone_init(void) {
    cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (cmd_sock < 0) { perror("drone socket"); return -1; }

    int reuse = 1;
    setsockopt(cmd_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&tello_addr, 0, sizeof(tello_addr));
    tello_addr.sin_family = AF_INET;
    tello_addr.sin_port   = htons(CMD_PORT);
    inet_pton(AF_INET, TELLO_IP, &tello_addr.sin_addr);

    return 0;
}

void drone_send(const char *cmd) {
    if (cmd_sock >= 0)
        sendto(cmd_sock, cmd, strlen(cmd), 0,
               (struct sockaddr *)&tello_addr, sizeof(tello_addr));
}

static inline int clamp100(int v) {
    return v < -100 ? -100 : (v > 100 ? 100 : v);
}

void drone_rc(int roll, int pitch, int throttle, int yaw) {
    char buf[64];
    snprintf(buf, sizeof(buf), "rc %d %d %d %d",
             clamp100(roll), clamp100(pitch),
             clamp100(throttle), clamp100(yaw));
    drone_send(buf);
}

void drone_cleanup(void) {
    if (cmd_sock >= 0) {
        close(cmd_sock);
        cmd_sock = -1;
    }
}
