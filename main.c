
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <errno.h>

#include "arm_memmap.h"
#include "io.h"

#define PERRORF(fmt, ...) \
    fprintf(stderr, fmt ": %s\n", __VA_ARGS__, strerror(errno))

#define LISTEN_PORT "50000"
#define SCENE_MAGIC 0x5343454E


int MEMFD = -1; // fd to dev/mem
void* SDRAM;
void* LWBRIDGE; // todo: move trigger to sysfs

static socket_t LISTEN_SOCK = INV_SOCKET;
static socket_t ACCEPT_SOCK = INV_SOCKET;

// Map pointer to a memory-mapped device.
void* mmap_dev(unsigned int base, unsigned int span)
{
    void* virt = mmap(NULL, span, (PROT_READ | PROT_WRITE), MAP_SHARED, MEMFD, base);
    if (virt == MAP_FAILED) {
        PERRORF("mmap failed for 0x%08x", base);
        close(MEMFD);
        return NULL;
    }
    return virt;
}

// Unmap pointer to memory-mapped device.
int munmap_dev(void* virt, unsigned int span)
{
    int e = munmap(virt, span);
    if (e != 0) {
        perror("munmap() failed\n");
    }
    return e;
}

int map_devices(void)
{
    MEMFD = open("/dev/mem", (O_RDWR | O_SYNC));
    if (MEMFD == -1) {
        fprintf(stderr, "failed to open /dev/mem\n");
        return -1;
    }

    SDRAM = mmap_dev(SDRAM_BASE, SDRAM_SPAN);
    if (!SDRAM) { return -1; }
    LWBRIDGE = mmap_dev(LW_BRIDGE_BASE, LW_BRIDGE_SPAN);
    if (!LWBRIDGE) {
        munmap_dev(SDRAM, SDRAM_SPAN);
        return -1;
    }

    return 0;
}

void unmap_devices()
{
    munmap_dev(SDRAM, SDRAM_SPAN);
    munmap_dev(LWBRIDGE, LW_BRIDGE_SPAN);
    close(MEMFD);
}

static volatile sig_atomic_t sigint = 0;

// strlen may not be signal safe in our version of linux?
#define SIGINT_MSG "Received ctrl+c, quitting...\n"
#define SIGINT_MSGLEN (sizeof(SIGINT_MSG)-1)

#define QUIT_IF_SIGINT if (sigint == 1) { goto fail; }

void sigint_handler(int signum) 
{
    (void)signum;
    if (sigint == 0) {
        // fprintf is not signal-safe, use write
        ssize_t e = write(STDERR_FILENO, SIGINT_MSG, SIGINT_MSGLEN);
        (void)e; // gcc Wunused-result

        TCP_close(ACCEPT_SOCK);
        TCP_close(LISTEN_SOCK);
        ACCEPT_SOCK = INV_SOCKET;
        LISTEN_SOCK = INV_SOCKET;
        
        sigint = 1;
    }
}

extern int raytrace(unsigned* data, int size, bool cam_fit_x, int max_bounces, char** pimg, int* pimg_size);

int main(int argc, char** argv)
{
    bool verbose = false;
    bool cam_fit_x = false;
    int max_bounces = 3;

    if (argc > 1) {
        for (int i = 1; i < argc; ++i) 
        {
            if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
                verbose = true;
            } else if (strcmp(argv[i], "--cam-fit_x") == 0) {
                cam_fit_x = true;
            } else if (strcmp(argv[i], "--cam_fit_y") == 0) {
                cam_fit_x = false;
            } else if (strcmp(argv[i], "--disable_refl") == 0) {
                max_bounces = 0;
            } else {
                printf("Unrecognized option %s\n", argv[i]);
                return -1;
            }
        }
    }

    if (map_devices() != 0) {
        return -1;
    }

    // Install ctrl+c handler to allow server to gracefully quit
    // (otherwise TCP port can be broken until OS restart)
    struct sigaction act;
    act.sa_flags = 0;
    act.sa_handler = sigint_handler;
    if (sigaction(SIGINT, &act, NULL) != 0) {
        fprintf(stderr, "failed to set sigint handler");
        goto fail;
    }

    LISTEN_SOCK = TCP_listen2(LISTEN_PORT, true, verbose);
    if (LISTEN_SOCK == INV_SOCKET) { goto fail; }

#define DASHES "----------------------------\n"
#define ABANDON_MSG "Abandoned connection.\n"

    // Perform incoming raytracing jobs forever
    while (true) 
    {
        QUIT_IF_SIGINT

        if (verbose) { printf(DASHES); }
        printf("Waiting for connection from client...\n");
        if (verbose) { printf(DASHES); }

        // Wait for a connection from host
        ACCEPT_SOCK = TCP_accept2(LISTEN_SOCK, verbose);
        if (ACCEPT_SOCK == INV_SOCKET) { 
            TCP_close(LISTEN_SOCK);
            goto fail;
        }

        // Receive scene. 
        // If client fails to transmit, abandon and wait for new connection
        char* recvbuf;
        int nrecv = TCP_recv2(ACCEPT_SOCK, &recvbuf, verbose);
        if (nrecv == -1) {
            fprintf(stderr, ABANDON_MSG);
            continue;
        }
        // must be 32-bit aligned, with magic number
        else if (nrecv < 4 || (nrecv % 4) != 0) {
            free(recvbuf);
            TCP_close(ACCEPT_SOCK);
            fprintf(stderr, "Invalid recv size\n" ABANDON_MSG);
            continue;
        }

        unsigned* data = (unsigned*)recvbuf;
        if (data[0] != SCENE_MAGIC) {
            free(recvbuf);
            TCP_close(ACCEPT_SOCK);
            fprintf(stderr, "Endian check failed\n" ABANDON_MSG);
            continue;
        } 
        
        char* sendbuf;
        int img_size;
        if (raytrace(data, nrecv / 4, cam_fit_x, max_bounces, &sendbuf, &img_size) != 0) {
            free(recvbuf);
            goto fail_rt;
        }
        free(recvbuf);
        
        QUIT_IF_SIGINT

        if (TCP_send2(ACCEPT_SOCK, sendbuf, img_size, verbose) != img_size) {
            free(sendbuf);
            fprintf(stderr, ABANDON_MSG);
            continue;
        }

        free(sendbuf);
        TCP_wrshutdown(ACCEPT_SOCK);
        TCP_close(ACCEPT_SOCK);
    }

fail_rt:
    TCP_close(ACCEPT_SOCK);
    TCP_close(LISTEN_SOCK);

fail:
    unmap_devices();
    return -1;   
}
