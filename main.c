
#include <stdlib.h>
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


#define LISTEN_PORT "50000"
// 'SCEN' in ascii, used for endianness check
#define SCENE_MAGIC 0x5343454E
#define RTINTR_SYSFS "/sys/bus/platform/drivers/fpga_rtintr/fpga_rtintr"

#define PERRORF(fmt, ...) \
    fprintf(stderr, fmt ": %s\n", __VA_ARGS__, strerror(errno))


static int MEMFD = -1; // fd to dev/mem
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

volatile sig_atomic_t sigint = 0;

// strlen may not be signal safe in our linux version
#define SIGINT_MSG "Received ctrl+c, quitting...\n"
#define SIGINT_MSGLEN (sizeof(SIGINT_MSG)-1)

void sigint_handler(int signum) 
{
    (void)signum;
    if (sigint == 0) {
        // fprintf is not signal-safe
        ssize_t e = write(STDERR_FILENO, SIGINT_MSG, SIGINT_MSGLEN);
        (void)e; // gcc Wunused-result

        TCP_close(ACCEPT_SOCK);
        TCP_close(LISTEN_SOCK);
        ACCEPT_SOCK = INV_SOCKET;
        LISTEN_SOCK = INV_SOCKET;
        
        sigint = 1;
    }
}

int main(int argc, char** argv)
{
    bool verbose = false;
    if (argc == 2 && (strcmp(argv[1], "-v") == 0 || strcmp(argv[1], "--verbose") == 0)) {
        verbose = true;
    }

    MEMFD = open("/dev/mem", (O_RDWR | O_SYNC));
    if (MEMFD == -1) {
        fprintf(stderr, "failed to open /dev/mem\n");
        return -1;
    }

    void* vsdram = mmap_dev(SDRAM_BASE, SDRAM_SPAN);
    if (!vsdram) { return -1; }
    void* lwbase = mmap_dev(LW_BRIDGE_BASE, LW_BRIDGE_SPAN);
    if (!lwbase) {
        munmap_dev(vsdram, SDRAM_SPAN);
        return -1;
    }

    volatile unsigned* sdram = (unsigned*)(vsdram);
    volatile uint8_t* rtdev = (uint8_t*)(lwbase + RAYTRACE_BASEOFF);

    // Install ctrl+c handler to allow server to gracefully quit
    // (otherwise TCP port can be broken until OS restart)
    struct sigaction act;
    act.sa_flags = 0;
    act.sa_handler = sigint_handler;
    if (sigaction(SIGINT, &act, NULL) != 0) {
        fprintf(stderr, "failed to set sigint handler");
        goto fail;
    }

#define QUIT_IF_SIGINT if (sigint == 1) { goto fail; }

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
        unsigned nbytes_img = data[1] * data[2] * 3;
        
        // Copy data to SDRAM.
        // this is slow. there are faster ways of doing this:
        // https://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HPS_peripherials/FPGA_addr_index.html
        nrecv /= 4;
        for (int i = 1; i < nrecv; ++i) {
            sdram[i] = data[i];
        }
        free(recvbuf);
        recvbuf = NULL;

        QUIT_IF_SIGINT

        printf("Start raytracing...\n");
        *rtdev = 1;

        // Wait for interrupt from FPGA
        int intrfd = open(RTINTR_SYSFS, O_RDONLY);
        if (intrfd == -1) {
            perror("intr sysfs open failed\n");
            goto fail_rt;
        }
        uint8_t rtstat;
        if (read(intrfd, &rtstat, 1) == -1) 
        {
            perror("intr sysfs read failed\n");
            close(intrfd);
            goto fail_rt;
            // todo: read may also fail due to Ctrl+c. In this
            // case we probably want to reset the FPGA
        }
        close(intrfd);
        
        printf("Finished raytracing, status 0x%02x\n", rtstat);

        QUIT_IF_SIGINT
       
        unsigned ncopy = nbytes_img / 4;
        // align to 32-bit, unaligned reads from SDRAM are not supported
        if ((nbytes_img % 4) != 0) { 
            ncopy++;
        }

        unsigned* sendbuf = (unsigned*)malloc(ncopy);
        for (unsigned i = 0; i < ncopy; ++i) {
            sendbuf[i] = sdram[i];
        }

        if (TCP_send2(ACCEPT_SOCK, (char*)sendbuf, nbytes_img, verbose) != nbytes_img) {
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
    munmap_dev(vsdram, SDRAM_SPAN);
    munmap_dev(lwbase, LW_BRIDGE_SPAN);
    close(MEMFD);

    return -1;   
}
