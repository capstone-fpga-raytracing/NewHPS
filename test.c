
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>

#include "arm_memmap.h"

#define PERRORF(fmt, ...) \
    fprintf(stderr, fmt ": %s\n", __VA_ARGS__, strerror(errno))

// fd to dev/mem.
static int MEMFD = -1;

// Map pointer to a memory-mapped device.
void* mmap_dev(unsigned int base, unsigned int span)
{
    void* virt = mmap(NULL, span, (PROT_READ | PROT_WRITE), MAP_SHARED, MEMFD, base);
    if (virt == MAP_FAILED) {
        PERRORF("mmap failed for 0x%p", base);
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

int main(void)
{
   MEMFD = open("/dev/mem", (O_RDWR | O_SYNC));
   if (MEMFD == -1) {
       printf("ERROR: could not open /dev/mem\n");
       return -1;
   }

   void* sdram = mmap_dev(SDRAM_BASE, SDRAM_SPAN);
   if (!sdram) { return -1; }
   void* lwbase = mmap_dev(LW_BRIDGE_BASE, LW_BRIDGE_SPAN);
   if (!lwbase) {
       munmap_dev(sdram, SDRAM_SPAN);
       return -1;
   }

   volatile unsigned* psdram = (unsigned*)(sdram);

   printf("Writing to FPGA SDRAM\n");
   // Write to FPGA sdram
   psdram[0] = 0x12f62211;
   psdram[1] = 0x34563312;
   psdram[2] = 0x90194411;
   psdram[3] = 0x67255511;
   psdram[4] = 0x6611;
   psdram[5] = 0x7711;
   psdram[6] = 0x8811;
   psdram[7] = 0x9911;
   psdram[8] = 0xaa11;
   psdram[9] = 0xbb11;
   psdram[10] = 0xcc11;
   psdram[11] = 0xdd11;
   psdram[12] = 0xee11;
   psdram[13] = 0xff11;
   psdram[14] = 0x1100;

   printf("Triggering FPGA read\n");
   volatile unsigned *prt_trigger = (unsigned *)(lwbase + RAYTRACE_BASEOFF);
   *prt_trigger = 1;

   printf("After trigger\n");
   munmap_dev(sdram, SDRAM_SPAN);
   munmap_dev(sdram, LW_BRIDGE_SPAN);
   close(MEMFD);

   //  // Read from FPGA write
   //   volatile int ncnt = 0;
   //   while(ncnt != 2000000) ncnt++;

   //   printf("start reading written values\n");
   //   volatile int* data = (int*)(SDRAM_BASE + 32);
   //   for(int i = 0; i < 32; ++i)
   //   {
   //      printf("0x%08x\n", data[i]);
   //   }

   return 0;
}
