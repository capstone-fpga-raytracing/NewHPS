#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Constants */
#define RESOLUTION_X 320
#define X_BOUND 319
#define RESOLUTION_Y 240
#define Y_BOUND 239

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "arm_memmap.h"

/* Prototypes for functions used to access physical memory addresses */
int open_physical(int);
void *map_physical(int, unsigned int, unsigned int);
void close_physical(int);
int unmap_physical(void *, unsigned int);

int main(void)
{
   // volatile int * FPGA_SDRAM_BASE = (int *) SDRAM_BASE;
   volatile int *FPGA_SDRAM_BASE_p = (int *)SDRAM_BASE; // virtual address pointer to FPGA SDRAM

   int fd = -1;         // used to open /dev/mem for access to physical addresses
   void *SDRAM_VIRTUAL; // used to map physical addresses for the on-chip SDRAM
   void *LW_VIRTUAL;

   if ((fd = open_physical(fd)) == -1)
      return (-1);
   if ((SDRAM_VIRTUAL = map_physical(fd, SDRAM_BASE, SDRAM_SPAN)) == NULL)
      return (-1);
   if ((fd = open_physical(fd)) == -1)
      return (-1);
   if ((LW_VIRTUAL = map_physical(fd, LW_BRIDGE_BASE, LW_BRIDGE_SPAN)) == NULL)
      return (-1);

   printf("About to write to FPGA SDRAM\n");
   FPGA_SDRAM_BASE_p = (unsigned int *)(SDRAM_VIRTUAL);
   // Write to FPGA sdram
   FPGA_SDRAM_BASE_p[0] = 0x12f62211;
   FPGA_SDRAM_BASE_p[1] = 0x34563312;
   FPGA_SDRAM_BASE_p[2] = 0x90194411;
   FPGA_SDRAM_BASE_p[3] = 0x67255511;
   FPGA_SDRAM_BASE_p[4] = 0x6611;
   FPGA_SDRAM_BASE_p[5] = 0x7711;
   FPGA_SDRAM_BASE_p[6] = 0x8811;
   FPGA_SDRAM_BASE_p[7] = 0x9911;
   FPGA_SDRAM_BASE_p[8] = 0xaa11;
   FPGA_SDRAM_BASE_p[9] = 0xbb11;
   FPGA_SDRAM_BASE_p[10] = 0xcc11;
   FPGA_SDRAM_BASE_p[11] = 0xdd11;
   FPGA_SDRAM_BASE_p[12] = 0xee11;
   FPGA_SDRAM_BASE_p[13] = 0xff11;
   FPGA_SDRAM_BASE_p[14] = 0x1100;

   printf("About to trigger FPGA read\n");
   volatile int *tFPGA_SDRAM_BASE = (int *)(LW_VIRTUAL + 0x3050); // Set to point to read trigger
   *tFPGA_SDRAM_BASE = 1;                                         // trigger FPGA read

   printf("After trigger\n");
   unmap_physical(SDRAM_VIRTUAL, SDRAM_SPAN);  // release the physical-memory mapping
   close_physical(fd);                         // close /dev/mem
   unmap_physical(LW_VIRTUAL, LW_BRIDGE_SPAN); // release the physical-memory mapping
   close_physical(fd);                         // close /dev/mem

   //  // Read from FPGA write
   //   volatile int ncnt = 0;
   //   while(ncnt != 2000000) ncnt++;

   //   printf("start reading written values\n");
   //   volatile int* data = (int*)(SDRAM_BASE + 32);
   //   for(int i = 0; i < 32; ++i)
   //   {
   //      printf("0x%08x\n", data[i]);
   //   }
}

// Open /dev/mem, if not already done, to give access to physical addresses
int open_physical(int fd)
{
   if (fd == -1)
      if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
      {
         printf("ERROR: could not open \"/dev/mem\"...\n");
         return (-1);
      }
   return fd;
}

// Close /dev/mem to give access to physical addresses
void close_physical(int fd)
{
   close(fd);
}

/*
 * Establish a virtual address mapping for the physical addresses starting at base, and
 * extending by span bytes.
 */
void *map_physical(int fd, unsigned int base, unsigned int span)
{
   void *virtual_base;

   // Get a mapping from physical addresses to virtual addresses
   virtual_base = mmap(NULL, span, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, base);
   if (virtual_base == MAP_FAILED)
   {
      printf("ERROR: mmap() failed...\n");
      close(fd);
      return (NULL);
   }
   return virtual_base;
}

/*
 * Close the previously-opened virtual address mapping
 */
int unmap_physical(void *virtual_base, unsigned int span)
{
   if (munmap(virtual_base, span) != 0)
   {
      printf("ERROR: munmap() failed...\n");
      return (-1);
   }
   return 0;
}
