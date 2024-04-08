## clone
`git clone --recursive https://github.com/capstone-team-2023844-fpga-raytracing/NewHPS.git`
## compile
- userspace: 
  - `gcc main.c raytrace.c ext/IO/io.c -I ext/IO -O3 -std=gnu99 -lm`
  - CPU-only: add `-DCPU_TEST` to command
- kernel: has its own makefile
