# HPS (Hard Processor System)

The DE1SoC board has a built-in ARM a9 processor, which we use as a transit point between the host (laptop/PC) and the FPGA core.  

This repo contains code for that processor.
It consists of three parts:
- a barebones TCP server so the host can send data to the FPGA easily
- a Linux driver for our core
- the shader (we ran out of time to move this to the FPGA core)

## Instructions
- Setup the NewFPGA repo first. This contains instructions on compiling the FPGA core.
- Clone this repo: `git clone --recursive https://github.com/capstone-team-2023844-fpga-raytracing/NewHPS.git`
- Once the FPGA is programmed, compile everything in this repo:
  - kernel driver:
     - `cd` to the kernel/ directory and type `make` to build. This produces a .ko file. If you see strange errors, trying setting the date and time first eg. `date -s '2023-12-25 12:34:56'`.
     - Use the fpga_program.sh script to program the FPGA and install the kernel driver (the script uses a hard-coded path to access the compiled kernel module, please change this).
     - If the programming is successful, the message 'dEAd' sbould show up on the FPGA hex display. 
  - server + shader: `gcc main.c raytrace.c ext/IO/io.c -I ext/IO -O3 -std=gnu99 -lm -lrt`.
- The last compile should produce the file `a.out`. You can now start the TCP server: `./a.out --verbose`
- If the server starts successfully, you should see the message: `Waiting for connection from client...`
- You may now use our [command line tool](https://github.com/capstone-fpga-raytracing/host) to start a render from your laptop/PC. See the host repo for instructions on setting up the tool.
