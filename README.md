# HPS (Hard Processor System)

The DE1-SoC board has a built-in ARM processor, which we use to simplify communication between the PC and the FPGA core.   
Intel provides a custom Linux image for this processor to make it easier to access on-board hardware (like Ethernet).

The code here targets this version of Linux. It consists of 3 parts:
- a barebones TCP server so the PC can send data to the DE1-SoC easily
- a Linux driver for our core
- the shading algorithm (we ran out of time to move this to the FPGA core) 

Clone this repo using `git clone --recursive https://github.com/capstone-team-2023844-fpga-raytracing/NewHPS.git`. Note the `--recursive`.

## Instructions
- Setup the NewFPGA repo first! It contains instructions on compiling the FPGA core and booting DE1-SoC Linux.
- Once done, the NewHPS repo should be on the DE1. Perform the following steps on the DE1 Linux remote shell:
  - server + shader: `gcc main.c raytrace.c ext/IO/io.c -I ext/IO -O3 -std=gnu99 -lm -lrt`. This produces file `a.out`.
  - kernel driver:
     - `cd` to the kernel/ directory and type `make` to build. This produces a .ko driver file. If you see errors, trying setting the date and time first eg. `date -s '2023-12-25 12:34:56'`.
     - Use the fpga_program script (in the NewFPGA repo) to program the FPGA and install the driver. eg. usage `./fpga_program.sh compiled_core.rbf`
       - (the script uses a hard-coded path to access the .ko file, modify this if necessary before running).
     - If programming is successful, the message 'dEAd' should show up on the FPGA hex display. 
- You can now start the TCP server: `./a.out --verbose`
- If the server starts successfully, you should see the message: `Waiting for connection from client...`
- You may now use our [command line tool](https://github.com/capstone-fpga-raytracing/host) to start a render from your PC. See the host repo for instructions on building the tool.
