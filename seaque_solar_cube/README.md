# SEAQUE QCB Firmware
This repo has the firmware for the SEAQUE Quantum Control Board, which controls a majority of experiment functions. It was originally an STM32CubeIDE project, but is now buildable with CMake via the `build.sh` script.

## How to Build
These instructions assume you are using a unix-like environment, like Linux, MacOS, or WSL. Fist make sure to install the GNU Arm Embedded Toolchain, as well as CMake. With these, the script `build.sh` should run without issue.

## How to Program/Debug
Install openocd, and in the main directory of the repo, run the command `openocd`. It should pick up the `openocd.cfg` file. There is also a `.gdbinit` file that connects to the openocd server, sets a source layout, and grabs symbols from the right file. To start debugging, simply run `arm-none-eabi-gdb`. To program the board, run the gdb command `load`.

target extended-remote :3333
lay src
focus cmd
