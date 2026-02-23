# Programming

    ./build.sh && sh /home/evan/.arduino15/packages/STMicroelectronics/tools/STM32Tools/2.1.1/stm32CubeProg.sh 0 build/qcb.elf -g
	
# Debugging

program with gdb:
	load build/elf

connect with:
openocd

$ arm-none-eabi-gdb example.elf
then:
	target extended-remote localhost:3333





file build/qcb.elf
target extended-remote 10.195.192.196:3341
load



lay src
# reset and halt
mon reset halt
b main
c

b main
b [lineno]

# save to file
(gdb) set logging file large_array.txt
(gdb) set logging on
p packet_data[0]@3072


dump binary memory out.bin &packet_data[0] &packet_data[3072] 