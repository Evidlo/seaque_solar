file build/qcb.elf
target extended-remote 10.195.192.196:3341
lay src
load

mon reset halt
b main
c
