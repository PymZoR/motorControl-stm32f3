target remote | openocd -f board/st_nucleo_f3.cfg -f gdb-pipe.cfg
monitor halt
monitor gdb_sync
stepi
