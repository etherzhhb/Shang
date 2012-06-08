C50_DIR = /tools/home1/dsp/c50

CC     = $(C50_DIR)/dspcl
LINKER = $(C50_DIR)/dsplnk

board_test.exe : board_test.obj board_test.cmd
	$(LINKER) board_test.cmd
	sim5x board_test.exe &

msb.exe : msb.obj msb.cmd
	$(LINKER) msb.cmd
	sim5x msb.exe &

%.obj : %.c
	$(CC) $*.c -c -g -o2 -mn -k -s -as -v50 -d__TMS320C50__ 
