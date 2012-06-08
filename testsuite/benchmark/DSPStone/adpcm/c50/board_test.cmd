
MEMORY
{
    PAGE 0:
		ON_CHIP		:origin=30h, length=1fd0h
    PAGE 1:   	
		REGS		:origin=0h,length=60h
		SCRATCH_RAM	:origin=60h,length=20h    
 		INT_DRAM	:origin=100h,length=400h  
		INT_SRAM	:origin=800h,length=0400h
		EXT_RAM		:origin=0c00h,length=0f400h
}

SECTIONS
{
	.text : {} > ON_CHIP	PAGE=0
	.cinit: {} > ON_CHIP	PAGE=0
	.stack: {} > INT_DRAM   PAGE=1
	.bss  : {} > INT_DRAM   PAGE=1
 	.data : {} > EXT_RAM	PAGE=1
}

board_test.obj
-c
-stack 100
-m board_test.map
-o board_test.exe
-l /tools/home1/dsp/c50/rts50.lib
