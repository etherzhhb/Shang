; Simulator Command File
; If you DONT WANT to use the SERIAL PORTS use the following map

ma 0,0,0x10000,ram                  ; Allocate PROGRAM MEMORY    0 - 0xffff

ma 0,1,0x80,ram                     ; Allocate DATA MEMORY       0 - 0x7f
                                    ; RESERVED DATA MEMORY     0x80 - 0xff
ma 0x100,1,0x400,ram                ; Allocate DATA MEMORY    0x100 - 0x4ff
                                    ; RESERVED DATA MEMORY    0x500 - 0x7ff
ma 0x800,1,0xf800,ram               ; Allocate DATA MEMORY    0x800 - 0xffff

; Sample Connect commands to connect Input and Output Files to the
; Serial Ports

e *0x28=0
e *0x29=0
e *0x2a=0
;e *0x7=0x3eh

;fill 0x2000,1,0x800,0xff

sconfig asm.cfg  ; load own screen configuration
wa clk


mem 0x2000
mem2 0x7e

