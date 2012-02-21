h22194
s 00009/00000/00000
d D 1.1 94/04/20 14:51:53 dsp 1 0
c date and time created 94/04/20 14:51:53 by dsp
e
u
U
f e 0
t
T
I 1
all: board_test.cld
	run56sim -t board_test.cld

board_test.cld: board_test.cln g721.cln
	g56k board_test.cln g721.cln -o board_test.cld

%.cln: %.c
	g56k -D__G56K__ -alo -c $*.c

E 1
