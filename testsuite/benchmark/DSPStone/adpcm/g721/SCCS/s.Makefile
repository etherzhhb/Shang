h65289
s 00022/00008/00042
d D 1.2 94/11/03 14:23:02 schlaege 2 1
c cross mode tests added
e
s 00050/00000/00000
d D 1.1 94/03/24 11:55:16 dsp 1 0
c date and time created 94/03/24 11:55:16 by dsp
e
u
U
f e 0
t
T
I 1
D 2
CC = CC
E 2

D 2
all: itu_encoder itu_decoder
E 2
I 2
CC = gcc

all: test

test : itu_encoder itu_decoder board_test
E 2
	
itu_encoder : itu_encoder.o g721.o
	$(CC) itu_encoder.o g721.o -o itu_encoder
D 2
	# u-LAW sequences
E 2
I 2
	# u-LAW mode
E 2
	# Testing normal input...
	itu_encoder -u test_seq/u_LAW_reset/nrm.m rn32fm.i
	cmp rn32fm.i test_seq/u_LAW_reset/rn32fm.i
	# Testing overload input...
	itu_encoder -u test_seq/u_LAW_reset/ovr.m rv32fm.i
	cmp rv32fm.i test_seq/u_LAW_reset/rv32fm.i
D 2
	# A-LAW sequences
E 2
I 2
	# A-LAW mode
E 2
	# Testing normal input...
	itu_encoder -a test_seq/A_LAW_reset/nrm.a rn32fa.i
	cmp rn32fa.i test_seq/A_LAW_reset/rn32fa.i
	# Testing overload input...
	itu_encoder -a test_seq/A_LAW_reset/ovr.a rv32fa.i
	cmp rv32fa.i test_seq/A_LAW_reset/rv32fa.i
	# *** Encoder passed all tests!

itu_decoder : itu_decoder.o g721.o
	$(CC) itu_decoder.o g721.o -o itu_decoder
D 2
	# u-LAW sequences
E 2
I 2
	# u-LAW mode
E 2
	# Testing normal input...
	itu_decoder -u test_seq/u_LAW_reset/rn32fm.i rn32fm.o
	cmp rn32fm.o test_seq/u_LAW_reset/rn32fm.o
	# Testing overload input...
	itu_decoder -u test_seq/u_LAW_reset/rv32fm.i rv32fm.o
	cmp rv32fm.o test_seq/u_LAW_reset/rv32fm.o
	# Testing i-input...
	itu_decoder -u test_seq/u_LAW_reset/i32 ri32fm.o
	cmp ri32fm.o test_seq/u_LAW_reset/ri32fm.o
D 2
	# A-LAW sequences
E 2
I 2
	# Testing cross input...
	itu_decoder -u test_seq/A_LAW_reset/rn32fa.i rn32fx.o
	cmp rn32fx.o test_seq/A_LAW_reset/rn32fx.o
	# A-LAW mode
E 2
	# Testing normal inputs
	itu_decoder -a test_seq/A_LAW_reset/rn32fa.i rn32fa.o
	cmp rn32fa.o test_seq/A_LAW_reset/rn32fa.o
	# Testing overload inputs
	itu_decoder -a test_seq/A_LAW_reset/rv32fa.i rv32fa.o
	cmp rv32fa.o test_seq/A_LAW_reset/rv32fa.o
	# Testing i-inputs
	itu_decoder -a test_seq/A_LAW_reset/i32 ri32fa.o
	cmp ri32fa.o test_seq/A_LAW_reset/ri32fa.o
I 2
	# Testing cross inputs
	itu_decoder -a test_seq/u_LAW_reset/rn32fm.i rn32fc.o
	cmp rn32fc.o test_seq/u_LAW_reset/rn32fc.o
E 2
	# *** Decoder passed all tests!

I 2
board_test: g721.o board_test.o
	$(CC) -g g721.o board_test.o -o board_test
	board_test

E 2
%.o : %.c
D 2
	$(CC) -DTEST_SEQ -c -g -Wall $*.c
E 2
I 2
	$(CC) -c -g -Wall $*.c
E 2

D 2

E 2
I 2
clean:
	rm -f *.o *32*.i *~
E 2
E 1
