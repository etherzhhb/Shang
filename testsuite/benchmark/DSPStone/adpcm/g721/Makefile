CC = CC

all: itu_encoder itu_decoder
	
itu_encoder : itu_encoder.o g721.o
	$(CC) itu_encoder.o g721.o -o itu_encoder
	# u-LAW sequences
	# Testing normal input...
	itu_encoder -u test_seq/u_LAW_reset/nrm.m rn32fm.i
	cmp rn32fm.i test_seq/u_LAW_reset/rn32fm.i
	# Testing overload input...
	itu_encoder -u test_seq/u_LAW_reset/ovr.m rv32fm.i
	cmp rv32fm.i test_seq/u_LAW_reset/rv32fm.i
	# A-LAW sequences
	# Testing normal input...
	itu_encoder -a test_seq/A_LAW_reset/nrm.a rn32fa.i
	cmp rn32fa.i test_seq/A_LAW_reset/rn32fa.i
	# Testing overload input...
	itu_encoder -a test_seq/A_LAW_reset/ovr.a rv32fa.i
	cmp rv32fa.i test_seq/A_LAW_reset/rv32fa.i
	# *** Encoder passed all tests!

itu_decoder : itu_decoder.o g721.o
	$(CC) itu_decoder.o g721.o -o itu_decoder
	# u-LAW sequences
	# Testing normal input...
	itu_decoder -u test_seq/u_LAW_reset/rn32fm.i rn32fm.o
	cmp rn32fm.o test_seq/u_LAW_reset/rn32fm.o
	# Testing overload input...
	itu_decoder -u test_seq/u_LAW_reset/rv32fm.i rv32fm.o
	cmp rv32fm.o test_seq/u_LAW_reset/rv32fm.o
	# Testing i-input...
	itu_decoder -u test_seq/u_LAW_reset/i32 ri32fm.o
	cmp ri32fm.o test_seq/u_LAW_reset/ri32fm.o
	# A-LAW sequences
	# Testing normal inputs
	itu_decoder -a test_seq/A_LAW_reset/rn32fa.i rn32fa.o
	cmp rn32fa.o test_seq/A_LAW_reset/rn32fa.o
	# Testing overload inputs
	itu_decoder -a test_seq/A_LAW_reset/rv32fa.i rv32fa.o
	cmp rv32fa.o test_seq/A_LAW_reset/rv32fa.o
	# Testing i-inputs
	itu_decoder -a test_seq/A_LAW_reset/i32 ri32fa.o
	cmp ri32fa.o test_seq/A_LAW_reset/ri32fa.o
	# *** Decoder passed all tests!

%.o : %.c
	$(CC) -DTEST_SEQ -c -g -Wall $*.c


