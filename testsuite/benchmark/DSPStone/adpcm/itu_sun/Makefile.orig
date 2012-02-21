# Makefile for CCITT subroutines and sample programs
#

# Modify the CC definition to invoke your ANSI-C compiler
#
CC = acc


# Rules to build sample programs do not depend on a UNIX library program
#
ALL = encode decode
all: $(ALL)

OBJECTS = g711.o g72x.o g721.o g723_24.o g723_40.o

encode: $(OBJECTS)
	$(CC) -o $@ encode.c $(OBJECTS)

decode: $(OBJECTS)
	$(CC) -o $@ decode.c $(OBJECTS)


# Library rules for UNIX systems
#
LIB = ccitt.a
lib: $(LIB)

$(LIB): $(OBJECTS)
	@rm -f $(LIB)
	ar cr $(LIB) $(OBJECTS)
	@ranlib $(LIB)

clean:
	/bin/rm -f $(LIB) $(ALL) *.o
