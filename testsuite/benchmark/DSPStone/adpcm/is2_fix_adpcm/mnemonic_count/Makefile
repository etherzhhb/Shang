CC = CC

%.exe: %.c 
	$(CC) -o $*.exe $*.c
	$*.exe
	cmp -l data.out sample.out


%.o : %.c
	$(CC) -c -g -Wall $*.c


