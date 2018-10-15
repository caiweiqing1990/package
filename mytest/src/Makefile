# build executable on typing make
all: mytest msgtest

%.o: %.c
	$(CC) $(CFLAGS) -c -I. -Iinclude -o $@ $^

mytest: mytest.o
	$(CC) -o $@ $^ -lpthread -liconv
	
msgtest: msgtest.o
	$(CC) -g -o $@ $^ -L. -liconv

clean:
	rm -f *.o $(all)
