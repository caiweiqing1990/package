# build executable on typing make
all: pcm_test

%.o: %.c
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) -c -I. -Iinclude -o $@ $^

pcm_test: pcm_test.o
	$(CC) -g -o $@ $^ -L. -lpthread

clean:
	rm -f *.o pcm_test
