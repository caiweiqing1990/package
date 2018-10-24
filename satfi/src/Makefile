SRCS=$(wildcard *.c)
OBJS=$(patsubst %.c, %.o, $(SRCS))
#LDFLAGS+=-lpthread -lasound 
LDFLAGS+= -lpthread -liconv
CFLAGS+=-g 

all:satfi httpdown power_mode

satfi:satfi.o timer.o config.o log.o sequence.o led_control.o
	$(CC) -o $@ $^ $(LDFLAGS)

httpdown:httpdown.o
	$(CC) -o $@ $< $(LDFLAGS)
	
power_mode:power_mode.o
	$(CC) -o $@ $< $(LDFLAGS)
	
testvad:testvad.o
	$(CC) -o $@ $< $(LDFLAGS) -lspeex -lspeexdsp

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^
	
.PHONY: clean
clean:
	rm *.o satfi getip
