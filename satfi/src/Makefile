SRCS=$(wildcard *.c)
OBJS=$(patsubst %.c, %.o, $(SRCS))
#LDFLAGS+=-lpthread -lasound 
LDFLAGS+= -lpthread -liconv -lspeex -lspeexdsp
CFLAGS+=-g 

all:satfi httpdown power_mode 3g_manage broadcast_client broadcast_server

satfi:satfi.o timer.o config.o log.o sequence.o led_control.o
	$(CC) -o $@ $^ $(LDFLAGS)

httpdown:httpdown.o
	$(CC) -o $@ $<
	
power_mode:power_mode.o
	$(CC) -o $@ $< 
	
3g_manage:3g_manage.o
	$(CC) -o $@ $< $(LDFLAGS)
	
broadcast_client:broadcast_client.o
	$(CC) -o $@ $< 
	
broadcast_server:broadcast_server.o
	$(CC) -o $@ $< 

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^
	
.PHONY: clean
clean:
	rm *.o satfi getip
