SRCS=$(wildcard *.c)
OBJS=$(patsubst %.c, %.o, $(SRCS))
LDFLAGS+=-lpthread -lasound 
CFLAGS+=-g

all:satfi getip power_mode 3g_manage httpdown clientapp

satfi:satfi.o timer.o config.o log.o sequence.o led_control.o
	$(CC) $(LDFLAGS) satfi.o timer.o config.o log.o led_control.o -o satfi 

getip:getip.o
	$(CC) $(LDFLAGS) getip.o -o getip

clientapp:clientapp.o
	$(CC) $(LDFLAGS) clientapp.o -o clientapp

httpdown:httpdown.o
	$(CC) $(LDFLAGS) httpdown.o -o httpdown

3g_manage:3g_manage.o
	$(CC) $(LDFLAGS) 3g_manage.o -o 3g_manage
	
power_mode:power_mode.o
	$(CC) $(LDFLAGS) power_mode.o -o power_mode
	
satfi.o:satfi.c
	$(CC) $(CFLAGS) -c satfi.c

timer.o:timer.c
	$(CC) $(CFLAGS) -c timer.c

config.o:config.c
	$(CC) $(CFLAGS) -c config.c

log.o:log.c
	$(CC) $(CFLAGS) -c log.c

sequence.o:sequence.c
	$(CC) $(CFLAGS) -c sequence.c
	
led_control.o:led_control.c
	$(CC) $(CFLAGS) -c led_control.c
	
alsaplay.o:alsaplay.c
	$(CC) $(CFLAGS) -c alsaplay.c

alsarecord.o:alsarecord.c
	$(CC) $(CFLAGS) -c alsarecord.c	

getip.o:getip.c
	$(CC) $(CFLAGS) -c getip.c

httpdown.o:httpdown.c
	$(CC) $(CFLAGS) -c httpdown.c

clientapp.o:clientapp.c
	$(CC) $(CFLAGS) -c clientapp.c

3g_manage.o:3g_manage.c
	$(CC) $(CFLAGS) -c 3g_manage.c
	
power_mode.o:power_mode.c
	$(CC) $(CFLAGS) -c power_mode.c
	
.PHONY: clean
clean:
	rm *.o satfi getip
