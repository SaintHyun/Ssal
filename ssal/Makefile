CC=gcc
CFLAGS=-Wall -O2
LDFLAGS=-lpigpio -lrt -lpthread

all: tempctrl

tempctrl: main.c
	$(CC) $(CFLAGS) -o tempctrl main.c $(LDFLAGS)

clean:
	rm -f tempctrl