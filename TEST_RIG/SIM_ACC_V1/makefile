CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra
all: test_sim
test_sim: test_sim.c nav.c nav.h
	$(CC) $(CFLAGS) -o $@ test_sim.c nav.c -lm
clean:
	rm -f test_sim
