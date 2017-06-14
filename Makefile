CC = gcc
CFLAGS = -Wall -g
OBJS = simulator.o gptcall.o
PROG = simulator.exe
all: $(PROG)
simulator.exe: $(OBJS)
	$(CC) $(OBJS) -o simulator.exe
simulator.o: simulator.c func_lib.h
gptcall.o: gptcall.c gptcall.h

clean: $(PROG) $(OBJS)
	rm $(PROG) $(OBJS)
