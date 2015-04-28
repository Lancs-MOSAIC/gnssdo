CFLAGS = -O -Wall
LDFLAGS = -lrt -lm -lpthread -lgps
OBJS = gnssdo.o gpsdthread.o

gnssdo: $(OBJS)

gnssdo.o: gpsdthread.h
gpsdthread.o: gpsdthread.h


