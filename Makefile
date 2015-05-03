CFLAGS = -O -Wall
LDFLAGS = -lrt -lm -lpthread -lgps
OBJS = gnssdo.o gpsdthread.o

all: gnssdo dtoverlay

gnssdo: $(OBJS)

gnssdo.o: gpsdthread.h
gpsdthread.o: gpsdthread.h

dtoverlay: gps-pps-00A0.dtbo

# Device tree overlays
%.dtbo: %.dts
	dtc -O dtb -o $@ -b 0 -@ $<

