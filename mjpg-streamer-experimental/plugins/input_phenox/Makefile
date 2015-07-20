###############################################################
#
# Purpose: Makefile for "M-JPEG Streamer"
# Author.: http://github.com/shigidono
# Version: 0.0
# License: GPL
#
# Dependencies: phenox shared library, libjpeg-turbo >= 1.3
#
###############################################################

SRCS = $(shell find . -name "*.c")

OBJS = $(SRCS:.c=.o)

CC = gcc

TARGET = input_phenox.so


OTHER_HEADERS = ../../mjpg_streamer.h ../../utils.h ../output.h ../input.h

CFLAGS += -O2 -DLINUX -D_GNU_SOURCE -Wall -shared -fPIC -g -I../../ -I/opt/libjpeg-turbo/include -pg -I../ -I/root/phenox/library/headers/ -lm `pkg-config --cflags opencv` 

LIBRARY_PATH += -lpthread -ldl /root/phenox/library/sobjs/pxlib.so `pkg-config --libs opencv` -L/opt/libjpeg-turbo/lib -l :libturbojpeg.so

clean:
	rm -f *.o core *~ *.so *.lo

all:$(OBJS)
	$(CC) -o $(TARGET) $^ $(LIBRARY_PATH) $(CFLAGS)

%.o:%.c
	$(CC) $(CFLAGS) -c $< -o $@


