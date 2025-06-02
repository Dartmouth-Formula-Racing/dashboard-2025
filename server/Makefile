CC = gcc
CFLAGS = -Wall -O2
LIBS = -lpigpio -lrt -pthread -lwebsockets -lm

all: dashboard

dashboard: canbus.o websocket_server.o
	$(CC) $(CFLAGS) -o dashboard canbus.o websocket_server.o $(LIBS)

canbus.o: canbus.c canbus.h
	$(CC) $(CFLAGS) -c canbus.c

websocket_server.o: websocket_server.c canbus.h
	$(CC) $(CFLAGS) -c websocket_server.c

clean:
	rm -f *.o dashboard
