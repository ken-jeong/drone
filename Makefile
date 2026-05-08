CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -g -MMD -MP -pthread -Icommon -Iserver -Iclient
LDFLAGS = -pthread -lm -lncurses

COMMON_OBJ = common/protocol.o common/log.o common/event_bus.o
SERVER_OBJ = server/server_main.o server/drone_table.o server/mission.o \
             server/ui.o $(COMMON_OBJ)
CLIENT_OBJ = client/drone_main.o client/drone_ui.o $(COMMON_OBJ)

ALL_OBJ = $(SERVER_OBJ) $(CLIENT_OBJ)
DEPS    = $(ALL_OBJ:.o=.d)

all: bin/server bin/drone

bin:
	mkdir -p bin

bin/server: $(SERVER_OBJ) | bin
	$(CC) -o $@ $(SERVER_OBJ) $(LDFLAGS)

bin/drone: $(CLIENT_OBJ) | bin
	$(CC) -o $@ $(CLIENT_OBJ) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -rf bin $(ALL_OBJ) $(DEPS)

-include $(DEPS)

.PHONY: all clean
