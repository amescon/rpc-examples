CC = gcc

SRCS = $(wildcard *.h) $(wildcard *.c)

all: rs485_poll
rs485_poll:  $(SRCS)
	$(CC) $^ -o $@
clean:
	rm -f rs485_poll