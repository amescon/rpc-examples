CC = gcc

SRCS = $(wildcard *.h) $(wildcard *.c)

all: rs485_write
rs485_write:  $(SRCS)
	$(CC) $^ -o $@
clean:
	rm -f rs485_write