CC = gcc

SRCS = $(wildcard *.h) $(wildcard *.c)

all: rs485_read
rs485_read:  $(SRCS)
	$(CC) $^ -o $@
clean:
	rm -f rs485_read