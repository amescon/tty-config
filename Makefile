# Name of the output file
NAME = tty-config

SRCS = $(wildcard *.h) $(wildcard *.c)

all: tty-config

tty-config: $(SRCS)
	$(CC)  $^ -o $@

clean:
	rm -f tty-config
