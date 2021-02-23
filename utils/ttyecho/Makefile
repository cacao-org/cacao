CC      = gcc
CFLAGS  =
DESTDIR  = /usr/bin

.PHONY: all
all: ttyecho

ttyecho: ttyecho.c
	$(CC) $(CFLAGS) -o ttyecho ttyecho.c

.PHONY: install
install:
	mkdir -p $(DESTDIR)
	install -m 0755 ttyecho -t $(DESTDIR)

uninstall:
	rm $(DESTDIR)/ttyecho

.PHONY: clean 
clean:
	rm -f ttyecho

