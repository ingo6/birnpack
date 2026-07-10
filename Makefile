CC      ?= cc
CFLAGS  ?= -O3 -pthread
LDLIBS   = -lm

birnpack: src/welle_fast.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

test: birnpack
	bash tests/roundtrip.sh

clean:
	rm -f birnpack

.PHONY: test clean
