APP := kyocera_pmd_init
SRCS := main.c

PKGCONF ?= pkg-config

CFLAGS  = -O3 -g -Wall -Wextra -Wno-unused-parameter -D_GNU_SOURCE -DALLOW_EXPERIMENTAL_API
CFLAGS += $(shell $(PKGCONF) --cflags libdpdk)

LDFLAGS  = $(shell $(PKGCONF) --static --libs libdpdk)
LDFLAGS += -lpthread -ldl -lnuma -lrt -lm

.PHONY: all clean

all: $(APP)

$(APP): $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)  

clean:
	rm -f $(APP) *.o
