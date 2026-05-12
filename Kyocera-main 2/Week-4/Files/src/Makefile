# Makefile for kyocera_pmd_init (WEEK 4 / Task 3 skeleton)
# Target: NXP LX2160A, LSDK 2004 DPDK 19.11-qoriq (static libs under /usr/local/lib).

APP  := kyocera_pmd_init
SRCS := main.c

DPDK_INC := /usr/local/include/dpdk
DPDK_LIB := /usr/local/lib

CFLAGS  += -O3 -g -Wall -Wextra -Wno-unused-parameter -D_GNU_SOURCE
CFLAGS  += -I$(DPDK_INC)

LDFLAGS += -L$(DPDK_LIB)

# PMDs that MUST be --whole-archive (constructor-registered)
PMD_LIBS := \
  -Wl,--whole-archive \
  -lrte_pmd_dpaa2 \
  -lrte_bus_fslmc \
  -lrte_mempool_dpaa2 \
  -Wl,--no-whole-archive

# Core DPDK libs (ordered for static-link dependency resolution)
CORE_LIBS := \
  -lrte_ethdev \
  -lrte_mbuf \
  -lrte_mempool \
  -lrte_mempool_ring \
  -lrte_ring \
  -lrte_net \
  -lrte_pci \
  -lrte_bus_pci \
  -lrte_bus_vdev \
  -lrte_kvargs \
  -lrte_cmdline \
  -lrte_timer \
  -lrte_hash \
  -lrte_common_dpaax \
  -lrte_eal

SYS_LIBS := -lpthread -ldl -lnuma -lrt -lm

LDFLAGS += $(PMD_LIBS) $(CORE_LIBS) $(SYS_LIBS)

.PHONY: all clean
all: $(APP)

$(APP): $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f $(APP) *.o
