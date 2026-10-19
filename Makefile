# Standalone Makefile for ddc-c (pure C, no C++ required)
#
# Targets:
#   all              build all ddc-c tools
#   gen_ddc_c        bitmap generator (tools/gen_ddc_c.c)
#   bench_ops_w8_c   boolean benchmark (tools/bench_ops_w8_c.c)
#   ddc_c_selftest   selftest (test/ddc_c_selftest.c)
#   clean            remove build outputs
#
# Usage (from ddc_c/ root):
#   make
#   make gen_ddc_c
#   make clean

CC      := /usr/bin/gcc
CFLAGS  := -O3 -march=native -funroll-loops -std=gnu11 -Wall -Wextra
INC     := -I include

DDC_C_SRCS := \
    src/ddc_c_base.c \
    src/ddc_c_ops.c \
    src/ddc_c_serial.c \
    src/ddc_c_sparse.c

OUTDIR  := build

.PHONY: all clean

all: $(OUTDIR) $(OUTDIR)/gen_ddc_c $(OUTDIR)/bench_ops_w8_c $(OUTDIR)/ddc_c_selftest

$(OUTDIR):
	mkdir -p $(OUTDIR)

$(OUTDIR)/gen_ddc_c: tools/gen_ddc_c.c $(DDC_C_SRCS)
	$(CC) $(CFLAGS) $(INC) $^ -o $@

$(OUTDIR)/bench_ops_w8_c: tools/bench_ops_w8_c.c $(DDC_C_SRCS)
	$(CC) $(CFLAGS) $(INC) $^ -o $@

$(OUTDIR)/ddc_c_selftest: test/ddc_c_selftest.c $(DDC_C_SRCS)
	$(CC) $(CFLAGS) $(INC) $^ -o $@

clean:
	rm -f $(OUTDIR)/gen_ddc_c $(OUTDIR)/bench_ops_w8_c $(OUTDIR)/ddc_c_selftest
