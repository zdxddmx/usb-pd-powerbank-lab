CC      ?= cc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Werror -Iinc
LDFLAGS ?= -lm

SRC_COMMON = src/battery.c src/bms.c src/pd.c src/powerbank.c
OUTDIR     = build

.PHONY: all test sim clean

all: $(OUTDIR)/pb_sim $(OUTDIR)/pb_test

$(OUTDIR):
	@mkdir -p $(OUTDIR) 2>/dev/null || mkdir $(OUTDIR)

$(OUTDIR)/pb_sim: $(SRC_COMMON) src/main_sim.c inc/pb.h | $(OUTDIR)
	$(CC) $(CFLAGS) $(SRC_COMMON) src/main_sim.c -o $@ $(LDFLAGS)

$(OUTDIR)/pb_test: $(SRC_COMMON) test/test_pb.c inc/pb.h | $(OUTDIR)
	$(CC) $(CFLAGS) $(SRC_COMMON) test/test_pb.c -o $@ $(LDFLAGS)

test: $(OUTDIR)/pb_test
	./$(OUTDIR)/pb_test

sim: $(OUTDIR)/pb_sim
	@mkdir -p out 2>/dev/null || mkdir out
	./$(OUTDIR)/pb_sim discharge > out/discharge.csv
	./$(OUTDIR)/pb_sim charge    > out/charge.csv
	./$(OUTDIR)/pb_sim sleep     > out/sleep.csv
	@echo "CSV 已输出到 out/"

clean:
	rm -rf $(OUTDIR) out
