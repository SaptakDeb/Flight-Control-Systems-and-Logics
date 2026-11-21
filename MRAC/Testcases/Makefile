CC ?= gcc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra

bwb_mrac_sim: BWB_MRAC_SIM.c BWB_MRAC.c BWB_PID.c bwb_plant.h
	$(CC) $(CFLAGS) -o $@ BWB_MRAC_SIM.c -lm

test: bwb_mrac_sim
	./bwb_mrac_sim --report mrac_results.txt --csv failure.csv

clean:
	rm -f bwb_mrac_sim
