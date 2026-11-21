CC ?= gcc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra

bwb_pid_sim: BWB_PID_SIM.c BWB_PID.c bwb_plant.h
	$(CC) $(CFLAGS) -o $@ BWB_PID_SIM.c -lm

test: bwb_pid_sim
	./bwb_pid_sim --report pid_results.txt --csv step_response.csv

clean:
	rm -f bwb_pid_sim
