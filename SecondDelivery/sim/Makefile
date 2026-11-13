# Simulador del seguidor (corre en la PC, no en el Zumo).
#   make            -> compila
#   make run        -> vuelta con parámetros por defecto -> vuelta.csv
W := ../ZumoWaypointFollower
sim_follower: sim_follower.c $(W)/follower.c $(W)/follower.h $(W)/waypoints.c $(W)/waypoints.h
	gcc -O2 -Wall -Wextra -I$(W) -o $@ sim_follower.c $(W)/follower.c $(W)/waypoints.c -lm
run: sim_follower
	./sim_follower > vuelta.csv
clean:
	rm -f sim_follower vuelta.csv
.PHONY: run clean
