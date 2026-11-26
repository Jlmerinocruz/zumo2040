# Simulador del seguidor (corre en la PC, no en el Zumo).
#   make            -> compila
#   make run        -> vuelta con parámetros por defecto -> vuelta.csv
#   make test       -> prueba unitaria de goto_wp.c (ir a un waypoint)
W := ../ZumoWaypointFollower
sim_follower: sim_follower.c $(W)/follower.c $(W)/follower.h $(W)/waypoints.c $(W)/waypoints.h
	gcc -O2 -Wall -Wextra -I$(W) -o $@ sim_follower.c $(W)/follower.c $(W)/waypoints.c -lm
test_goto: test_goto.c $(W)/goto_wp.c $(W)/goto_wp.h $(W)/follower.c $(W)/follower.h $(W)/waypoints.c $(W)/waypoints.h
	gcc -O2 -Wall -Wextra -I$(W) -o $@ test_goto.c $(W)/goto_wp.c $(W)/follower.c $(W)/waypoints.c -lm
test: test_goto
	./test_goto
run: sim_follower
	./sim_follower > vuelta.csv
clean:
	rm -f sim_follower test_goto vuelta.csv
.PHONY: run test clean
