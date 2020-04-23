//
// Created by daniel on 23.04.20.
//


#include "header.h"

#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

void init_rgbled();


typedef int bool;
#define true 1
#define false 0

//define states
#define FIND 0
#define LINE 1
#define WALL 2

void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
    init_camera();
    init_rgbled();
}

void robot_loop() {

    int loop_counter = 0;
    int STATE = FIND;

    while (robot_go_on()) {
        loop_counter++;

        switch (STATE){
            case FIND:
                STATE = stateFind();
                break;

            case LINE:
                STATE = stateLine();
                break;

            case WALL:
                STATE = stateWall();
                break;
        }

    }
    cleanup_robot();
}

int main (int argc, char **argv) {
#if SIMULATION
#else
    ip = argv[1];
#endif

    robot_setup();
    robot_loop();
}

