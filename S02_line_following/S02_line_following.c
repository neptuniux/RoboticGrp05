
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

#define MAX_GROUND 300
//#define NORM_SPEED 2.0



void robot_setup() {
    init_robot();
    init_sensors();
    //calibrate_prox();
}

void robot_loop() {
   short int IR_ground[GROUND_SENSORS_COUNT];

    while (robot_go_on()) {

        get_ground(IR_ground);
        double gs_right = (1*IR_ground[GS_LEFT] + 100*IR_ground[GS_CENTER] + 500*IR_ground[GS_RIGHT]) / 601.;
        double gs_left = (500*IR_ground[GS_LEFT] + 100*IR_ground[GS_CENTER] + 1*IR_ground[GS_RIGHT]) / 601.;
        double ds_right = (NORM_SPEED * gs_right) / MAX_GROUND;
        double ds_left = (NORM_SPEED * gs_left) / MAX_GROUND;
        double speed_right = bounded_speed(MAX_SPEED - ds_left);
        double speed_left = bounded_speed(MAX_SPEED - ds_right);
        printf("left = %f; right = %f\n",ds_left, ds_right);
        set_speed(speed_left, speed_right);
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
