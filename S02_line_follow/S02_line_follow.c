#define SIMULATION 1

#if SIMULATION
    #include "../API/webots/webotsAPI.h"
#else
    #include "../API/epuck/epuckAPI.h"
#endif

void robot_setup() {
    init_robot();
    init_sensors();
}

void follower() {
    short int IR_ground[GROUND_SENSORS_COUNT];
    get_ground(IR_ground);

    double gsRight = ((10.0 * IR_ground[GS_LEFT] + 2.0 * IR_ground[GS_CENTER] + 0.0 * IR_ground[GS_RIGHT]) / (12.0));
    double gsLeft = ((0.0 * IR_ground[GS_LEFT] + 2.0 * IR_ground[GS_CENTER] + 10.0 * IR_ground[GS_RIGHT]) / (12.0));

    double dsRight = (NORM_SPEED * gsRight) / 800;
    double dsLeft = (NORM_SPEED * gsLeft) / 800;

    double speedRight = bounded_speed(NORM_SPEED - dsRight);
    double speedLeft = bounded_speed(NORM_SPEED - dsLeft);

    set_speed(speedLeft, speedRight);
}

void robot_loop() {
    while (robot_go_on()) {
        follower();
    }
    cleanup_robot();
}


int main(int argc, char **argv) {

#if SIMULATION
#else
    ip = argv[1];
#endif

    robot_setup();
    robot_loop();
}






