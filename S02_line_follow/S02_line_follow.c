
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif





int State =0;

void robot_setup() {
    init_robot();
    init_sensors();
}

#define MAXSTEPS 500

void follower(){
    enable_led(0);
    enable_led(1);
    enable_led(2);
    enable_led(3);

    short int IR_ground[GROUND_SENSORS_COUNT];
    get_ground(IR_ground);

    double gsRight = ((10.0*IR_ground[GS_LEFT]+2.0*IR_ground[GS_CENTER]+0.0*IR_ground[GS_RIGHT])/(12.0));
    double gsLeft = ((0.0*IR_ground[GS_LEFT]+2.0*IR_ground[GS_CENTER]+10.0*IR_ground[GS_RIGHT])/(12.0));

    double dsRight = (NORM_SPEED * gsRight) / 800;
    double dsLeft = (NORM_SPEED * gsLeft) / 800;

    double speedRight = bounded_speed(NORM_SPEED - dsRight);
    double speedLeft = bounded_speed(NORM_SPEED - dsLeft);

    set_speed(speedLeft, speedRight);

}

void explorer(){
    disable_led(0);
    disable_led(1);
    disable_led(2);
    disable_led(3);
    short int prox_values[8];
    get_prox_calibrated(prox_values);

    double proxRight = ((4.0*prox_values[0]+3.0*prox_values[1]+2.0*prox_values[2]+1.0*prox_values[3])/(10.0));
    double proxLeft = ((4.0*prox_values[7]+3.0*prox_values[6]+2.0*prox_values[5]+1.0*prox_values[4])/(10.0));

    double dsRight = (MAX_SPEED * proxRight) / MAX_PROX;
    double dsLeft = (MAX_SPEED * proxLeft) / MAX_PROX;
    double speedRight = bounded_speed(MAX_SPEED - dsRight);
    double speedLeft = bounded_speed(MAX_SPEED - dsLeft);
    set_speed(speedRight,speedLeft);

    if ((speedLeft >= MAX_SPEED/1.5) && (speedRight>=MAX_SPEED/1.5)){
        State=1;
    }
}

void robot_loop() {
    do{
        follower();
    } while (robot_go_on());
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






