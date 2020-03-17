
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"  
#else

#include <jmorecfg.h>
#include "../API/epuck/epuckAPI.h"
#endif

int State = 1;
int saw=0;

void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
}

void follower(){
    enable_led(0);
    enable_led(1);
    enable_led(2);
    enable_led(3);
    double a=4,b=3,c=2,d=1;
    short int prox_values[8];
    get_prox_calibrated(prox_values);

    double proxRight = ((4.0*prox_values[0]+3.0*prox_values[1]+2.0*prox_values[2]+1.0*prox_values[3])/(10.0));
    double proxLeft = ((4.0*prox_values[7]+3.0*prox_values[6]+2.0*prox_values[5]+1.0*prox_values[4])/(a+b+c+d));

    double dsRight = (MAX_SPEED * proxRight) / MAX_PROX;
    double dsLeft = (MAX_SPEED * proxLeft) / MAX_PROX;
    double speedRight = bounded_speed(MAX_SPEED - dsRight);
    double speedLeft = bounded_speed(MAX_SPEED - dsLeft);

    set_speed(speedLeft, speedRight);
    if ((prox_values[0]+prox_values[7])/2. >= 900){
        set_speed(0,0);
        State = 1;
    }

}

void explorer(){
    disable_led(0);
    disable_led(1);
    disable_led(2);
    disable_led(3);
    //play_sound(1);
    double a=4,b=3,c=2,d=1;
    short int prox_values[8];
    get_prox_calibrated(prox_values);

    double proxRight = ((a*prox_values[0]+b*prox_values[1]+c*prox_values[2]+d*prox_values[3])/(a+b+c+d));
    double proxLeft = ((a*prox_values[7]+b*prox_values[6]+c*prox_values[5]+d*prox_values[4])/(a+b+c+d));

    double dsRight = (MAX_SPEED * proxRight) / MAX_PROX;
    double dsLeft = (MAX_SPEED * proxLeft) / MAX_PROX;
    double speedRight = bounded_speed(MAX_SPEED - dsRight);
    double speedLeft = bounded_speed(MAX_SPEED - dsLeft);

    set_speed(speedRight,speedLeft);
    if ((prox_values[0]+prox_values[7])/2. >= 100){
       // set_speed(0,0);

        saw = 1;
    }

    if (saw=1 && speedLeft >= MAX_SPEED && speedRight>=MAX_SPEED){
        State=0;
    }
}


void robot_loop() {


    while (robot_go_on()) {
        if (State == 1){
            explorer();
        } else{
            lover();
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




