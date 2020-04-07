
#define SIMULATION 0

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

void explorer(), lover();
typedef int bool;
#define true 1
#define false 0

int STATE = false;
int SAW_OBSTACLE = 0;
int a = 1; int b = 2; int c = 3; int d = 4; double e = 10.;
void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
}

void robot_loop() {
   short int prox_values[8];

    while (robot_go_on()) {

        get_prox_calibrated(prox_values);
        double prox_right = (a*prox_values[0] + b*prox_values[1] + c*prox_values[2] + d*prox_values[3]) / e;
        double prox_left = (a*prox_values[7] + b*prox_values[6] + c*prox_values[5] + d*prox_values[4]) / e;
        double ds_right = (MAX_SPEED * prox_right) / MAX_PROX;
        double ds_left = (MAX_SPEED * prox_left) / MAX_PROX;
        double speed_right = bounded_speed(MAX_SPEED - ds_right);
        double speed_left = bounded_speed(MAX_SPEED - ds_left);

        if (STATE == 0)
        {
            if(speed_right < 200 || speed_left < 200){
             SAW_OBSTACLE = 1;
             explorer(speed_left, speed_right);

           } else if ((speed_right > 380 && speed_left > 380) && SAW_OBSTACLE)
           {
               SAW_OBSTACLE = 0;
               lover(speed_left, speed_right);
           } else
           {
               explorer(speed_left, speed_right);
           }

        } else if (STATE == 1)
        {
           if (speed_right < 0.0 && speed_left < 0.0)
           {
              for (int i = 2000; i > 0; i--) {
                explorer(speed_left, speed_right);
              }

           } else
           {
           lover(speed_left, speed_right);
           }
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

void explorer(double speed_left, double speed_right){
    STATE = false;
    a = 1; b = 2; c= 2; d = 1;
    e = a + b + c + d;
    disable_led(0);
    disable_led(1);
    disable_led(2);
    disable_led(3);
    set_speed(speed_right, speed_left);
}

void lover(double speed_left, double speed_right){
    STATE = true;
    a = 5; b = 3; c = 2; d = 1;
    e = a + b + c + d;
    enable_led(0);
    enable_led(1);
    enable_led(2);
    enable_led(3);
    set_speed(speed_left, speed_right);

}
