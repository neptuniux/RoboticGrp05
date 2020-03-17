
#define SIMULATION 0

#if SIMULATION
#include "../API/webots/webotsAPI.h"  
#else
#include "../API/epuck/epuckAPI.h"
#endif

void robot_setup() {
    init_robot();
    init_sensors();
    //calibrate_prox();
}

void robot_loop() {

    int counter = 0;
    while (robot_go_on()) {
      double left = (++counter % 100) > 50 ? -0.5 : 0.5; 
      double right = (++counter % 100) > 50 ? 1 : -1; 
      //printf("%f\n",dir*NORM_SPEED);
      set_speed(bounded_speed(left*NORM_SPEED), bounded_speed(NORM_SPEED*right));
      
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




