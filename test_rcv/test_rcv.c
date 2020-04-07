
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"  
#else
#include "../API/epuck/epuckAPI.h"
#endif

//#define MAX_PROX 200

void robot_setup() {
    init_robot();
    init_communication();
}

void robot_loop() {
    
    
    char rcv[12];
    int id = get_robot_ID();

    while (robot_go_on()) {

        // fetch proximity sensor values 
        
        receive_msg(rcv);
        printf("robot %d receives %s\n", id, rcv);
        
        
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




