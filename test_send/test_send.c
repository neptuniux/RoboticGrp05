
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
    char tmp[12];
    int id = get_robot_ID();

    while (robot_go_on()) {

        // fetch proximity sensor values 
        
        
        sprintf(tmp, "test");
        printf("robot %d sends %s\n", id, tmp);
        send_msg(tmp);
        // basic love behavior
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




