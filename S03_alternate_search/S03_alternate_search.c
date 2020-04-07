//
// Created by daniel on 02.04.20.
//

#define SIMULATION 1

#if SIMULATION

#include <stdbool.h>
#include <unistd.h>
#include "../API/webots/webotsAPI.h"

#else
#include "../API/epuck/epuckAPI.h"
#endif

void explorer(), lover(), fsm(), sendUpdateStates(),sendCurrentStatus();
int receiveOtherStatus();
void receiveUpdateState();

#define STEXPLORER 0
#define STLOVER 1
#define STEQUIL 2


#define NBRROBOT 2

int STATE = STEXPLORER;
int SAW_OBSTACLE = 0;
int equilibirum = 0;

int a = 1;

int b = 2;
int c = 3;
int d = 4;
double e = 10.;


void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
}

int states[NBRROBOT][2];


void robot_loop() {


    short int prox_values[8];

    while (robot_go_on()) {
        int id = get_robot_ID();

        printf("robot nr %d , State=%d , Obstacle=%d \n", id, states[id-1][0],states[id-1][1]);

        get_prox_calibrated(prox_values);
        double prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / e;
        double prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / e;
        double ds_right = (MAX_SPEED * prox_right) / MAX_PROX;
        double ds_left = (MAX_SPEED * prox_left) / MAX_PROX;
        double speed_right = bounded_speed(MAX_SPEED - ds_right);
        double speed_left = bounded_speed(MAX_SPEED - ds_left);

        int oldState = states[id -1][0];
        receiveUpdateState(id);
        int newState = states[id -1][0];

        if(oldState==2 && newState ==0){
            for (int i = 2000; i > 0; i--) {
                explorer(speed_left, speed_right);
            }
        }
        fsm(id,speed_left,speed_right);

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

void fsm(int robotNumber, double speed_left, double speed_right) {
    int currentState = states[robotNumber - 1][0];
    int currentSaw = states[robotNumber - 1][1];

    switch (currentState){

        case STEXPLORER:
            if (speed_right < 0.75 || speed_left < 0.75) {
                //the robot saw an obstacle
                states[robotNumber - 1][1] = 1;
                //explorer(robotNumber,speed_left, speed_right);
            } else if ((speed_right > 3.2 && speed_left > 3.2) && currentSaw) {
                //the robot is far form the obstacle
                states[robotNumber - 1][1] = 0;
                lover(robotNumber,speed_left, speed_right);
            } else {
                explorer(robotNumber,speed_left, speed_right);
            }
            break;

        case STLOVER:
            if (speed_right < 0.0 && speed_left < 0.0) {
                states[robotNumber - 1][0] = STEQUIL;
                sendUpdateStates(robotNumber);
            } else {
                lover(robotNumber,speed_left, speed_right);
            }
            break;

        case STEQUIL:
            set_speed(0,0);
            sendCurrentStatus(robotNumber);
            break;
    }


}

void explorer(int robotNumber, double speed_left, double speed_right) {
    states[robotNumber - 1][0] = STEXPLORER;
    a = 1;
    b = 2;
    c = 2;
    d = 1;
    e = a + b + c + d;
    disable_led(0);
    disable_led(1);
    disable_led(2);
    disable_led(3);
    set_speed(speed_right, speed_left);
}

void lover(int robotNumber, double speed_left, double speed_right) {
    states[robotNumber - 1][0] = STLOVER;
    a = 5;
    b = 3;
    c = 2;
    d = 1;
    e = a + b + c + d;
    enable_led(0);
    enable_led(1);
    enable_led(2);
    enable_led(3);
    set_speed(speed_left, speed_right);

}

void sendUpdateStates(int robotID){
    char tmp[5];
    int newVal;
    init_communication();

    int otherStatus = receiveOtherStatus();
    if (otherStatus == 2){newVal=STEXPLORER;}else{newVal=STLOVER;}

    sprintf(tmp,"%d",newVal);
    printf("robot %d sends an update %s \n \n", robotID, tmp);
    send_msg(tmp);


}

void sendCurrentStatus(int robotID){
    char tmp[5];
    init_communication();
    sprintf(tmp,"%d", states[robotID-1][0]);
    printf("robot %d sends his current status %s \n", robotID, tmp);
    send_msg(tmp);

}

void receiveUpdateState(int robotId){
    char rcv[5];
    int currentState = states[robotId - 1][0];
    init_communication();
    receive_msg(rcv);
    if (strcmp(rcv,"ZZZZ")!=0 && currentState != atoi(rcv) && atoi(rcv)!=2) {
        printf("robot %d receives a state update %s \n", robotId, rcv);
        states[robotId-1][0] =  atoi(rcv);
    }

}

int receiveOtherStatus(){
    char rcv[5];
    init_communication();
    receive_msg(rcv);
    printf("Otherrecieved=%s , test = %d",rcv, strcmp(rcv,"ZZZZ"));
    if (strcmp(rcv,"ZZZZ")!=0) {
        printf("robot receives a status %s \n", rcv);
        return atoi(rcv);
    }else{
        return -1;
    }

}


