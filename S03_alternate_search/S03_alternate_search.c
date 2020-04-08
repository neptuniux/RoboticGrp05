//
// Created by daniel on 02.04.20.
//


#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

void explorer(), lover(), stop();

//typedef int bool;
#define STOP 0
#define LOVER 1
#define EXPLORER 2
#define MSG_LENGTH 5
#define NB_ROBOT 2

// control board

int init = 1;
int EQ = 0;
int ack = 0;
int go = 0;

int STATE = LOVER;
int SAW_OBSTACLE = 0;
int HAS_STOPPED = 0;
int a = 1;
int b = 2;
int c = 3;
int d = 4;
double e = 10.;

char rcv[MSG_LENGTH];
char tmp[MSG_LENGTH];
char tmp2[MSG_LENGTH];


void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
    init_communication();
}

void robot_loop() {
    int id = get_robot_ID();
    short int prox_values[8];


    while (robot_go_on()) {


      receive_msg(rcv);
      if (strcmp(rcv,"stop")==0){
        ack++;
        printf("The messages recieved id stop \n");

      } else if (strcmp(rcv,"go")==0){
        go++;
        printf("The messages recieved id go \n");

      }else if (strcmp(rcv,"EQ")==0){
        EQ = 1;
        printf("The messages recieved id EQ \n");

      }


        //printf("ID %d, %d, %d\n", id, SAW_OBSTACLE, STATE);
        get_prox_calibrated(prox_values);
        double prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / e;
        double prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / e;
        double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;
        double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
        double speed_right = bounded_speed(NORM_SPEED - ds_right);
        double speed_left = bounded_speed(NORM_SPEED - ds_left);

        if (STATE == EXPLORER) {
            if (speed_right < NORM_SPEED*0.25 || speed_left < NORM_SPEED*0.25) {
                SAW_OBSTACLE = 1;
                explorer(speed_left, speed_right);
            } else if ((speed_right > NORM_SPEED*0.5 && speed_left > NORM_SPEED*0.5) && SAW_OBSTACLE) {
                SAW_OBSTACLE = 0;
                lover(speed_left, speed_right);
            } else {
                explorer(speed_left, speed_right);
            }

        } else if (STATE == LOVER) {
            if ((prox_right > 80.0 || prox_left > 80.0) && init == 1) {
              if (ack == NB_ROBOT - 1) {
                stop();
                sprintf(tmp2, "%s", "go");
                send_msg(tmp2);
              }else {
              sprintf(tmp, "%s", "stop");
              send_msg(tmp);
              stop();
              init = 0;
              }
            } else if (speed_right < 0.0 && speed_left < 0.0) {
              stop();
              sprintf(tmp2, "%s", "EQ");
              send_msg(tmp2);
            } else {
                lover(speed_left, speed_right);
            }
        } else if(STATE == STOP){
            if(go == NB_ROBOT - 1){
              lover(speed_left, speed_right);
              go = 0;
            }
            if(EQ == 1){
            EQ = 0;
              if(init == 0){
                explorer(speed_left, speed_right);
              } else {
                lover(speed_left, speed_right);
                init = 0;
              }
            }
        }
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

void explorer(double speed_left, double speed_right) {
    STATE = EXPLORER;
    a = 1;
    b = 2;
    c = 2;
    d = 1;
    e = a + b + c + d;
    for(int i = 0; i < 4; i++){
      disable_rgbled(i);
      enable_rgbled(i, 0x0000ff);
    }

    set_speed(speed_right, speed_left);
}

void lover(double speed_left, double speed_right) {
    STATE = LOVER;
    a = 5;
    b = 3;
    c = 2;
    d = 1;
    e = a + b + c + d;
    for(int i = 0; i < 4; i++){
      disable_rgbled(i);
      enable_rgbled(i, 0xff0000);
    }
    set_speed(speed_left, speed_right);

}

void stop(){
  STATE = STOP;
  for(int i = 0; i < 4; i++){
    disable_rgbled(i);
    enable_rgbled(i, 0x00ff00);
  }
  set_speed(0, 0);
}
