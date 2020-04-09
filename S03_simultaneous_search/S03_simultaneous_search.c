
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

void explorer(), lover(), stop();
int send_id(int), send(char[]);

//typedef int bool;
#define STOP 0
#define LOVER 1
#define EXPLORER 2
#define MSG_LENGTH 5
#define NB_ROBOT 3    //MUST match number of robots on the field.

// control board, i.e variables that control the transitions
int robot[NB_ROBOT];
int robot_count=0;
int ack = 0;
int go = 0;
int sent = 0;


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


void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
    init_communication();
    // initialize robot[] to 0
    for (int r = 0; r < NB_ROBOT; r++) {
      robot[r] = 0;
    }
}

void robot_loop() {
    int id = get_robot_ID();
    short int prox_values[8];

    while (robot_go_on()) {
      //control that robot_count doesn't go over the array size
      if(robot_count >= NB_ROBOT){
        robot_count = 0;
      }
      receive_msg(rcv);
      int rcv_id = rcv[0]- 48;
      if (rcv_id <= NB_ROBOT){
        robot[robot_count] = rcv_id;
        robot_count++;
      } else if (strcmp(rcv,"go")==0){
        go=1;
      }else if (sent == 1) {
        sent = 0;
        robot[robot_count] = id;
        robot_count++;
      }

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
          if (speed_right < 0.0 && speed_left < 0.0) {
            for(int i = 0; i< NB_ROBOT; i++){
              if (robot[i] > 0){
                ack++;
              }
            }
            if (ack == NB_ROBOT - 1) {
              sent = send_id(id);
              go = send("go");
              stop();
            }else {
              sent = send_id(id);
              stop();
            }
          } else {
            lover(speed_left, speed_right);
          }
        } else if(STATE == STOP){
          if(go == 1){
            robot_count = 0;
            ack = 0;
            go = 0;
            for (int r = 0; r < NB_ROBOT; r++) {
              robot[r] = 0;
            }
            explorer(speed_left, speed_right);
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

int send_id(int id){
  sprintf(tmp, "%d", id);
  send_msg(tmp);
  return 1;
}

int send(char msg[]){
  sprintf(tmp, "%s", msg);
  send_msg(tmp);
  return 1;
}
