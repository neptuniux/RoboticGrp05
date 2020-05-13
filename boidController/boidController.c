
#define SIMULATION 1

#include <math.h>

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

#define MAX_ROTA 2
#define MAX_PROX 200
#define PI acos(-1.0)

char rcv[MSG_LENGTH];
char cur[15];
char rota[15];
char x_str[15];
char y_str[15];
double rotation, current, x, y;
WbDeviceTag emitter,receiver;

double error = 0;
double deriv = 0;
double integ = 0;


void robot_setup() {
    init_robot();
    init_sensors();
    //init_communication();
    init_boids();
}

void robot_loop() {
  int id = get_robot_ID();
  short int prox_values[8];

  wb_receiver_set_channel(receiver, id + 1);


  while (robot_go_on()) {
    receive_msg(rcv);
    char token = rcv[0];

    if(token == 't'){
      strncpy(rota, rcv + 1, strlen(rcv) - 1);
      rotation = strtof(rota, NULL);
      if(rotation < 0.1 && rotation > -0.1){
        rotation = 0;
      }
      //printf("id = %d r = %f\n", id, rotation);
    } else if (token == 'c') {
      strncpy(cur, rcv + 1, strlen(rcv) - 1);
      current = strtof(cur, NULL);
      if(current < 0.02 && current > -0.02){
        current = 0;
      }
      //printf("id = %d, c =%f\n", id, current);
    } else if (token == 'x') {
      strncpy(x_str, rcv + 1, strlen(rcv) - 1);
      x = strtof(x_str, NULL);
      //printf("id = %d x = %f\n", id, x);
    } else if (token == 'y') {
      strncpy(y_str, rcv + 1, strlen(rcv) - 1);
      y = strtof(y_str, NULL);
      //printf("d = %d y = %f\n", id, y);
    }
    
    //printf("id = %d r = %f c= %f\n", id, rotation, current);
    // fetch proximity sensor values
    get_prox_calibrated(prox_values);

    // basic explorer behavior
    double prox_right = (1*prox_values[0] + 2*prox_values[1] + 2*prox_values[2] + 1*prox_values[3]) / 4.;
    double prox_left = (1*prox_values[7] + 2*prox_values[6] + 2*prox_values[5] + 1*prox_values[4]) / 4.;
    double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;
    double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
    double speed_right = bounded_speed(MAX_SPEED/2 - ds_right);
    double speed_left = bounded_speed(MAX_SPEED/2 - ds_left);
    // make sure speed values are legal

    if(prox_right < 250 && prox_left < 250){
      //printf("id = %d, r = %f\n", id, rotation);
      if(rotation > 0){
        speed_left = bounded_speed(speed_left * 1);
        speed_right = speed_right*0.2;
      } else if (rotation < 0){
        speed_right = bounded_speed(speed_right * 1);
        speed_left = speed_left*0.2;
      }
    } else{
      speed_right = bounded_speed(NORM_SPEED - ds_right);
      speed_left = bounded_speed(NORM_SPEED - ds_left);
    }

    set_speed(speed_right,speed_left);
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
