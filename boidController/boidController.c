
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

#define weight_rotation 5
#define weight_center 1

char rcv[MSG_LENGTH];
char cur[15];
char rota[15];
char centr[15];
double rotation, current, x, y, masscentre;

WbDeviceTag emitter,receiver;

double error = 0;
double deriv = 0;
double integ = 0;


void robot_setup() {
    init_robot();
    init_prox();
    //init_communication();
    init_boids();
}

void robot_loop() {
  int id = get_robot_ID();
  short int prox_values[8];
  rotation = 0;

  wb_receiver_set_channel(receiver, id + 1);

  double change_rota_left=0;
  double change_rota_right=0;
  double change_cntr_left=0;
  double change_cntr_right=0;


  while (robot_go_on()) {
    receive_msg(rcv);
    char token = rcv[0];

    if(token == 't'){
      strncpy(rota, rcv + 1, strlen(rcv) - 1);
      rotation = strtof(rota, NULL);
      if(rotation < 0.1 && rotation > -0.1){
        rotation = 0;
      }
    } else if (token == 'c') {
      strncpy(cur, rcv + 1, strlen(rcv) - 1);
      current = strtof(cur, NULL);
    } else if (token == 'c') {
        strncpy(centr, rcv + 1, strlen(rcv) - 1);
        masscentre= strtof(centr, NULL);
        if(masscentre < 0.1 && masscentre > -0.1){
            masscentre= 0;
        }
    }
    get_prox(prox_values);

    // basic explorer behavior
    double prox_right = (2*prox_values[0] + 2*prox_values[1] + 2*prox_values[2] + 1*prox_values[3]) / 7.;
    double prox_left = (2*prox_values[7] + 2*prox_values[6] + 2*prox_values[5] + 1*prox_values[4]) / 7.;
    double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;
    double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
    double speed_right = bounded_speed(NORM_SPEED - ds_right);
    double speed_left = bounded_speed(NORM_SPEED - ds_left);
    // make sure speed values are legal

    if(prox_right < MAX_PROX && prox_left < MAX_PROX){
      if(rotation > 0){
          change_rota_left=bounded_speed(speed_left*1.5);
          change_rota_right=bounded_speed(speed_right*0.6);

        //speed_left = bounded_speed(speed_left * 1.5);
        //speed_right = speed_right*0.6;
      } else if (rotation < 0){
          change_rota_left=bounded_speed(speed_left*0.6);
          change_rota_right=bounded_speed(speed_right*1.5);

        //speed_right = bounded_speed(speed_right * 1.5);
        //speed_left = speed_left*0.6;
      }
      if(masscentre >0){
          change_cntr_left=bounded_speed(speed_left*1.5);
          change_cntr_right=bounded_speed(speed_right*0.6);
      }else if(masscentre <0){
          change_cntr_left=bounded_speed(speed_left*0.6);
          change_cntr_right=bounded_speed(speed_right*1.5);
      }
      speed_right=bounded_speed((weight_rotation*change_rota_right+weight_center*change_cntr_right)/(weight_rotation+weight_center));
      speed_left=bounded_speed((weight_rotation*change_rota_left+weight_center*change_cntr_left)/(weight_rotation+weight_center));


    } else{
      speed_right = bounded_speed(NORM_SPEED - ds_right);
      speed_left = bounded_speed(NORM_SPEED - ds_left);
    }
    if(prox_values[3] > 600 && prox_values[4] > 600){
      speed_right = bounded_speed(NORM_SPEED);
      speed_left = bounded_speed(NORM_SPEED);
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
