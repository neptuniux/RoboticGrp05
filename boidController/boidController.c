
#define SIMULATION 1

#include <math.h>

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

/// Weight of the rotation and center of mass adjustment
#define K_ROTATION 2.0
#define K_CENTER 1.0

char rcv[MSG_LENGTH];
// strings used to temporarly hold the values received before float transformation
char rota[15];
char centr[15];

double rotation, mass_center;

void robot_setup() {
    init_robot();
    init_prox();
    init_boids();
}

void robot_loop() {
  int id = get_robot_ID();
  short int prox_values[8];
  double rotation = 0, mass_center = 0;
  double change_rota_left = 0, change_rota_right = 0, change_cntr_left = 0, change_cntr_right = 0;

/// Set the channel of the robot to the ID + 1 (see webotsAPI.c)
  setChannel(id);


  while (robot_go_on()) {
    receive_msg(rcv);
    char token = rcv[0];

/// Manage received message depending on the token (first char)
    if(token == 't'){
      // copy the string after the token and convert it to float
      strncpy(rota, rcv + 1, strlen(rcv) - 1);
      rotation = strtof(rota, NULL);
      // restrict the values to reduce impact small changes
      //(since the speed adjustment is not proportinal to the value of rotation)
      if(rotation < 0.05 && rotation > -0.05){
        rotation = 0;
      }
    } else if (token == 'c') {
      // copy the string after the token and convert it to float
      strncpy(centr, rcv + 1, strlen(rcv) - 1);
        mass_center = strtof(centr, NULL);
        // restrict the values to reduce impact small changes
        //(since the speed adjustment is not proportinal to the value of rotation)
        if(mass_center != 0.0 /*&& mass_center > 0.0*/){
            mass_center = 0;
        }
    }
    get_prox(prox_values);
    //printf("id = %d, %f \n", id, mass_center);

    // basic explorer behavior
    double prox_right = (2*prox_values[0] + 2*prox_values[1] + 2*prox_values[2] + 1*prox_values[3]) / 5.;
    double prox_left = (2*prox_values[7] + 2*prox_values[6] + 2*prox_values[5] + 1*prox_values[4]) / 5.;
    double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;
    double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
    double speed_right = bounded_speed(NORM_SPEED*1.5 - ds_right);
    double speed_left = bounded_speed(NORM_SPEED * 1.5 - ds_left);

    /// Adjust the speed only if the robot is not currently avoiding an obstacle
    /// And if there is adjustment to be done
    if(prox_right < MAX_PROX+25 && prox_left < MAX_PROX+25){
      if(rotation != 0){
        if(rotation < 0){
          change_rota_left = 1;
          change_rota_right = -1;
        } else if (rotation > 0){
          change_rota_left = -1;
          change_rota_right = 1;
        }
      }else{
        change_rota_left = 0;
        change_rota_right = 0;
      }
      if(mass_center != 0){
        if(mass_center < 0){
          change_cntr_left = 1;
          change_cntr_right = -1;
        }else if(mass_center >0){
          change_cntr_left = -1;
          change_cntr_right = 1;
        }
      } else{
        change_cntr_left = 0;
        change_cntr_right = 0;
      }
      // Set the speed as a compound of the adjustment from the rotation and center of mass
      speed_right = bounded_speed(speed_right +((K_ROTATION * change_rota_right + K_CENTER * change_cntr_right)/(K_ROTATION + K_CENTER)));
      speed_left = bounded_speed(speed_left +((K_ROTATION * change_rota_left + K_CENTER * change_cntr_left)/(K_ROTATION + K_CENTER)));
    }
    /// To free a robot that is stuck with its back against a wall
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
