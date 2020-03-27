
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

#include <stdlib.h>
#include <time.h>



#define TIME_STEP 64


#define K 0.015
#define T_I 1000
#define T_D 0.15
#define PID_WALL_FOLLOW_TARGET 200
#define PID_MAX_DS 200
// define the states, FIND a wall & FOLLOW a wall
#define LOVER 0
#define FOLLOW 1
// define the values for wall detection
#define EMPTY 0
#define FOUND 1
//define values for turn direction
#define LEFT 'L'
#define RIGHT 'R'


double a = 1, b = 5, c = 5, d = 0;


int STATE = LOVER;
int WALL = EMPTY;
int ANGLE = 0;
char direction;
int counter = 0;
int rand_int;

// methods declaration
void clockwise(), anticlockwise(), lover(), led_on(char side), led_off(char side);
int random_int();

double error = 0;
double deriv = 0;
double integ = 0;

double pid(double proxLR) {

  double prev_err = error;
  error = proxLR - PID_WALL_FOLLOW_TARGET;

  //deriv = (error - prev_err);
  deriv = (error - prev_err)*1000/TIME_STEP;
  //integ += error;
  integ += error*TIME_STEP/1000;

  return K * ( error + 1.0 / T_I * integ + T_D * deriv);
}

void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
}

void robot_loop() {
	short int prox_values[PROX_SENSORS_COUNT];

	// open files for writing
    FILE *log = fopen("logPID.csv", "w");

    if (log == NULL) {
        printf("Error opening file!\n");
        return ;
    }

    // write header in CSV file
    fprintf(log, "time,P,I,D,ds,left speed,right speed\n");

    int loop_counter = 0;

    while (robot_go_on()) {

        loop_counter++;

        get_prox_calibrated(prox_values);

        if(STATE == LOVER && WALL == EMPTY){
          double prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a+b+c+d);
          double prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a+b+c+d);
          double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;
          double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
          double speed_right = bounded_speed(NORM_SPEED - ds_right);
          double speed_left = bounded_speed(NORM_SPEED - ds_left);
          // Reached equilibrium
          if (speed_right < 0.1 && speed_left < 0.1) {
            WALL = FOUND;
            direction = (random_int() == 0) ? RIGHT : LEFT;
          } else{
            lover(speed_left, speed_right);
          }
        }
        else if(STATE == LOVER && WALL == FOUND){
          if (direction == RIGHT) {
            if (counter <= 17) {
              clockwise();
              counter++;
            }else{
              STATE = FOLLOW;
              WALL = EMPTY;
              counter = 0;
            }
          } else if (direction == LEFT) {
            if (counter <= 17) {
              anticlockwise();
              counter++;
            }else{
              STATE = FOLLOW;
              WALL = EMPTY;
              counter = 0;
            }
          }
        }
        else if(STATE == FOLLOW) {
          //If a wall has been found turn in the current direction
          if (WALL == FOUND) {
            if (direction == RIGHT) {
              if (counter < 17) {
                clockwise();
                counter++;
              }else{
                STATE = FOLLOW;
                WALL = EMPTY;
                counter = 0;
              }
            } else if (direction == LEFT) {
              if (counter < 17) {
                anticlockwise();
                counter++;
              }else{
                STATE = FOLLOW;
                WALL = EMPTY;
                counter = 0;
              }
            }
          }
          //If there is no wall in front register when one is found
          if (WALL == EMPTY) {
            if ((prox_values[0] > (PID_WALL_FOLLOW_TARGET)) && (prox_values[7] > (PID_WALL_FOLLOW_TARGET))) {
              WALL = FOUND;
            }
/////////////////////////////// Follow the wall and register logs /////////////////////////////////////////////////////////////////////////////
          double a = 0, b = 5, c = 5, d = 0;
          double prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a+b+c+d);
          double prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a+b+c+d);
          // if no wall on left or right, register needs to turn
          if (prox_right < PID_WALL_FOLLOW_TARGET/3 || prox_left < PID_WALL_FOLLOW_TARGET/3) {
            ANGLE = 1;
          }
          //compute PID response according to IR sensor value
          if (direction == RIGHT){
            if (ANGLE == 1) {
              if (counter <= 30) {
                set_speed(NORM_SPEED, 0.2*NORM_SPEED);
                counter++;
              }else{
                ANGLE = 0;
                counter = 0;
              }
            }
            // adjust speed values to turn away from obstacle, with wall to the left
            double ds = pid(prox_left);
            double speed_right = (abs(ds)>PID_MAX_DS ? 0 : NORM_SPEED) - ds;
            double speed_left  = (abs(ds)>PID_MAX_DS ? 0 : NORM_SPEED) + ds;
            fprintf(log, "%d,%f,%f,%f,%f,%f,%f\n", loop_counter, K*error, K*integ/T_I, K*deriv*T_D, ds, speed_left,speed_right);
            led_off(RIGHT);
            led_on(LEFT);
            set_speed(bounded_speed(speed_left), bounded_speed(speed_right));

          } else if (direction == LEFT){
            if (ANGLE == 1) {
              if (counter <= 30) {
                set_speed(0.2*NORM_SPEED, NORM_SPEED);
                counter++;
              }else{
                ANGLE = 0;
                counter = 0;
              }
            }
            // adjust speed values to turn away from obstacle, with wall to the right
            double ds = pid(prox_right);
            double speed_right = (abs(ds)>PID_MAX_DS ? 0 : NORM_SPEED) + ds;
            double speed_left  = (abs(ds)>PID_MAX_DS ? 0 : NORM_SPEED) - ds;
            fprintf(log, "%d,%f,%f,%f,%f,%f,%f\n", loop_counter, K*error, K*integ/T_I, K*deriv*T_D, ds, speed_left,speed_right);
            led_off(LEFT);
            led_on(RIGHT);
            set_speed(bounded_speed(speed_left), bounded_speed(speed_right));

          }
        }
      }
    }
  }

int main (int argc, char **argv) {
    #if SIMULATION
    #else
    ip = argv[1];
    #endif

    robot_setup();
    robot_loop();
}
void lover(double speed_left, double speed_right){
  a = 2, b = 1, c = 0, d = 0;
  set_speed(speed_left, speed_right);
}


void clockwise(){
    set_speed(NORM_SPEED, -NORM_SPEED);
}

void anticlockwise(){
    set_speed(-NORM_SPEED, NORM_SPEED);
}

int random_int(){
  srand(time(NULL));
  rand_int = rand() % 2;
  return rand_int;
}

void led_on(char side){
  switch (side) {
    case RIGHT:
    enable_led(0);
    enable_led(1);
    break;

    case LEFT:
    enable_led(3);
    enable_led(2);
    break;
  }
}

void led_off(char side){
  switch (side) {
    case RIGHT:
    disable_led(0);
    disable_led(1);
    break;

    case LEFT:
    disable_led(3);
    disable_led(2);
    break;
  }
}
