
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
double a = 1, b = 5, c = 5, d = 0;

int STATE = 0;
int WALL = 0;
char D;
int counter = 0;
int x = 0;
int rand_int;

void clockwise(), anticlockwise(), lover();

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
        double prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a+b+c+d);
        double prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a+b+c+d);
        double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;
        double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
        double speed_right = bounded_speed(NORM_SPEED - ds_right);
        double speed_left = bounded_speed(NORM_SPEED - ds_left);
        printf("%f %f\n",speed_left, speed_right );
        printf("STATE: %d, WALL: %d, Counter: %d\n", STATE, WALL, rand_int);

        if(STATE == 0 && WALL == 0){
          if (speed_right < 0.1 && speed_left < 0.1) {
            WALL = 1;
            printf("%d, %d\n",prox_values[0], prox_values[7]);
            srand(time(NULL));
            rand_int = rand() % 2;
            D = rand_int == 0 ? 'R' : 'L';
          } else{
            lover(speed_left, speed_right);
          }
        }
        else if(STATE == 0 && WALL == 1){
          if (D == 'R') {
            if (counter <= 17) {
              clockwise();
              counter++;
            }else{
              STATE = 1;
              WALL = 0;
              counter = 0;
            }
            printf("STATE: %d, WALL: %d, D: %c, Counter: %d,\n", STATE, WALL, D, counter);
          } else if (D == 'L') {
            if (counter <= 17) {
              anticlockwise();
              counter++;
            }else{
              STATE = 1;
              WALL = 0;
              counter = 0;
            }
            printf("STATE: %d, WALL: %d,D: %c, Counter: %d, x: %d\n", STATE, WALL, D, counter, x);
          }

        }
        else if(STATE == 1) {
          if (WALL == 1) {
            if (D == 'R') {
              if (counter < 17) {
                clockwise();
                counter++;
              }else{
                STATE = 1;
                WALL = 0;
                counter = 0;
              }
              printf("STATE: %d, WALL: %d,D: %c, Counter: %d, x: %d\n", STATE, WALL, D, counter, x);
            } else if (D == 'L') {
              if (counter < 17) {
                anticlockwise();
                counter++;
              }else{
                STATE = 1;
                WALL = 0;
                counter = 0;
              }
            }
            printf("STATE: %d, WALL: %d,D: %c, Counter: %d, x: %d\n", STATE, WALL, D, counter, x);

          }
          if (WALL == 0) {
            if ((prox_values[0] > (PID_MAX_DS-20)) && (prox_values[7] > (PID_MAX_DS))) {
              WALL = 1;
            }

          double a = 1, b = 5, c = 5, d = 0;
          double prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a+b+c+d);
          double prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a+b+c+d);
          //compute PID response according to IR sensor value
          if (D == 'R'){
            double ds = pid(prox_left);
            double speed_right = (abs(ds)>PID_MAX_DS ? 0 : NORM_SPEED) - ds;
            double speed_left  = (abs(ds)>PID_MAX_DS ? 0 : NORM_SPEED) + ds;
            fprintf(log, "%d,%f,%f,%f,%f,%f,%f\n", loop_counter, K*error, K*integ/T_I, K*deriv*T_D, ds, speed_left,speed_right);

          set_speed(bounded_speed(speed_left), bounded_speed(speed_right));
          } else if (D == 'L'){
          //double ds = (D == 'L') ? pid(prox_right) : -pid(prox_left);
          //printf("R: %f, L: %f, ds: %f\n", pid(prox_right), pid(prox_left), ds);
          // adjust speed values to turn away from obstacle
          double ds = pid(prox_right);
          double speed_right = (abs(ds)>PID_MAX_DS ? 0 : NORM_SPEED) + ds;
          double speed_left  = (abs(ds)>PID_MAX_DS ? 0 : NORM_SPEED) - ds;
          fprintf(log, "%d,%f,%f,%f,%f,%f,%f\n", loop_counter, K*error, K*integ/T_I, K*deriv*T_D, ds, speed_left,speed_right);

          set_speed(bounded_speed(speed_left), bounded_speed(speed_right));
          }
          // write a line of data in log file
          //fprintf(log, "%d,%f,%f,%f,%f,%f,%f\n", loop_counter, K*error, K*integ/T_I, K*deriv*T_D, ds, speed_left,speed_right);

          //set_speed(bounded_speed(speed_left), bounded_speed(speed_right));

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
