
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

#define MAX_GROUND 800
#define MIN_GROUND 300

#define K 0.009
#define T_I 100
#define T_D 0.001
#define PID_DELTA_TARGET 490
#define PID_MAX_DS 490

#define FIND 0
#define LINE 1
#define WALL 2

#define NONE 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define WHITE 4

int color();
void led_on(int), led_off(), init_rgbled();


double error = 0;
double deriv = 0;
double integ = 0;

double pid(double delta) {

  double prev_err = error;
  error = delta - PID_DELTA_TARGET;
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
    init_camera();
    init_rgbled();


}

void robot_loop() {
   short int IR_ground[GROUND_SENSORS_COUNT];
   short int prox_values[8];
   /*
   // open files for writing
    FILE *log = fopen("logPID.csv", "w");

    if (log == NULL) {
        printf("Error opening file!\n");
        return ;
    }

    // write header in CSV file
    fprintf(log, "time,P,I,D,ds,left speed,right speed\n");
    */
    int loop_counter = 0;
    int STATE = FIND;

    while (robot_go_on()) {
        loop_counter++;
        double ds;
        get_ground(IR_ground);
        get_prox_calibrated(prox_values);
        double prox = (prox_values[0] + prox_values[7])/2.0;
        double ds_prox = (NORM_SPEED * prox) / MAX_PROX;
        double gs = (IR_ground[GS_RIGHT] + IR_ground[GS_CENTER] + IR_ground[GS_LEFT]) /3.0;
        if (STATE == FIND){
          if (gs < 350) {
            STATE = LINE;
          } else {
            set_speed(NORM_SPEED, NORM_SPEED);
          }
        } else if (STATE == LINE) {
        printf("%f\n", prox);
          if (prox >= MAX_PROX) {
            STATE = WALL;
          } else if (gs > 475 && gs < 495){
          // the values of the sensors is betwwen the detection error, to reduce shackiness
          ds = 0;
          } else{
          ds = pid(gs);
          }
          double speed_right = bounded_speed((abs(ds)>(PID_MAX_DS) ? 0 : NORM_SPEED) + ds);
          double speed_left  = bounded_speed((abs(ds)>(PID_MAX_DS) ? 0 : NORM_SPEED) - ds);
          set_speed(speed_left, speed_right);
          //fprintf(log, "%d,%f,%f,%f,%f,%f,%f\n", loop_counter, K*error, K*integ/T_I, K*deriv*T_D, ds, speed_left,speed_right);
          } else if (STATE == WALL){
          double speed_right = bounded_speed(NORM_SPEED - ds_prox);
          double speed_left = bounded_speed(NORM_SPEED - ds_prox);
          if(speed_right < 0.01 && speed_left < 0.01){
            if (color() == RED) {
              led_on(RED);
              printf("Wall is red\n");
            } else if (color() == GREEN) {
              led_on(GREEN);
              printf("Wall is green\n");
            } else if (color() == BLUE) {
              led_on(BLUE);
              printf("Wall is blue\n");
            } else if (color() == WHITE) {
              led_off();
              printf("Wall is white\n");
            } else if (color() == NONE) {
              led_on(NONE);
              printf("Unknown\n");
            }
            set_speed(0, 0);
          }
          set_speed(speed_left, speed_right);
          
        }
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

int color(){
  unsigned char red[CAMERA_WIDTH * CAMERA_HEIGHT];
  unsigned char green[CAMERA_WIDTH * CAMERA_HEIGHT];
  unsigned char blue[CAMERA_WIDTH * CAMERA_HEIGHT];
  int redpx = 0;
  int greenpx = 0;
  int bluepx = 0;
  get_camera(red, green, blue);
  for (int n = 55; n < 64; n++) {
    for (int m = 0; m < CAMERA_WIDTH; m++) {
      int redpixel = (int) red[n * CAMERA_WIDTH + m];
      int greenpixel = (int) green[n * CAMERA_WIDTH + m];
      int bluepixel = (int) blue[n * CAMERA_WIDTH + m];
      if(redpixel > 100){
        redpx++;
      }
      if (greenpixel > 100){
        greenpx++;
      }
      if (bluepixel > 100) {
        bluepx++;
      }
    }
  }
  if (redpx > greenpx && redpx > bluepx) {
  printf("%d %d %d\n", redpx, greenpx, bluepx);
    return RED;
  } else if (greenpx > redpx && greenpx > bluepx) {
  printf("%d %d %d\n", redpx, greenpx, bluepx);
    return GREEN;
  } else if (bluepx > redpx && bluepx > greenpx) {
  printf("%d %d %d\n", redpx, greenpx, bluepx);
    return BLUE;
  } else if(redpx == greenpx && redpx == bluepx && greenpx == bluepx) {
    printf("%d %d %d\n", redpx, greenpx, bluepx);
    return WHITE;
  }
  return NONE;
}
void led_on(int color){
  switch (color) {
    case 1:
    for(int i = 0; i < 4; i++){
      disable_rgbled(i);
      enable_rgbled(i,0xff0000);
      disable_led(i);
    }
    enable_led(3);
    break;
    case 2:
    for(int i = 0; i < 4; i++){
      disable_rgbled(i);
      enable_rgbled(i,0x00ff00);
      disable_led(i);
    }
    enable_led(2);
    break;
    case 3:
    for(int i = 0; i < 4; i++){
      disable_rgbled(i);
      enable_rgbled(i,0x0000ff);
      disable_led(i);
    }
    enable_led(1);
    break;
    case 4:
    enable_rgbled(0, 0x0000ff);
    enable_rgbled(1, 0x00ff00);
    enable_rgbled(2, 0xff0000);
    enable_rgbled(3, 0x0000ff);
  }
}

void led_off(){
  for(int i = 0; i < 4; i++){
    disable_rgbled(i);
    disable_led(i);
  }
}

