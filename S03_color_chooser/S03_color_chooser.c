
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

// states
#define LOVER 1
#define EXPLORER 2
#define AVOID 3

#define NONE 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define WHITE 4
#define PIXEL_THRESHOLD 600

#define COUNT_EXP 15
#define COUNT_LOV 20
#define COUNT_OBST 1
#define THRESH_SPEED 0.00001
#define THRESH_PROX_LO 80
#define THRESH_PROX_HI 200
#define DIST 1

int color();
void led_on(int), led_off();

void robot_setup() {
    init_robot();
    init_sensors();
    init_camera();
    init_rgbled();
    init_tof();
    calibrate_prox();
}

void robot_loop() {
	short int prox_values[8];
  short int tof[TOF_SENSOR_COUNT];

	int counter_wall = 0;
	int counter_expl = 0;
	int counter_obstacle = 1;
  int counter = 0;
  int state = LOVER;


    while (robot_go_on()) {

        get_prox_calibrated(prox_values);
        get_tof(tof);

        double prox_left = (4 * prox_values[7] + 2 * prox_values[6] + prox_values[5]) / 7.;
        double prox_right = (4 * prox_values[0] + 2 * prox_values[1] + prox_values[2]) / 7.;

        double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
        double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;

        double speed_left = NORM_SPEED - ( ((state == EXPLORER)||(state == AVOID)) ? ds_right : ds_left);;
        double speed_right = NORM_SPEED - ( ((state == EXPLORER)||(state == AVOID)) ? ds_left : ds_right);;

        // increment counters
        if (state == LOVER) {

          if (((abs(speed_right) < THRESH_SPEED) && (abs(speed_left) < THRESH_SPEED))|| tof[0] < DIST){
              counter_wall++;
              if(counter < 1){
                int wall_color = color();

                if (wall_color == RED) {
                  led_on(RED);
                  printf("Wall is red\n");
                } else if (wall_color == GREEN) {
                  led_on(GREEN);
                  printf("Wall is green\n");
                } else if (wall_color == BLUE) {
                  led_on(BLUE);
                  printf("Wall is blue\n");
                } else if (wall_color == WHITE) {
                  led_off();
                  printf("Wall is white\n");
                } else if (wall_color == NONE) {
                  led_on(NONE);
                  printf("Unknown\n");
                }
                counter++;
              }

            }
        } else if (state == EXPLORER){
            counter_expl++;
        } else if (state == AVOID){
            if ((abs(prox_right) < THRESH_PROX_LO) && (abs(prox_left) < THRESH_PROX_LO)) {
                counter_expl++;
            }
        }

        // state machine
        if (state == EXPLORER) {
            if (counter_obstacle > COUNT_OBST) {
              state = LOVER;
              counter_wall = 0;
              counter_obstacle = 0;
            } else if (counter_expl > COUNT_EXP) {
              state = AVOID;
              counter_expl = 0;
            }
        } else if ((state == AVOID) && (counter_expl > COUNT_EXP)) {
              counter_expl = 0;
              counter_obstacle++;
              counter = 0;
              state = EXPLORER;
        } else if ((state == LOVER) && (counter_wall > COUNT_LOV)) {
              counter_expl = 0;
              counter_obstacle = 0;
              state = AVOID;
        }


        speed_left = bounded_speed(speed_left);
        speed_right = bounded_speed(speed_right);

        set_speed(speed_left, speed_right);
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
      if(redpixel > 150){
        redpx++;
      }
      if (greenpixel > 150){
        greenpx++;
      }
      if (bluepixel > 150) {
        bluepx++;
      }
    }
  }
  printf("%d, %d, %d\n",redpx, greenpx, bluepx);
  if (redpx > greenpx && redpx > bluepx) {
    return RED;
  } else if (greenpx > redpx && greenpx > bluepx) {
    return GREEN;
  } else if (bluepx > redpx && bluepx > greenpx) {
    return BLUE;
  } else if(redpx == greenpx && redpx == bluepx && greenpx == bluepx) {
    return WHITE;
  }
  /*if (redpx > PIXEL_THRESHOLD && greenpx < redpx && bluepx < redpx) {
    return RED;
  } else if (redpx <greenpx && greenpx > PIXEL_THRESHOLD && bluepx < greenpx) {
    return GREEN;
  } else if (redpx < bluepx && greenpx < bluepx && bluepx > PIXEL_THRESHOLD) {
    return BLUE;
  } else if(redpx > 1000 && greenpx > 1000 && bluepx > 1000) {
    return 0;
  }*/
  return NONE;
}

void led_on(int color){
  switch (color) {
    case 1:
    disable_rgbled(0);
    disable_rgbled(1);
    enable_rgbled(2, 0xff0000);
    enable_rgbled(3, 0xff0000);
    break;
    case 2:
    disable_rgbled(0);
    enable_rgbled(1, 0x00ff00);
    enable_rgbled(2, 0x00ff00);
    disable_rgbled(3);
    break;
    case 3:
    enable_rgbled(0, 0x0000ff);
    enable_rgbled(1, 0x0000ff);
    disable_rgbled(2);
    disable_rgbled(3);
    break;
    case 4:
    enable_rgbled(0, 0x0000ff);
    enable_rgbled(1, 0x00ff00);
    enable_rgbled(2, 0xff0000);
    enable_rgbled(3, 0x0000ff);
  }
}

void led_off(){
  disable_rgbled(0);
  disable_rgbled(1);
  disable_rgbled(2);
  disable_rgbled(3);
}
