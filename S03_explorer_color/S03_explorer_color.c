
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

#define RED 1
#define GREEN 2
#define BLUE 3

// states
#define EXPLORER 1
#define AVOID 2


#define COUNT_EXP 5
#define COUNT_OBST 1
#define THRESH_SPEED 100
#define THRESH_PROX_LO 80
#define THRESH_PROX_HI 200


void robot_setup() {
    init_robot();
    init_sensors();
    init_camera();
    init_rgbled();
    calibrate_prox();
}

void robot_loop() {
  int color();
  int counter = 0;
  short int prox_values[8];
  int counter_expl = 0;
  int counter_obstacle = 1;
  int state = EXPLORER;


    while (robot_go_on()) {

        get_prox_calibrated(prox_values);

        double prox_left = (4 * prox_values[7] + 2 * prox_values[6] + prox_values[5]) / 7.;
        double prox_right = (4 * prox_values[0] + 2 * prox_values[1] + prox_values[2]) / 7.;

        double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
        double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;



        // increment counters
        if (state == EXPLORER){
          if ((abs(prox_right) > THRESH_PROX_HI) || (abs(prox_left) > THRESH_PROX_HI)){

              if(counter < 1){
                int wall_color = color();
                printf("%d\n", wall_color);
                if (wall_color == 1) {
                  disable_rgbled(0);
                  disable_rgbled(1);
                  enable_rgbled(2, 0xff0000);
                  enable_rgbled(3, 0xff0000);
                  printf("Wall is red\n");
                } else if (wall_color == 2) {
                  disable_rgbled(0);
                  enable_rgbled(1, 0x00ff00);
                  enable_rgbled(2, 0x00ff00);
                  disable_rgbled(3);
                  printf("Wall is green\n");
                } else if (wall_color == 3) {
                  enable_rgbled(0, 0x0000ff);
                  enable_rgbled(1, 0x0000ff);
                  disable_rgbled(2);
                  disable_rgbled(3);
                  printf("Wall is blue\n");
                } else if (wall_color == 0) {
                  disable_rgbled(0);
                  disable_rgbled(1);
                  disable_rgbled(2);
                  disable_rgbled(3);
                  printf("Wall is white\n");
                }
                counter++;
              }
              counter_expl++;
            }
        } else if (state == AVOID){
            if ((abs(prox_right) < THRESH_PROX_LO) && (abs(prox_left) < THRESH_PROX_LO)) {
                counter_expl++;
            }
        }
        // state machine
        if (state == EXPLORER) {
            if (counter_expl > COUNT_EXP) {
              state = AVOID;
              counter_expl = 0;
            }
        } else if ((state == AVOID) && (counter_expl > COUNT_EXP)) {
              counter_expl = 0;
              counter_obstacle++;
              counter = 0;
              state = EXPLORER;
        }
        double speed_left = bounded_speed(NORM_SPEED - ds_right);
        double speed_right = bounded_speed(NORM_SPEED - ds_left);
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
  struct color {int nb; char name[6];};
  struct color redpx = {0, "RED"};
  struct color greenpx = {0, "GREEN"};
  struct color bluepx = {0, "BLUE"};
  get_camera(red, green, blue);
  for (int n = 55; n < 64; n++) {
    for (int m = 0; m < CAMERA_WIDTH; m++) {
      int redpixel = (int) red[n * CAMERA_WIDTH + m];
      int greenpixel = (int) green[n * CAMERA_WIDTH + m];
      int bluepixel = (int) blue[n * CAMERA_WIDTH + m];
      if(redpixel > 150){
        redpx.nb++;
      }
      if (greenpixel > 150){
        greenpx.nb++;
      }
      if (bluepixel > 150) {
        bluepx.nb++;
      }
    }
  }
  printf("%d, %d, %d\n", redpx.nb, greenpx.nb, bluepx.nb);
  if (redpx.nb > 1000 && greenpx.nb < 1000 && bluepx.nb < 1000) {
    return RED;
  } else if (redpx.nb < 1000 && greenpx.nb > 1000 && bluepx.nb < 1000) {
    return GREEN;
  } else if (redpx.nb < 1000 && greenpx.nb < 1000 && bluepx.nb > 1000) {
    return BLUE;
  } else if(redpx.nb > 1000 && greenpx.nb > 1000 && bluepx.nb > 1000) {
    return 0;
  }
  return 0;
}
