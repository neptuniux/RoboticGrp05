//
// Created by daniel on 23.04.20.
//


#define SIMULATION 1

#if SIMULATION

#include "../API/webots/webotsAPI.h"

#else
#include "../API/epuck/epuckAPI.h"
#endif
void stop(); 
//define states


#define FIND 0
#define LINE 1
#define WALL 2
#define AVOID 3

#define MAX_GROUND 800


#define MIN_GROUND 300
#define K 0.009

#define T_I 100
#define T_D 0.001
#define PID_DELTA_TARGET 490
#define PID_MAX_DS 490
#define NONE 0

#define RED 1
#define GREEN 2
#define BLUE 3
#define WHITE 4


/*number of robots on the field*/
#define NB_ROBOT 3  

int send_id(int), send(char[]);
#define MSG_LENGTH 5

int robot[NB_ROBOT];
int robot_count=0;
int ack = 0;
int go = 0;
int sent = 0;
int SAW_OBSTACLE = 0;
int HAS_STOPPED = 0;
char rcv[MSG_LENGTH];
char tmp[MSG_LENGTH];



int color();

void led_on(int), led_off(), init_rgbled();


double error_line = 0;
double deriv_line = 0;
double integ_line = 0;

double error_wall = 0;
double deriv_wall = 0;
double integ_wall = 0;


double pid_line(double delta) {

    double prev_err = error_line;
    error_line = delta - PID_DELTA_TARGET;
    //deriv = (error - prev_err);
    deriv_line = (error_line - prev_err) * 1000 / TIME_STEP;
    //integ_line += error;
    integ_line += error_line * TIME_STEP / 1000;
    return K * (error_line + 1.0 / T_I * integ_line + T_D * deriv_line);
}

double pid_wall(double delta) {
    double prev_err = error_wall;
    error_wall = delta - 200;
    //deriv = (error - prev_err);
    deriv_wall = (error_wall - prev_err) * 1000 / TIME_STEP;
    //integ_line += error;
    integ_wall += error_wall * TIME_STEP / 1000;
    return 0.015 * (error_wall + 1.0 / 1000 * integ_wall + 0.10 * deriv_wall);
}


void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
    init_camera();
    init_rgbled();
    init_communication();
    // initialize robot[] to 0
    for (int r = 0; r < NB_ROBOT; r++) {
      robot[r] = 0;
    }
}


void robot_loop() {
    int id = get_robot_ID();
    short int IR_ground[GROUND_SENSORS_COUNT];
    short int prox_values[8];

    int loop_counter = 0;
    int STATE = FIND;

    int counter = 0;
    int current_wall = NONE;

    int line_crossed = 0;

    double speed_right = 0;
    double speed_left = 0;

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
        
        loop_counter++;
        loop_counter++;
        double ds = 0;
        get_ground(IR_ground);
        get_prox_calibrated(prox_values);
        double prox = (prox_values[0] + prox_values[7]) / 2.0;
        double ds_prox = (NORM_SPEED * prox) / MAX_PROX;
        double gs = (IR_ground[GS_RIGHT] + IR_ground[GS_CENTER] + IR_ground[GS_LEFT]) / 3.0;

        double a = 1, b = 5, c = 5, d = 0;
        double prox_right =
                (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a + b + c + d);
        double prox_left =
                (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a + b + c + d);

        switch (STATE) {
            case FIND:
                led_off();
                if (gs < 350) {
                    STATE = LINE;
                } else {
                    set_speed(NORM_SPEED, NORM_SPEED);
                }
                break;

            case LINE:
                led_off();
                if (prox >= MAX_PROX) {
                    STATE = WALL;
                } else if (gs > 475 && gs < 495) {
                    // the values of the sensors is betwen the detection error, to reduce shackiness
                    ds = 0;
                } else {
                    ds = pid_line(gs);
                }
                speed_right = bounded_speed((abs(ds) > (PID_MAX_DS) ? 0 : NORM_SPEED) + ds);
                speed_left = bounded_speed((abs(ds) > (PID_MAX_DS) ? 0 : NORM_SPEED) - ds);
                set_speed(speed_left, speed_right);
                break;

            case WALL:

                speed_right = bounded_speed(NORM_SPEED - ds_prox);
                speed_left = bounded_speed(NORM_SPEED - ds_prox);
                
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
                }

                if (current_wall == RED) {
                    if (counter < 16) {
                        speed_left = bounded_speed(NORM_SPEED);
                        speed_right = bounded_speed(-NORM_SPEED);
                        counter++;
                    } else {
                        counter = 0;
                        STATE = AVOID;
                    }
                } else if (current_wall == BLUE) {
                    if (counter < 16) {
                        speed_left = bounded_speed(-NORM_SPEED);
                        speed_right = bounded_speed(NORM_SPEED);
                        counter++;
                    } else {
                        counter = 0;
                        STATE = AVOID;
                    }
                    } else if (current_wall == GREEN) {                    
                          if(go == 1){
                          robot_count = 0;
                          ack = 0;
                          go = 0;
                          stop();
                    } else { 
                      if(go == 1){
                      robot_count = 0;
                      ack = 0;
                      go = 0;
                      stop();
                   
                   
                } else {

                    switch (color()) {
                        case RED:
                            led_on(RED);
                            printf("Wall is red\n");
                            current_wall = RED;
                            break;
                        case GREEN:
                            led_on(GREEN);
                          if(STATE == STOP){
                            if(go == 1){
                            robot_count = 0;
                            ack = 0;
                            go = 0;
                              for (int r = 0; r < NB_ROBOT; r++) {
                                robot[r] = 0;
                              }
                            }
                          }
                            
                            if (prox >= MAX_PROX) {    
                            } else {
                                STATE = FIND;
                            }
                            break;
                        case BLUE:
                            led_on(BLUE);
                            printf("Wall is blue\n");
                            current_wall = BLUE;
                            break;
                        case WHITE:
                            led_off();
                            break;
                        case NONE:
                            led_on(NONE);
                            printf("Unknown\n");
                            break;
                            //}
                            //set_speed(0, 0);

                    }
                }
                set_speed(speed_left, speed_right);
                break;

            case AVOID:
                if (gs < 350 || line_crossed == 1) {
                    if (counter < 10) {
                        line_crossed = 1;
                        if (current_wall == RED) {
                            speed_left = bounded_speed(NORM_SPEED);
                            speed_right = bounded_speed(-NORM_SPEED);
                        } else if (current_wall == BLUE) {
                            speed_left = bounded_speed(-NORM_SPEED);
                            speed_right = bounded_speed(NORM_SPEED);
                        }
                        counter++;
                    } else {
                        STATE = FIND;
                        current_wall = NONE;
                        line_crossed = 0;
                        counter = 0;
                    }
                } else {
                    if (current_wall == RED) {
                        ds = pid_wall(prox_left);
                        speed_right = (abs(ds) > 200 ? 0 : NORM_SPEED) - ds;
                        speed_left = (abs(ds) > 200 ? 0 : NORM_SPEED) + ds;
                    } else if (current_wall == BLUE) {
                        ds = pid_wall(prox_right);
                        speed_right = (abs(ds) > 200 ? 0 : NORM_SPEED) + ds;
                        speed_left = (abs(ds) > 200 ? 0 : NORM_SPEED) - ds;
                    }
                }
                set_speed(bounded_speed(speed_left), bounded_speed(speed_right));
                break;
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


int color() {
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
            if (redpixel > 100) {
                redpx++;
            }
            if (greenpixel > 100) {
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
    } else if (redpx == greenpx && redpx == bluepx && greenpx == bluepx) {
        printf("%d %d %d\n", redpx, greenpx, bluepx);
        return WHITE;
    }
    return NONE;
}

void led_on(int color) {
    switch (color) {
        case 1:
            for (int i = 0; i < 4; i++) {
                disable_rgbled(i);
                enable_rgbled(i, 0xff0000);
                disable_led(i);
            }
            enable_led(3);
            break;
        case 2:
            for (int i = 0; i < 4; i++) {
                disable_rgbled(i);
                enable_rgbled(i, 0x00ff00);
                disable_led(i);
            }
            enable_led(2);
            break;
        case 3:
            for (int i = 0; i < 4; i++) {
                disable_rgbled(i);
                enable_rgbled(i, 0x0000ff);
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

void led_off() {
    for (int i = 0; i < 4; i++) {
        disable_rgbled(i);
        disable_led(i);
    }
}

void stop(){
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
