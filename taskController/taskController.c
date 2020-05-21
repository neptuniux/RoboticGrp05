//àééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééééé
// Created by daniel on 23.04.20.
//


#define SIMULATION 1

#if SIMULATION

#include "../API/webots/webotsAPI.h"

#else
#include "../API/epuck/epuckAPI.h"
#endif

//define states
#define FIND 0
#define LINE 1
#define WALL 2
#define AVOID 3
#define STOP 4

//Parameters for line PID
#define K_LINE 0.01
#define T_I_LINE 100
#define T_D_LINE 0.05
#define LINE_GS_TARGET 490
#define LINE_MAX_DS 490

//Parameters for wall PID
#define K_WALL 0.01
#define T_I_WALL 1000
#define T_D_WALL 0.1
#define PID_WALL_FOLLOW_TARGET 200
#define PID_MAX_DS 200

#define NONE 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define EPUCK 4

//define for communications
/*number of robots on the field*/
#define NB_ROBOT 3

int send_id(int), send(char[]);

#define MSG_LENGTH 5

int robot[NB_ROBOT];
int robot_count = 0;
int sent = 0;
char rcv[MSG_LENGTH];
char tmp[MSG_LENGTH];

int color();

void led_on(int), led_off(), init_rgbled(), phase_led(int);


double error_line = 0;
double deriv_line = 0;
double integ_line = 0;

double error_wall = 0;
double deriv_wall = 0;
double integ_wall = 0;


double pid_line(double delta) {

    double prev_err = error_line;
    error_line = delta - LINE_GS_TARGET;
    //deriv = (error - prev_err);
    deriv_line = (error_line - prev_err) * 1000 / TIME_STEP;
    //integ_line += error;
    integ_line += error_line * TIME_STEP / 1000;
    return K_LINE * (error_line + 1.0 / T_I_LINE * integ_line + T_D_LINE * deriv_line);
}

double pid_wall(double delta) {
    double prev_err = error_wall;
    error_wall = delta - 200;
    //deriv = (error - prev_err);
    deriv_wall = (error_wall - prev_err) * 1000 / TIME_STEP;
    //integ_line += error;
    integ_wall += error_wall * TIME_STEP / 1000;
    return K_WALL * (error_wall + 1.0 / T_I_WALL * integ_wall + T_D_WALL * deriv_wall);
}


void robot_setup() {
    init_robot();
    init_sensors();
    init_communication();
    calibrate_prox();
    init_camera();
    init_rgbled();
}


void robot_loop() {
    int id = get_robot_ID();
    short int IR_ground[GROUND_SENSORS_COUNT];
    short int prox_values[8];
    
    int ack = 0;
    int phase = 0;
    int loop_counter = 0;
    int STATE = FIND;
    int counter = 0;
    int current_wall = NONE;
    int line_crossed = 0;

    double speed_right = 0;
    double speed_left = 0;

    while (robot_go_on()) {
        loop_counter++;

        //control that robot_count doesn't go over the array size
        if (robot_count >= NB_ROBOT) {
            robot_count = 0;
       }
        receive_msg(rcv);
        int rcv_id = rcv[0] - 48;
        if (rcv_id <= NB_ROBOT) {
            robot[robot_count] = rcv_id;
            robot_count++;
            ack++;
            printf("[0]:%d, [1]: %d, [2]: %d, count =%d, %d\n",robot[0], robot[1], robot[2], robot_count, ack);
        } else if (sent == 1) {
            sent = 0;
            robot[robot_count] = id;
            robot_count++;
            ack++;
             printf("[0]:%d, [1]: %d, [2]: %d, count =%d, %d\n",robot[0], robot[1], robot[2], robot_count, ack);
        }

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
                enable_led(0);
                if (prox >= MAX_PROX) {
                    STATE = WALL;
                } else if (gs > 470 && gs < 500) {
                    // the values of the sensors is betwen the detection error, to reduce shackiness
                    ds = 0;
                } else {
                    ds = pid_line(gs);
                }
                speed_right = bounded_speed((abs(ds) > (PID_MAX_DS) ? 0 : NORM_SPEED) - ds);
                speed_left = bounded_speed((abs(ds) > (PID_MAX_DS) ? 0 : NORM_SPEED) + ds);
                set_speed(speed_left, speed_right);
                break;

            case WALL:

                speed_right = bounded_speed(NORM_SPEED - ds_prox);
                speed_left = bounded_speed(NORM_SPEED - ds_prox);

                //this if is used to rotate the robot in front of a wall
                if (current_wall == RED) {
                    if (counter < 16) {
                        speed_left = bounded_speed(-NORM_SPEED);
                        speed_right = bounded_speed(NORM_SPEED);
                        counter++;
                    } else {
                        counter = 0;
                        STATE = AVOID;
                    }
                } else if (current_wall == BLUE) {
                    if (counter < 16) {
                        speed_left = bounded_speed(NORM_SPEED);
                        speed_right = bounded_speed(-NORM_SPEED);
                        counter++;
                    } else {
                        counter = 0;
                        STATE = AVOID;
                    }
                } else {

                    //if the robot see the wall for the first time
                    switch (color()) {
                        case RED:
                            led_on(RED);
                            current_wall = RED;
                            break;
                        case GREEN:
                            led_on(GREEN);
                            //if the robot is directly in front of a green wall
                            STATE = STOP;
                            sent = send_id(id);
                            if(phase == 3){
                              phase_led(2);
                            }
                            break;
                        case BLUE:
                            led_on(BLUE);
                            current_wall = BLUE;
                            break;
                        case EPUCK:
                            led_off();
                            sent = send_id(id);
                            STATE = STOP;
                            if(phase == 3){
                              phase_led(3);
                            }
                            break;
                        default:
                          break;

                    }
                }
                set_speed(speed_left, speed_right);
                break;

            //stop when the e-puck sees a green wall or another e-puck


            case AVOID:
                //if the robot is back on a line
                if (gs < 350 || line_crossed == 1) {
                    if (counter < 10) {
                        line_crossed = 1;
                        //turn one direction or an other depending of the color
                        if (current_wall == RED) {
                            speed_left = bounded_speed(-NORM_SPEED);
                            speed_right = bounded_speed(NORM_SPEED);
                        } else if (current_wall == BLUE) {
                            speed_left = bounded_speed(NORM_SPEED);
                            speed_right = bounded_speed(-NORM_SPEED);
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
                        ds = pid_wall(prox_right);
                        speed_right = (abs(ds) > 200 ? 0 : NORM_SPEED) + ds;
                        speed_left = (abs(ds) > 200 ? 0 : NORM_SPEED) - ds;
                    } else if (current_wall == BLUE) {
                        ds = pid_wall(prox_left);
                        speed_right = (abs(ds) > 200 ? 0 : NORM_SPEED) - ds;
                        speed_left = (abs(ds) > 200 ? 0 : NORM_SPEED) + ds;
                    }
                }
                set_speed(bounded_speed(speed_left), bounded_speed(speed_right));
                break;

            case STOP:
              set_speed(0, 0);
              
              if (ack >= 3){
                  phase_led(1);
                if (prox < MAX_PROX/3) {
                  STATE = LINE;
                  phase = 3;
                  sent = send_id(id);
                }
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
    if (redpx > 100 && bluepx > 100 && greenpx > 100) {
        printf("%d %d %d\n", redpx, greenpx, bluepx);
        return EPUCK;
    }else if (redpx > greenpx && redpx > bluepx) {
        printf("%d %d %d\n", redpx, greenpx, bluepx);
        return RED;
    } else if (greenpx > redpx && greenpx > bluepx) {
        printf("%d %d %d\n", redpx, greenpx, bluepx);
        return GREEN;
    } else if (bluepx > redpx && bluepx > greenpx) {
        printf("%d %d %d\n", redpx, greenpx, bluepx);
        return BLUE;
    }
    return NONE;
}

void led_on(int color) {
    switch (color) {
        case RED:
            for (int i = 0; i < 4; i++) {
                disable_rgbled(i);
                enable_rgbled(i, 0xff0000);
                disable_led(i);
            }
            enable_led(0);
            enable_led(1);
            enable_led(2);

            break;
        case GREEN:
            for (int i = 0; i < 4; i++) {
                disable_rgbled(i);
                enable_rgbled(i, 0x00ff00);
                disable_led(i);
            }
            break;
        case BLUE:
            for (int i = 0; i < 4; i++) {
                disable_rgbled(i);
                enable_rgbled(i, 0x0000ff);
                disable_led(i);
            }
            enable_led(0);
            enable_led(2);
            enable_led(3);
            break;
    }
}

void led_off() {
    for (int i = 0; i < 4; i++) {
        disable_rgbled(i);
        disable_led(i);
    }
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
void phase_led(int type) {
  for (int i = 0; i < 4; i++) {
    disable_led(i);
   }
   switch(type){
     case 1 :
       enable_led(1);
       enable_led(2);
       enable_led(3);
       break;
     case 2:
       enable_led(0);
       enable_led(1);
       enable_led(3);
       break;
     case 3:
       enable_led(1);
       enable_led(3);
       break;
   }
}

/*
Si communication

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

counteur arret au 2eme vert

int greenwall = 0; (on rajoute ensuite dans green un greenwall ++)

    while(greenwall == 2) {
        stop; (ou set speed 0)
    }
(while dans chaque autre cas)

senseur detecte qqch n etant aucun des murs devant (le premier robot à l arret)  se stop donc automatiquement  et ainsi fait sotp les 3 robots ensuite le while stop la simu pour le 2eme mur
color


*/
