
#define SIMULATION 1

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

char rcv[MSG_LENGTH];
char rota[15];
char x_str[15];
char y_str[15];
float rotation, x, y;
float alpha;
float *ptr = &alpha;
WbDeviceTag emitter,receiver;


void robot_setup() {
    init_robot();
    init_sensors();
    init_communication();
    init_accelerometer();
}

void robot_loop() {
    int id = get_robot_ID();
    short int prox_values[8];
    // TODO : the API to include these modifications -> init_boids() ???
    receiver = wb_robot_get_device("receiver");
    wb_receiver_set_channel(receiver, id + 1);
    receiver = wb_robot_get_device("receiver");
    int r = wb_receiver_get_channel(receiver);
    get_orientation(ptr);


    while (robot_go_on()) {
        receive_msg(rcv);
        char token = rcv[0];

        if(token == 'r'){
          strncpy(rota, rcv + 1, strlen(rcv) - 1);
          rotation = strtof(rota, NULL);
          printf("id = %d r = %f %f\n", id, rotation, alpha);
        } else if (token == 'x') {
          strncpy(x_str, rcv + 1, strlen(rcv) - 1);
          x = strtof(x_str, NULL);
          printf("id = %d x = %f\n", id, x);
        } else if (token == 'y') {
          strncpy(y_str, rcv + 1, strlen(rcv) - 1);
          y = strtof(y_str, NULL);
          printf("d = %d y = %f\n", id, y);
        }

        // fetch proximity sensor values
        get_prox_calibrated(prox_values);

        // basic explorer behavior
        double prox_right = (1*prox_values[0] + 2*prox_values[1] + 2*prox_values[2] + 1*prox_values[3]) / 4.;
        double prox_left = (1*prox_values[7] + 2*prox_values[6] + 2*prox_values[5] + 1*prox_values[4]) / 4.;
        double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;
        double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
        double speed_right = bounded_speed(NORM_SPEED - ds_right + rotation);
        double speed_left = bounded_speed(NORM_SPEED - ds_left + rotation);

        // make sure speed values are legal
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
