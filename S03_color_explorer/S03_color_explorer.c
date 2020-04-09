
#define SIMULATION 1

#if SIMULATION

#include "../API/webots/webotsAPI.h"

#else
#include "../API/epuck/epuckAPI.h"
#endif

#define NONE 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define WHITE 4
#define PIXEL_THRESHOLD 600

int color();

void led_on(int), led_off(), init_rgbled();

void robot_setup() {
    init_robot();
    init_sensors();
    calibrate_prox();
    init_camera();
    init_rgbled();
}

void robot_loop() {
    short int prox_values[8];
    int count = 0;
    while (robot_go_on()) {

        // fetch proximity sensor values
        get_prox_calibrated(prox_values);

        if (count > 20) {
            int wall_color = color();
            switch (wall_color) {
                case RED:
                    led_on(RED);
                    printf("Wall is red\n");
                    break;

                case GREEN:
                    led_on(GREEN);
                    printf("Wall is green\n");
                    break;

                case BLUE:
                    led_on(BLUE);
                    printf("Wall is blue\n");
                    break;

                case WHITE:
                    led_off();
                    printf("Wall is white\n");
                    break;

                default:
                    led_on(NONE);
                    printf("Unknown\n");
                    break;
            }
            count = 0;
        }

        // basic explorer behavior
        double prox_right = (2 * prox_values[0] + 2 * prox_values[1] + 2 * prox_values[2] + 1 * prox_values[3]) / 5.;
        double prox_left = (2 * prox_values[7] + 2 * prox_values[6] + 2 * prox_values[5] + 1 * prox_values[4]) / 5.;
        double ds_right = (NORM_SPEED * prox_right) / MAX_PROX;
        double ds_left = (NORM_SPEED * prox_left) / MAX_PROX;
        double speed_right = bounded_speed(NORM_SPEED - ds_right);
        double speed_left = bounded_speed(NORM_SPEED - ds_left);

        // make sure speed values are legal
        set_speed(speed_right, speed_left);
        count++;
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

            if (redpixel > 150) { redpx++; }
            if (greenpixel > 150) { greenpx++; }
            if (bluepixel > 150) { bluepx++; }
        }
    }

    printf("%d, %d, %d\n", redpx, greenpx, bluepx);
    if (redpx > greenpx && redpx > bluepx) {
        return RED;
    } else if (greenpx > redpx && greenpx > bluepx) {
        return GREEN;
    } else if (bluepx > redpx && bluepx > greenpx) {
        return BLUE;
    } else if (redpx == greenpx && redpx == bluepx && greenpx == bluepx) {
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
