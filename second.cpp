#include "ev3api.h"
#include "libcpp-test.h"
#include "app.h"

void go_to_line() {
    while (ev3_color_sensor_get_reflect(s1) > 30) {
        ev3_motor_steer(left_motor, right_motor, 50, 0);
    }
}

void stop_at_YR() {
    while (ev3_color_sensor_get_color(s1) != COLOR_WHITE && ev3_color_sensor_get_color(s2) != COLOR_WHITE){
        ev3_motor_steer(left_motor, right_motor, 40, 0);
    }
    while (ev3_color_sensor_get_color(s1) != COLOR_YELLOW && ev3_color_sensor_get_color(s2) != COLOR_RED) {
        ev3_motor_steer(left_motor, right_motor, 40, 0);
    } 
    motor_stop();
}

void turn_left(int option) {
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    while (250 - ev3_motor_get_counts(right_motor) > 0) {
        ev3_motor_steer(left_motor, right_motor, 30, -100);
    }
    if (option == 1) {
        while (ev3_color_sensor_get_reflect(s2) > 30) {
            ev3_motor_steer(left_motor, right_motor, 20, -100);
        }
    } else {
        while (ev3_color_sensor_get_reflect(s1) > 30) {
            ev3_motor_steer(left_motor, right_motor, 20, -100);
        }
    }
    motor_stop();
}

void turn_right(int option) {
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    while (225 - ev3_motor_get_counts(left_motor) > 0) {
        ev3_motor_steer(left_motor, right_motor, 30, 100);
    }
    if (option == 1) {
        while (ev3_color_sensor_get_reflect(s2) > 30) {
            ev3_motor_steer(left_motor, right_motor, 20, 100);
        }
    } else {
        while (ev3_color_sensor_get_reflect(s1) > 30) {
            ev3_motor_steer(left_motor, right_motor, 20, 100);
        }
    }
    motor_stop();
}

void align() {
    while (ev3_color_sensor_get_reflect(s1) > 30 || ev3_color_sensor_get_reflect(s2) > 30) {
        ev3_motor_steer(left_motor, right_motor, 10, 0);
    }
    motor_stop();
    on_for_counts(20, -20);
    while (ev3_color_sensor_get_reflect(s1) > 30) {
        ev3_motor_steer(left_motor, right_motor, 15, 50);
    }
    motor_stop();
    while (ev3_color_sensor_get_reflect(s2) > 30) {
        ev3_motor_steer(left_motor, right_motor, 15, -50);
    }
    motor_stop;
}

void ramping_right() {
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    int ct_diff;
    while (290 - ev3_motor_get_counts(left_motor) > 0) {
        if (ev3_motor_get_counts(left_motor) < 50) {
            ct_diff = 50 - ev3_motor_get_counts(left_motor);
            ev3_motor_steer(left_motor, right_motor, (5 + ct_diff/2), 100);
        } else if (ev3_motor_get_counts(left_motor) > 240) {
            ct_diff = 290 - ev3_motor_get_counts(left_motor);
            ev3_motor_steer(left_motor, right_motor, (5 + ct_diff/2), 100);
        } else {
            ev3_motor_steer(left_motor, right_motor, 30, 100);
        }
    }
    motor_stop();
}
