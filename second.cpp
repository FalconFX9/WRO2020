#include "ev3api.h"
#include "libcpp-test.h"
#include "app.h"
#include <stdio.h>
#include <stdlib.h>
#define ClearTimer(...)
#define ClearTimerMS(...)

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

void ramp_motors(int speed) {
    unsigned long time0 = TimerMS(0);
    ClearTimer(0);
    ClearTimerMS(0);
    int power = 0;
    while ((TimerMS(0) - time0) < 500) {
        power = (500 - (TimerMS(0) - time0)) * ((speed - 10)/500);
        ev3_motor_steer(left_motor, right_motor, (speed - power), 2);
    }
}

void brake_motors(int speed) {
    unsigned long time0 = TimerMS(0);
    ClearTimer(0);
    ClearTimerMS(0);
    int power = 0;
    while ((TimerMS(0) - time0) < 500) {
        power = (500 - (TimerMS(0) - time0)) * (speed/500);
        ev3_motor_steer(left_motor, right_motor, (power), 2);
    }
}

void ramping_cnts(int counts, int speed) {
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    int ct_diff;
    while (counts - ev3_motor_get_counts(left_motor) > 0) {
        if (ev3_motor_get_counts(left_motor) < 100) {
            ct_diff = 50 - ev3_motor_get_counts(left_motor);
            ev3_motor_steer(left_motor, right_motor, (5 + ((ct_diff/2) * (counts/100))), 0);
        } else if (ev3_motor_get_counts(left_motor) > (counts - 100)) {
            ct_diff = counts - ev3_motor_get_counts(left_motor);
            ev3_motor_steer(left_motor, right_motor, (5 + ((ct_diff/2) * (counts/100))), 0);
        } else {
            ev3_motor_steer(left_motor, right_motor, speed, 0);
        }
    }
    motor_stop();
}

void align(int speed, colorid_t color) {
    ev3_motor_set_power(left_motor, speed);
    ev3_motor_set_power(right_motor, speed);
    bool s1_c = true, s2_c = true;
    while (s1_c || s2_c) {
        if (ev3_color_sensor_get_color(s1) == color) {
            ev3_motor_stop(left_motor, true);
            s1_c = false;
        }
        if (ev3_color_sensor_get_color(s2) == color) {
            ev3_motor_stop(right_motor, true);
            s2_c = false;
        }
    }
}

void gyro_turns(int angle) {
    const float KP = 0.1;
    const float KD = 0.1;
    const float KI = 0.0001;
    int integral = 0;
    int error = 0;
    int last_error = 0;
    int derivative = 0;
    int power = 0;
    int start_angle = ev3_gyro_sensor_get_angle(gyro);
    while (abs(ev3_gyro_sensor_get_angle(gyro) - start_angle) < (abs(angle) - 2)) {
        error = abs((angle + start_angle) - ev3_gyro_sensor_get_angle(gyro));
        fprintf(bt, "%d\n", error);
        integral = integral + error;
        derivative = error - last_error;
        power = (error * KP) + (integral * KI) + (derivative * KD);
        if (angle < 0) {
            ev3_motor_steer(left_motor, right_motor, power + 2, -50);
        } else {
            ev3_motor_steer(left_motor, right_motor, power + 2, 50);
        }
    }
    motor_stop();
}
