#include "ev3api.h"
#include "libcpp-test.h"
#include "app.h"
#include <stdio.h>
#include <stdlib.h>

const sensor_port_t gyro = EV3_PORT_1;

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
    motor_stop();
}

void ramping_cnts(int counts, int speed) {
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    int ct_diff;
    while (counts - ev3_motor_get_counts(left_motor) > 0) {
        if (ev3_motor_get_counts(left_motor) < 50) {
            ct_diff = 50 - ev3_motor_get_counts(left_motor);
            ev3_motor_steer(left_motor, right_motor, (5 + ((ct_diff/2) * (counts/50))), 0);
        } else if (ev3_motor_get_counts(left_motor) > (counts - 50)) {
            ct_diff = counts - ev3_motor_get_counts(left_motor);
            ev3_motor_steer(left_motor, right_motor, (5 + ((ct_diff/2) * (counts/50))), 0);
        } else {
            ev3_motor_steer(left_motor, right_motor, speed, 0);
        }
    }
    motor_stop();
}

void gyro_turns(int angle) {
    const float KP = 0.15;
    const float KD = 0.15;
    const float KI = 0.0001;
    int integral = 0;
    int error = 0;
    int last_error = 0;
    int derivative = 0;
    int power = 0;
    int ng_power = 0;
    ev3_gyro_sensor_reset(gyro);
    char lcdstr[100];
    while (abs(ev3_gyro_sensor_get_angle(gyro)) < abs(angle) - 2) {
        sprintf(lcdstr, "%4d degrees", abs(ev3_gyro_sensor_get_angle(gyro)));
	    ev3_lcd_draw_string(lcdstr, 10, 10);
        error = abs(angle) - abs(ev3_gyro_sensor_get_angle(gyro));
        integral = integral + error;
        derivative = error - last_error;
        power = (error * KP) + (integral * KI) + (derivative * KD);
        if (abs(ev3_gyro_sensor_get_angle(gyro)) < 30) {
            ng_power = (30 - abs(ev3_gyro_sensor_get_angle(gyro))) / 2;
        } else {
            ng_power = 0;
        }
        if (angle > 0) {
            ev3_motor_steer(left_motor, right_motor, power + 2, -100);
        } else {
            ev3_motor_steer(left_motor, right_motor, power + 2, 100);
        }
    }
    motor_stop();
}

void pid_gyro() {
    const float KP = 0.2;
    float KI = 0;
    int intergral = 0;
    const float KD = 0.15;
    int last_error = 0;
    int derivative = 0;
    float Turn = 0;
    int error = 0;
    int gyro_sensor = 0;
    int i = 5;
    while (true) {
        gyro_sensor = ev3_gyro_sensor_get_angle(gyro);
        error = gyro_sensor;
        intergral = intergral + error;
        derivative = error - last_error;
        Turn = (error * KP) + (KI * intergral) + (KD * derivative);
        if (i < 80) {
            i += 1;
        }
        ev3_motor_steer(left_motor,right_motor,i,Turn);
    }     
}
