#include "ev3api.h"
#include "libcpp-test.h"
#include "app.h"
#include <stdio.h>
#include <stdlib.h>
#define ClearTimer(...)
#define ClearTimerMS(...)

void ramp_motors(int speed) {
    //Ramps up the motors over 500ms to the desired speed
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
    //Slows down the motors over 500ms from the initial speed
    unsigned long time0 = TimerMS(0);
    ClearTimer(0);
    ClearTimerMS(0);
    int power = 0;
    while ((TimerMS(0) - time0) < 500) {
        power = (500 - (TimerMS(0) - time0)) * (speed/500);
        ev3_motor_steer(left_motor, right_motor, (power), 2);
    }
}

void align(int speed, colorid_t color) {
    //Aligns the robot against a black line (line squarring)
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
    //Uses a PID controller for more "effective" turns. The turn function is preferred over this one.
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

/*
void follow_for_counts(int counts, int option = 0, int side = 1) {
    //Follows the line for specified counts
    //Option changes what sensor is used, while side changes the side of the line the sensor will follow.
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    if (option == 1) {
        while (counts - ev3_motor_get_counts(left_motor) > 0) {
            int sensor_value = ev3_color_sensor_get_reflect(s2);
            LineFollower().follower(sensor_value, side);
        }
    } else if (option == 2 ) {
        while (counts - ev3_motor_get_counts(left_motor) > 0) {
            int s1_value = ev3_color_sensor_get_reflect(s1);
            int s2_value = ev3_color_sensor_get_reflect(s2);
            LineFollower().two_sensor(s1_value, s2_value);
        } 
    } else {
        while (counts - ev3_motor_get_counts(left_motor) > 0) {
            int sensor_value = ev3_color_sensor_get_reflect(s1);
            LineFollower().follower(sensor_value, side);
        }
    }
}

void follow_for_lines(int lines, int side = 1, int option = 0) {
    //Follows a line for lines amoung of lines, along the specified side of the line, using the sensor designated by option.
    unsigned long time0;
    int lines_seen = 0;
    int s1_value, s2_value;
    ClearTimerMS(0);
    ClearTimer(0);
    time0 = TimerMS(0);
    while (lines_seen < lines) {
        if (option == 1) {
            s2_value = ev3_color_sensor_get_reflect(s1);
            s1_value = ev3_color_sensor_get_reflect(s2);
        } else {
            s1_value = ev3_color_sensor_get_reflect(s1);
            s2_value = ev3_color_sensor_get_reflect(s2);
        }
        LineFollower().follower(s1_value, side);
        if (s2_value < 30 && (TimerMS(0) - time0) > 75) {
            //if (lines_seen != lines - 1) {
            //    follow_for_counts(50, 0, side);
            //}
            lines_seen++;
            time0 = TimerMS(0);
        }
    }
    motor_stop();
}
*/
