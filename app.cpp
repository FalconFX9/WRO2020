/**
 * Line follower testing to check if ev3rt is adequate.
 *
 * Author: Aruther Goetzgay-Cockburn
 */

#include "ev3api.h"
#include "libcpp-test.h"
#include "app.h"
#include <stdio.h>
#include <stdlib.h>
#define ClearTimer(...)
#define ClearTimerMS(...)

SYSTIM TimerMS(int unused) {
    SYSTIM tim;
    get_tim(&tim);
    return tim;
}

//this is an edit
//this is also an edit
//this is an extra edit and it is an edit

const sensor_port_t g = EV3_PORT_1, s1 = EV3_PORT_1, s2 = EV3_PORT_2;
const motor_port_t left_motor = EV3_PORT_B, right_motor = EV3_PORT_C, lift_motor = EV3_PORT_A, grab_motor = EV3_PORT_D;
FILE *bt = ev3_serial_open_file(EV3_SERIAL_BT);

class LineFollower {
    public:
        float V_diff = (9100 - ev3_battery_voltage_mV()) / 1000;
        const int TARGET = 40;
        //KP was 0.3, KD was 0.12 * V_diff
        const float KP = 0.09;
        const float KD = 0.06;
        float KI = 0.0;
        int DEFAULT_SPEED = 70;
        int last_error;
        int derivative;
        int integral;
        int error;
        double last_steering;
        double steering;
        unsigned long time1;
        bool off_line;

    void follower(int sensor_value, int side = 1) {
        error = sensor_value - TARGET;
        
        if (abs(error) > 25 && !off_line) {
            time1 = TimerMS(0);
        } else if (abs(error) < 25 && off_line) {
            KI = 0;
            off_line = false;
        }
        if ((TimerMS(0) - time1) > 50) {
            KI = 0.25;
            off_line = true;
        }
        
        derivative = error - last_error;
        integral = error + integral;
        steering = ((error * KP) + (derivative * KD) + (integral * 0)) * side;
        if (abs((steering - last_steering)) > 15) {
            steering = 0;
        }
        /*
        if (steering < 0) {
            fprintf(bt, "%.2f", steering);
        } else {
            fprintf(bt, "%.2f+", steering);
        }
        */
        ev3_motor_steer(left_motor, right_motor, DEFAULT_SPEED, steering);
        last_error = error;
        last_steering = steering;
    }

    void two_sensor(int s1_value, int s2_value) {
        error = s1_value - s2_value;
        derivative = error - last_error;
        integral = error + integral;
        steering = (error * KP) + (derivative * KD) + (integral * KI) ;
        ev3_motor_steer(left_motor, right_motor, DEFAULT_SPEED, steering);
        last_error = error;
    }
};

void motor_stop() {
    ev3_motor_stop(left_motor, true);
    ev3_motor_stop(right_motor, true);
}

void on_for_counts(int counts, int power) {
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    while ((counts - abs(ev3_motor_get_counts(left_motor)) > 0) && (counts - abs(ev3_motor_get_counts(right_motor)) > 0)) {
        ev3_motor_steer(left_motor, right_motor, power, 0);
    }
    motor_stop();
}

void follow_for_counts(int counts, int option = 0, int side = 1) {
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

void follow_to_line(int option = 0, int side = 1) {
    if (option == 1) {
        while (ev3_color_sensor_get_reflect(s1) > 30) {
            int sensor_value = ev3_color_sensor_get_reflect(s2);
            LineFollower().follower(sensor_value, side);
        }
    } else {
        while (ev3_color_sensor_get_reflect(s2) > 30) {
            int sensor_value = ev3_color_sensor_get_reflect(s1);
            LineFollower().follower(sensor_value, side);
        }
    }
    motor_stop();
}

void follow_for_lines(int lines, int side = 1, int option = 0) {
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

void sleep(unsigned long ms) {
    unsigned long time0;
    ClearTimerMS(0);
    ClearTimer(0);
    time0 = TimerMS(0);
    while ((TimerMS(0) - time0) < ms) {  
    }
}

void lower() {
    ev3_motor_rotate(lift_motor, -175, 30, true);
}

void open() {
    ev3_motor_rotate(grab_motor, -460, 30, false);
}

void lift() {
    ev3_motor_rotate(lift_motor, 178, 10, false);
}

void close() {
    ev3_motor_rotate(grab_motor, -80, 30, true);
}

void turn_left_gyro() {
    ev3_gyro_sensor_reset(gyro);
    while (abs(ev3_gyro_sensor_get_angle(gyro)) < 90) {
        ev3_motor_steer(left_motor, right_motor, 30, -100);
    }
    motor_stop();
}

void main_task(intptr_t unused) {
    ev3_sensor_config(gyro, GYRO_SENSOR);
    ev3_gyro_sensor_reset(gyro);
    go_to_line();
    if (ev3_battery_voltage_mV() < 9500) {
        ev3_speaker_play_tone(500, 500);
    }
    //ev3_sensor_config(s1, COLOR_SENSOR);
    //ev3_sensor_config(s2, COLOR_SENSOR);
    // ev3_sensor_config(hitechnic, HT_NXT_COLOR_SENSOR);
    ev3_motor_config(left_motor, MEDIUM_MOTOR);
    ev3_motor_config(right_motor, MEDIUM_MOTOR);
    ev3_motor_config(lift_motor, LARGE_MOTOR);
    ev3_motor_config(grab_motor, MEDIUM_MOTOR);
    //ev3_motor_steer(left_motor, right_motor, 80, 0);
    //pid_gyro();
    ev3_motor_rotate(lift_motor, 20, 20, true);
    on_for_counts(300, 20);
    gyro_turns(-18);
    ev3_motor_rotate(grab_motor, 80, 40, true);
    sleep(1000);
    on_for_counts(85, -20);
    sleep(1000);
    gyro_turns(36);
    sleep(1000);
    gyro_turns(-18);
    ev3_motor_rotate(grab_motor, 180, 50, true);
    on_for_counts(200, 20);
    on_for_counts(235, -20);
    ev3_motor_rotate(lift_motor, 115, 30, true);
    on_for_counts(150, -20);
    lower();
    

    /*
    lower();
    on_for_counts(240, 20);
    ev3_motor_rotate(grab_motor, -110, 30, true);
    lift();
    sleep(200);
    gyro_turns(-90);
    on_for_counts(380, 20);
    gyro_turns(-90);
    ev3_speaker_play_tone(500, 500);


    stop_at_YR();
    ramping_right();
    align();
    int i = 0;
    while (true) {
        follow_for_lines(4, 1, 1);
        on_for_counts(95);
        turn_left(1);
        i++;
        tslp_tsk(150);
    }
    */
}
