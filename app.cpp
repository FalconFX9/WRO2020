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

const sensor_port_t s1 = EV3_PORT_3, s2 = EV3_PORT_2, hitechnic = EV3_PORT_4, gyro = EV3_PORT_1;
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
        int DEFAULT_SPEED = 30;
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

class pid_gyro {
    public:
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
        char lcdstr[100];

        void reset_gyro() {
            while (ev3_gyro_sensor_get_angle(gyro) != 0) {
                ev3_gyro_sensor_reset(gyro);
                sleep(0.5);
            }
        }
        
        void follow(int power, int start_angle)  {
            gyro_sensor = ev3_gyro_sensor_get_angle(gyro);
            sprintf(lcdstr, "%4d degrees", gyro_sensor);
	        ev3_lcd_draw_string(lcdstr, 10, 10);
            error = gyro_sensor - start_angle;
            intergral = intergral + error;
            derivative = error - last_error;
            Turn = (error * KP) + (KI * intergral) + (KD * derivative);
            ev3_motor_steer(left_motor,right_motor,power,Turn+2.5);
        }
};

void motor_stop() {
    ev3_motor_stop(left_motor, true);
    ev3_motor_stop(right_motor, true);
}

void on_for_counts(int counts, int power, int brake=1, int angle=0) {
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ramp_motors(power);
    counts -= 50;
    int start_angle = ev3_gyro_sensor_get_angle(gyro);
    if (angle != 0) {
        start_angle = angle;
    }
    while ((counts - abs(ev3_motor_get_counts(left_motor)) > 0) && (counts - abs(ev3_motor_get_counts(right_motor)) > 0)) {
        pid_gyro().follow(power, start_angle);
    }
    if (brake==1) {
        brake_motors(power);
        motor_stop();
    }
    
}

void on_for_counts_motor(int counts, int power, int brake=1) {
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ramp_motors(power);
    counts -= 50;
    int steering = 3;
    if (power < 0) {
        steering = -3;
    }
    while ((counts - abs(ev3_motor_get_counts(left_motor)) > 0) && (counts - abs(ev3_motor_get_counts(right_motor)) > 0)) {
        ev3_motor_steer(left_motor, right_motor, power, steering);
    }
    if (brake==1) {
        brake_motors(power);
        motor_stop();
    }
    
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
        while (ev3_color_sensor_get_color(s1) != COLOR_BLACK) {
            int sensor_value = ev3_color_sensor_get_reflect(s2);
            LineFollower().follower(sensor_value, side);
        }
    } else {
        while (ev3_color_sensor_get_reflect(s2) > 20) {
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
    ev3_motor_rotate(lift_motor, 178, 30, false);
}

void close() {
    ev3_motor_rotate(grab_motor, -80, 30, true);
}

void turn(int angle, int slow=1) {
    /**
    * \Brief Makes the robot turn on the spot using the gyro sensor
    * \param angle is the angle of the turn, positive is right, negative is left
    * \param slow is by default 1, and if the value is changed, the turn will be executed slower for higher precision. It doesn't need to be given a value as it has a default value
    */
    int start_angle = ev3_gyro_sensor_get_angle(gyro);
    int speed = 23;
    if (slow!=1) {
        speed = 13;
    }
    while (abs(ev3_gyro_sensor_get_angle(gyro) - start_angle) < (abs(angle) - 3)) {
        if (angle > 0) {
            ev3_motor_steer(left_motor, right_motor, speed, 100);
        } else {
            ev3_motor_steer(left_motor, right_motor, speed, -100);
        }
    }
    motor_stop();
}

void place_concrete() {
    //Lifts the claw up a bit ove the block to avoid excess friction
    ev3_motor_rotate(lift_motor, 20, 30, false);
    //Goes forward (this line and the line above would be in the code to get the the location, not in the placement code)
    on_for_counts(300, 20);
    //Turns left to offset one concrete block
    gyro_turns(-16);
    //Opens the claw to let go of the blocks
    ev3_motor_rotate(grab_motor, 80, 40, true);
    //Backs up the robot to make the claw only grab the rearmost block
    on_for_counts(85, -20);
    //Turns to offset the second block that was in the claw
    gyro_turns(32);
    sleep(500);
    //Returns to the original position
    gyro_turns(-18);
    //Opens the claw out flat
    ev3_motor_rotate(grab_motor, 150, 50, true);
    //Pushes the concrete blocks next to each other
    on_for_counts(200, 15);
    //Lifts the claw to not hold the block clamped under the claw in place anymore
    ev3_motor_rotate(lift_motor, 115, 30, true);
    //Backs up the robot so the claw passes over the block that was held under it
    on_for_counts(180, -20);
    //Lowers the claw again
    lower();
    //Lowers the claw a bit more to make sure it's on the ground
    ev3_motor_rotate(lift_motor, -5, 30, true);
    //Closes the claw to grab the 3rd concrete block
    ev3_motor_rotate(grab_motor, -230, 50, true);
    //Lifts the claw with the concrete block to stack it
    ev3_motor_rotate(lift_motor, 178, 10, true);
    //Goes forward to have the claw over the aligned blocks
    on_for_counts(165, 20);
    //Lowers the claw by a small amount to get just over the two aligned blocks
    ev3_motor_rotate(lift_motor, -30, 30, true);
    //Lets go of the 3rd block, to put it on top of the two others
    ev3_motor_rotate(grab_motor, 80, 40, true);
    //Backs up once the blocks are stacked
    on_for_counts(250, -20);
}

void get_sandbags_from_start() {
    //Reset gyro
    pid_gyro().reset_gyro();
    //Goes back until the robot hits the wall
    ev3_motor_steer(left_motor, right_motor, -60, 0);
    sleep(1150);
    motor_stop();
    //Sleep to make sure the robot stops moving
    sleep(300);
    //Goes from the wall to the middle of the mat
    on_for_counts_motor(800, 40);
    sleep(300);
    //Turns 90 degrees to the right
    turn(90);
    sleep(150);
    //Goes forwards for counts to pass the white circle the robot could otherwise interpret as the white before the line
    on_for_counts_motor(1069, 50, 0);
    //Checks for white then green and blue on both the sensors to stop at the blue-green line
    while (ev3_color_sensor_get_color(s1) != COLOR_WHITE && ev3_color_sensor_get_color(s2) != COLOR_WHITE){
        ev3_motor_steer(left_motor, right_motor, 50, 2);
    }
    while (ev3_color_sensor_get_color(s1) != COLOR_GREEN && ev3_color_sensor_get_color(s2) != COLOR_BLUE) {
        ev3_motor_steer(left_motor, right_motor, 50, 2);
    } 
    //Gradually slows down the motors then stops them
    brake_motors(40);
    motor_stop();
    //Turns to the left
    turn(-90);
    sleep(500);
    //Checks for white then black on both the sensors to stop at the black line
    while (ev3_color_sensor_get_color(s1) != COLOR_WHITE && ev3_color_sensor_get_color(s2) != COLOR_WHITE){
        ev3_motor_steer(left_motor, right_motor, 50, 0);
    }
    while (ev3_color_sensor_get_color(s1) != COLOR_BLACK && ev3_color_sensor_get_color(s2) != COLOR_BLACK) {
        ev3_motor_steer(left_motor, right_motor, 50, 0);
    } 
    //Gradually slows down the motors then stops them
    brake_motors(40);
    motor_stop();
    //Backs up for 250ms at -20 power to get behind the line so the robot can align itself
    ev3_motor_steer(left_motor, right_motor, -20, 0);
    sleep(250);
    motor_stop();
    //Robot aligns itself against the black line
    align(8, COLOR_BLACK);
    sleep(100);
    //Goes forward for 250ms to place the robot for the turn
    ev3_motor_steer(left_motor, right_motor, 15, 0);
    sleep(250);
    motor_stop();
    //Turns right
    turn(90);
    sleep(200);
    //Sets the start angle for the gyro PID straight-line controller
    //This could be swapped out for normal motor control
    int start_angle = ev3_gyro_sensor_get_angle(gyro);
    //Accelerates motors to 40
    ramp_motors(40);
    //Checks for white then black on both the sensors to stop at the black line
    while (ev3_color_sensor_get_color(s1) != COLOR_WHITE && ev3_color_sensor_get_color(s2) != COLOR_WHITE){
        pid_gyro().follow(40, start_angle);
    }
    while (ev3_color_sensor_get_color(s1) != COLOR_BLACK && ev3_color_sensor_get_color(s2) != COLOR_BLACK) {
        pid_gyro().follow(40, start_angle);
    }
    //Gradually slows down the motors then stops them
    brake_motors(40);
    motor_stop();
    //Goes back for 250ms to get behind the line
    ev3_motor_steer(left_motor, right_motor, -20, 0);
    sleep(250);
    motor_stop();
    //Robot aligns itself against the black line
    align(8, COLOR_BLACK);
    //Open the claw
    ev3_motor_rotate(grab_motor, -420, 50, true);
    //Lowers the claw
    ev3_motor_rotate(lift_motor, -170, 30, true);
    //Deactivates the brake on the motor
    ev3_motor_stop(lift_motor, false);
    //Goes forward for 200 counts, with a target angle of 90 (to get to the sanbags)
    on_for_counts(200, 20, 1, 90);
    //Closes claw to be the width of the sandbags
    ev3_motor_rotate(grab_motor, -155, 30, true);
    //Lifts the claw
    lift();
    sleep(500);
}

void go_to_house() {
    //Goes backwards to the intersection between the black and blue line
    on_for_counts_motor(540, -50);
    //Turns right with a slight overshoot (seems to be necessary)
    turn(95);
    sleep(150);
    //Goes forward to about the middle, to avoid false detection triggers of the next action
    on_for_counts_motor(600, 40, 0);
    //Checks for white then black on both the sensors to stop at the black line
    while (ev3_color_sensor_get_color(s1) != COLOR_WHITE && ev3_color_sensor_get_color(s2) != COLOR_WHITE){
        ev3_motor_steer(left_motor, right_motor, 30, 2);
    }
    while (ev3_color_sensor_get_color(s1) != COLOR_BLACK && ev3_color_sensor_get_color(s2) != COLOR_BLACK) {
        ev3_motor_steer(left_motor, right_motor, 30, 2);
    }
    //Gradually slows down the motors then stops them
    brake_motors(30);
    motor_stop();
    //Backs up from the line to allow the robot to align itself after
    ev3_motor_steer(left_motor, right_motor, -20, 0);
    sleep(250);
    motor_stop();
    sleep(150);
}

void place_sandbags() {
    //Robot aligns itself
    align(8, COLOR_BLACK);
    //Lowers the claw
    ev3_motor_rotate(lift_motor, -170, 30, true);
    //Turns slightly left to offset a sandbag
    turn(-10, 0);
    sleep(300);
    //Opens the claw to let go of the sandbags
    ev3_motor_rotate(grab_motor, 50, 80, true);
    //Backs up for 325ms to make the claw only grab one sandbag
    ev3_motor_steer(left_motor, right_motor, -20, 0);
    sleep(325);
    motor_stop();
    sleep(150);
    //Closes the claw around the second sandbag
    ev3_motor_rotate(grab_motor, -40, 80, true);
    //Turns to offset the other sandbag by the same angle
    turn(20, 0);
    //Lets go of the second sandbag and opens the claw flat
    ev3_motor_rotate(grab_motor, 240, 80, true);
    sleep(500);
    //Goes back to the initial position
    turn(-10, 0);
    //Goes forward to push the sandbags together and against the house
    on_for_counts(300, 15);
}

void check_concrete_color_marker() {
    //Initializes the rbg variable for the hitechnic
    rgb_raw_t color;
    //Lifts the claw up to not hit the house
    ev3_motor_rotate(lift_motor, 173, 30, true);
    //Goes backwards to be set up for the turn
    on_for_counts_motor(200, -15);
    //Turns left to go towards the concrete block color indicator (the slight undershoot is because the robot follows the line and will correct itself)
    turn(-87);
    //Follows the line until the black T intersection
    follow_to_line(1, -1);
    //Goes forward to get closer to the concrete color marker
    on_for_counts(300, 20);
    //Lowers the claw to place the hitechnic sensor in front of the marker
    ev3_motor_rotate(lift_motor, -178, 30, true);
    //Measure the RGB color
    ht_nxt_color_sensor_measure_rgb(hitechnic, &color);
    //Sends the values to the bluetooth terminal on the computer to check the data
    fprintf(bt, "%d\n", color.r);
    fprintf(bt, "%d\n", color.g);
    fprintf(bt, "%d\n", color.b);
}

void main_task(intptr_t unused) {
    //Checks if the battery is too low and needs charging
    if (ev3_battery_voltage_mV() < 9500) {
        ev3_speaker_play_tone(500, 5000);
    }
    //Initializes all the sensors and motors
    ev3_sensor_config(gyro, GYRO_SENSOR);
    ev3_sensor_config(s1, COLOR_SENSOR);
    ev3_sensor_config(s2, COLOR_SENSOR);
    ev3_sensor_config(hitechnic, HT_NXT_COLOR_SENSOR);
    ev3_motor_config(left_motor, MEDIUM_MOTOR);
    ev3_motor_config(right_motor, MEDIUM_MOTOR);
    ev3_motor_config(lift_motor, LARGE_MOTOR);
    ev3_motor_config(grab_motor, MEDIUM_MOTOR);
    //Resets the gyro sensor
    ev3_gyro_sensor_reset(gyro);
    //3 second sleep to allow all sensors to fully start up (notably the EV3 color sensors)
    sleep(3000);
}
