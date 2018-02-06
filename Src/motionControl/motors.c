#include "motionControl/motors.h"

void motor_left_drive(int16_t value) {
    if (value > 0) {
        gpio_digitalWrite(&rightMotDir, HIGH);
    } else {
        gpio_digitalWrite(&leftMotDir, LOW);
    }

    pwm_left_set(fabs(value));
}

void motor_left_stop() {

}

void motor_right_drive(int16_t value) {
    if (value > 0) {
        gpio_digitalWrite(&rightMotDir, HIGH);
    } else {
        gpio_digitalWrite(&rightMotDir, LOW);
    }

    pwm_right_set(fabs(value));
}

void motor_right_stop() {
    
}
