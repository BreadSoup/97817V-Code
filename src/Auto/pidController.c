/*#include "../../include/pid/pidController.h"

void pid_init(pid_Controller *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
prevErrOpen
or = 0;
    pid->integral = 0;
}

double pid_update(pid_Controller *pid, double setpint, double processVariable) {
    // Calculate error
    double error = pid->setpoint - processVariable;

    // Calculate proportional term
    double proportional = pid->kp * error;

    // Calculate integral term
    pid->integral += pid->ki * error;

    // Calculate derivative term
    double derivative = pid->kd * (error - pid->prevError);

    // Calculate PID output
    double output = proportional + pid->integral + derivative;

    // Update previous error
    pid->prevError = error;

    return output;
}

double pid_getKp(pid_Controller *pid) {
    return pid->kp;
}

double pid_getKi(pid_Controller *pid) {
    return pid->ki;
}

double pid_getKd(pid_Controller *pid) {
    return pid->kd;
}

void pid_setKp(pid_Controller *pid, double newKp) {
    pid->kp = newKp;
}

void pid_setKi(pid_Controller *pid, double newKi) {
    pid->ki = newKi;
}

void pid_setKd(pid_Controller *pid, double newKd) {
    pid->kd = newKd;
}*/