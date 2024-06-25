#include <stdbool.h>

void robotControllerInit();

double getRobotBearing();

double * robotControllerGetRobotCoordinate();

double robotControllerGetRobotHeading();

void motorStop();

void motorMoveForward();

void motorRotateLeft();

void motorRotateRight();

void motorTurnLeft();

void motorTurnRight();

void motor_rotate_left_in_degrees(float degrees);

bool* get_sensors_condition();

void print_sensor_values();
