/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Default controller of the e-puck robot
 */

/* include headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

//#include "robot_controller.h"
//#include "cartesian.h"


// tangensial/linear speed in m/s. 
// Tangensial speed = angular speed * wheel radius 
// Tangensial speed = 6.28 rad * 2.05 cm = 0.12874 m/s
#define TANGENSIAL_SPEED 0.12874

// Speed of robot to spinning in place (in cycles per second)
// 1 cycle = 360 degrees. 
// Robot rotational speed = tangensial speed / (phi * axle length) 
// note: axle length is distance between wheels
// Robot rotational speed = 0.12874 / (phi*0.052) = 0.787744755
#define ROBOT_ROTATIONAL_SPEED 0.772881647

// Speed of robot to spinning in place (in degrees per second)
// Robot angular speed in degrees = robot rotational speed * 360 
// Robot angular speed in degrees = 0.787744755*360 = 283.588111888 
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 278.237392796


/* Device stuff */
#define DISTANCE_SENSORS_NUMBER 8
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

#define GROUND_SENSORS_NUMBER 3
static WbDeviceTag ground_sensors[GROUND_SENSORS_NUMBER];
static double ground_sensors_values[GROUND_SENSORS_NUMBER] = {0.0, 0.0, 0.0};
static const char *ground_sensors_names[GROUND_SENSORS_NUMBER] = {"gs0", "gs1", "gs2"};

#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};

static WbDeviceTag left_motor, right_motor,receiver;
#define COMMUNICATION_CHANNEL 0
#define RANGE 0.35

#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 6.28
static double speeds[2];

/* Breitenberg stuff */
static double weights[DISTANCE_SENSORS_NUMBER][2] = {{-1.3, -1.0}, {-1.3, -1.0}, {-0.5, 0.5}, {0.0, 0.0},
                                                     {0.0, 0.0},   {0.05, -0.5}, {-0.75, 0},  {-0.75, 0}};
static double offsets[2] = {0.5 * MAX_SPEED, 0.5 * MAX_SPEED};

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

/////////////////////////////////////////////////////////////
int time_step=32;
#define COORDINATE_MATCHING_ACCURACY 0.01 //in meter
#define THETA_MATCHING_ACCURACY 1 //in degrees
#define GPS_SAMPLING_PERIOD 1 //in ms
WbDeviceTag gps;

#define COMPASS_SAMPLING_PERIOD 1 //in ms
WbDeviceTag compass;



double * convertVec3fToCartesianVec2f( double coordinate3f[3]) {
    double *coordinate2f = malloc(2*sizeof(double));
    coordinate2f[0] = coordinate3f[0];
    coordinate2f[1] = -coordinate3f[2];
    return coordinate2f;
}

double * convertCartesianVec2fToVec3f( double coordinate2f[2]) {
    double *coordinate3f = malloc(3*sizeof(double));
    coordinate3f[0] = coordinate2f[0];
    coordinate3f[2] = -coordinate2f[1];
    return coordinate3f;
}

double convertCompassBearingToHeading(double heading) {
    /* 
	 * in webots, heading increasement is rotate in clockwise
	 * in cartesian, heading increasement is rotate in counterclockwise
	 * so headingInCartesian = 360-headingInWebots
	 * */
    heading = 360-heading;
    
    /* 
	 * in webots, heading is 0 if robot faced to y+ axis
	 * in cartesian, heading is 0 if robot face to x+ axis
	 * so headingInCartesian = headingInWebots+90
	 * */
    heading = heading + 90;
    if (heading > 360.0)
        heading = heading - 360.0;

    return heading;
}

bool isCoordinateEqual( double coordinate1[2],  double coordinate2[2])
{
    if (fabs(coordinate1[0]-coordinate2[0]) < COORDINATE_MATCHING_ACCURACY &&
            fabs(coordinate1[1]-coordinate2[1]) < COORDINATE_MATCHING_ACCURACY) {
        return true;
    }
    else {
        return false;
    }
}

bool isCoordinateVectorEqual( double coordinateVector1,  double coordinateVector2)
{
    if (fabs(coordinateVector1-coordinateVector2) < COORDINATE_MATCHING_ACCURACY) {
        return true;
    }
    else {
        return false;
    }
}

bool isThetaEqual(double theta, double theta2)
{
    if (fabs(theta - theta2) < THETA_MATCHING_ACCURACY)
    {
        return true;
    } else
    {
        return false;
    }
}

double calcDestinationThetaInDegrees(double currentCoordinate[2],double destinationCoordinate[2]) {
    return atan2(destinationCoordinate[1] - currentCoordinate[1], destinationCoordinate[0] - currentCoordinate[0]) * 180 / M_PI;
}

double calcThetaDot(double heading, double destinationTheta) {
    double theta_dot = destinationTheta - heading;

    if (theta_dot > 180)
        theta_dot = -(360-theta_dot);
    else if (theta_dot < -180)
        theta_dot = (360+theta_dot);

    return theta_dot;
}

double calcRotatedThetaByThetaDot(double theta, double theta_dot)
{
	if (theta_dot == 0)
              return theta;

	theta += theta_dot;
	/*
	 * if theta negative or more than 360, then convert it to normal degree
	 * */
	
	if (theta < 0)
              theta = theta + 360;
	else if (theta >= 360)
              theta = theta - 360;
	
	return theta;
}

double * robotControllerGetRobotCoordinate()
{
	return convertVec3fToCartesianVec2f(wb_gps_get_values(gps));
	  
}

double getRobotBearing()
{
    /* calculate bearing angle in degrees */
    double *north = wb_compass_get_values(compass);
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0) {
        bearing = bearing + 360.0;
	}
	
    return bearing;
}

double robotControllerGetRobotHeading()
{
	return convertCompassBearingToHeading(getRobotBearing());
}

void motorStop()
{
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

void motorMoveForward()
{
    wb_motor_set_velocity(left_motor, MAX_SPEED);
    wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motorRotateLeft()
{
    wb_motor_set_velocity(left_motor, -MAX_SPEED);
    wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motorRotateRight()
{
    wb_motor_set_velocity(left_motor, MAX_SPEED);
    wb_motor_set_velocity(right_motor, -MAX_SPEED);
}




double calcDistance(double currentCoordinate[2],double destinationCoordinate[2]) {
    return sqrt(pow(destinationCoordinate[0]-currentCoordinate[0], 2) + pow(destinationCoordinate[1]-currentCoordinate[1], 2));
}


int getTimeStep()
{
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;
}

double calcDistanceToDestination(double destinationCoordinate[2])
{
	double *currentCoordinate = robotControllerGetRobotCoordinate();
	return calcDistance(currentCoordinate, destinationCoordinate);
}

double calcThetaDotToDestination(double destinationCoordinate[2])
{
	double *currentCoordinate = robotControllerGetRobotCoordinate();
	double robotHeading = robotControllerGetRobotHeading();
	double destinationTheta = calcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate);
	return calcThetaDot(robotHeading, destinationTheta);
}

 


static void turn_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  passive_wait(0.2);
}

static void turn_right() {
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  passive_wait(0.2);
}

static void go_backwards() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
  passive_wait(0.2);
}

 
 
static bool cliff_detected() {
 
  int i;
  bool flag = false;
  double distance_sensor_vals[DISTANCE_SENSORS_NUMBER];
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensor_vals[i] = wb_distance_sensor_get_value(distance_sensors[i]);
    printf("distance sensor %d %f",i,distance_sensor_vals[i]);
    if (distance_sensor_vals[i]>90.0)
      flag = true;
  }

  return flag;
}

void separation_alg(){
    int i, bariers[DISTANCE_SENSORS_NUMBER];
    bool flag = false;
    double distance_sensor_vals[DISTANCE_SENSORS_NUMBER];
    for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
      distance_sensor_vals[i] = wb_distance_sensor_get_value(distance_sensors[i]);
      if (distance_sensor_vals[i]>90.0){
        flag = true;
        bariers[i]=true;
      }
      else
        bariers[i]=false;
    }
    bool left_obs = bariers[5] || bariers[6] || bariers[7];
    bool right_obs = bariers[0] || bariers[1] || bariers[2];
    bool back_obs = bariers[3]||bariers[4];
    bool front_obs = bariers[0]||bariers[7];
    if( front_obs && left_obs || front_obs){
      go_backwards();
      go_backwards();
      turn_right();  
    
       
    }         	 
    else if(front_obs && right_obs ){
      go_backwards();
      go_backwards();
      turn_left();
    
    }
    else if(back_obs){
      motorMoveForward();
    }

}

void robotControllerMoveToDestination(double destinationCoordinate[2])
{
      double distanceToDestination;
      double duration;
     
	// if the robot is already at the destination location
	if (isCoordinateEqual(robotControllerGetRobotCoordinate(), destinationCoordinate))
	{
		 
		return;
	}

	double * currentCoordinate = robotControllerGetRobotCoordinate();
	 
	
	// thetaDot is the degree of rotation needed by the robot to face the destination
	// thetaDot is zero if robot is facing the destination
	double thetaDotToDestination = calcThetaDotToDestination(destinationCoordinate);
	//printf("e-puck : thetaDotToDestination: %.5f\n", thetaDotToDestination);
	
	// if the robot is not facing the destination
	if (!isThetaEqual(thetaDotToDestination, 0))
	{
		// if the destination is on the left, robot will rotate to left
		if (thetaDotToDestination > 0)
		{
			// set robot motor to rotate left
			motorRotateLeft();
		}
		// if the destination is on the right, robot will rotate to right
		else if (thetaDotToDestination < 0)
		{
			// set robot motor to rotate right
			motorRotateRight();
		}

		// the duration needed for the robot to rotate the body to face the destination
		double duration = abs(thetaDotToDestination) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
		//printf("e-puck : duration to face the destination: %.5f\n", duration);

		// run the simulator
		double start_time = wb_robot_get_time();
		do
		{
			step();
		}
		while (wb_robot_get_time() < start_time + duration);
	}

	// the distance needed for the robot to reach its destination
	distanceToDestination = calcDistanceToDestination(destinationCoordinate);
	 
	
	// the duration needed for the robot to reach its destination
	duration = distanceToDestination / TANGENSIAL_SPEED;
	 
	// set robot motor to move forward

	 motorMoveForward();

  	separation_alg();
  
	
           passive_wait(duration);

	 
           
}
//////////////////////////////////////////////////////////////

static void init_devices() {
  int i;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }
  for (i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_names[i]);

  // silently initialize the ground sensors if they exists
  for (i = 0; i < GROUND_SENSORS_NUMBER; i++)
    ground_sensors[i] = (WbDeviceTag)0;
  int ndevices = wb_robot_get_number_of_devices();
  for (i = 0; i < ndevices; i++) {
    WbDeviceTag dtag = wb_robot_get_device_by_index(i);
    const char *dname = wb_device_get_name(dtag);
    WbNodeType dtype = wb_device_get_node_type(dtag);
    if (dtype == WB_NODE_DISTANCE_SENSOR && strlen(dname) == 3 && dname[0] == 'g' && dname[1] == 's') {
      int id = dname[2] - '0';
      if (id >= 0 && id < GROUND_SENSORS_NUMBER) {
        ground_sensors[id] = wb_robot_get_device(ground_sensors_names[id]);
        wb_distance_sensor_enable(ground_sensors[id], get_time_step());
      }
    }
  }

  // get a handler to the motors and set target position to infinity (speed control).
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

   
  // get a handler to the gps
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, GPS_SAMPLING_PERIOD);
	
    // get a handler to the compass
    compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, COMPASS_SAMPLING_PERIOD);

    receiver = wb_robot_get_device("receiver");
    wb_receiver_set_channel(receiver, COMMUNICATION_CHANNEL);
    wb_receiver_enable(receiver, get_time_step());
    int sp = wb_receiver_get_sampling_period(receiver);
     
    step();
}

static void reset_actuator_values() {
  int i;
  for (i = 0; i < 2; i++)
    speeds[i] = 0.0;
  for (i = 0; i < LEDS_NUMBER; i++)
    leds_values[i] = false;
}

static void get_sensor_input() {
  int i;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);

    // scale the data in order to have a value between 0.0 and 1.0
    // 1.0 representing something to avoid, 0.0 representing nothing to avoid
    distance_sensors_values[i] /= 4096;
  }

  for (i = 0; i < GROUND_SENSORS_NUMBER; i++) {
    if (ground_sensors[i])
      ground_sensors_values[i] = wb_distance_sensor_get_value(ground_sensors[i]);
  }
}

static void set_actuators() {
  int i;
  for (i = 0; i < LEDS_NUMBER; i++)
    wb_led_set(leds[i], leds_values[i]);
  wb_motor_set_velocity(left_motor, speeds[LEFT]);
  wb_motor_set_velocity(right_motor, speeds[RIGHT]);
}

static void blink_leds() {
  static int counter = 0;
  counter++;
  leds_values[(counter / 10) % LEDS_NUMBER] = true;
}

static void run_braitenberg(double speed_control) {
  int i, j;
  for (i = 0; i < 2; i++) {
    speeds[i] = 0.0;
    for (j = 0; j < DISTANCE_SENSORS_NUMBER; j++)
      speeds[i] += distance_sensors_values[j] * weights[j][i];

    speeds[i] = offsets[i] + speeds[i] * MAX_SPEED*speed_control;
    if (speeds[i] > MAX_SPEED)
      speeds[i] = MAX_SPEED;
    else if (speeds[i] < -MAX_SPEED)
      speeds[i] = -MAX_SPEED;
  }
}

struct msg{
  int robot_no;
  double x,y,z;
};

int main(int argc, char **argv) {
  char message[50];
  sprintf(message,"%s","my_obstacle_avoidance is started!");
  printf("%s\n", message);
  wb_robot_init();
  const char *name = wb_robot_get_name();
 
  
  double destinationCoordinate[2] = {0.0, 0.0};
  int process_start=0;    
   

  init_devices();
  struct msg * msg;
  
  while (true) {
    
     if (wb_receiver_get_queue_length(receiver) > 0) {
        // read current packet's data 
        const char *buffer = wb_receiver_get_data(receiver);
        msg = (void*)buffer;
          // print null-terminated message 
        if (atoi(name)==msg->robot_no){
           
          destinationCoordinate[0]=msg->x;
          destinationCoordinate[1]=msg->z;
          process_start=1;
        }
        // fetch next packet 
        wb_receiver_next_packet(receiver);
      }
   
  
 
    

    reset_actuator_values();
    get_sensor_input();
    blink_leds();

    separation_alg();             

     
        if(process_start==0)
            run_braitenberg(1);
        else{
          double distToDest =calcDistanceToDestination(destinationCoordinate);
          if (distToDest<0.4){
            run_braitenberg(0.02);
                
              }
          else{
            
            char message[50];
            double *pos = robotControllerGetRobotCoordinate();
            sprintf(message,"%s %f %f",name,pos[0],pos[1]);
            printf("%s\n", message);
             
           separation_alg();
                    
            robotControllerMoveToDestination(destinationCoordinate);
       
             separation_alg();
           
           
         
              
          }
        }
   
    set_actuators();
    step();
  };

  return EXIT_SUCCESS;
}
