
/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>


#define TIME_STEP 64
#define PATH_LENGTH 20
#define PI 3.14159
#define PRINT true
#define NO_PRINT false
#define ROT_WHEEL_VEL 5
#define STR_WHEEL_VEL 10
#define ROT_THR .1
#define BALL_X_Z -.6

enum direction {
  RT,
  UP,
  LT,
  DN
};

void init_robot();
void print_values(int key);
void rotate_robot(double pos, bool print);
void go_straight(bool print);
double get_imu(int axis);
double get_gps(int coord);
int * create_path();
void follow_path(int * path);
void print_path(int * path);
void reset_robot();
double find_fitness();

WbDeviceTag gps;
WbDeviceTag imu;
WbDeviceTag acc;
WbDeviceTag gyr;
WbDeviceTag left_motor;
WbDeviceTag right_motor;

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {

  init_robot();
  wb_robot_step(TIME_STEP);
  //reset_robot();
  //rotate_robot(PI/2, PRINT);
  for (int i = 0; i < 10; i++) {
    int * path2 = create_path();
    follow_path(path2);
    find_fitness(path2);
    reset_robot();
    wb_robot_step(500);
  }
  //print_values('I');

  /* get a handler to the motors and set target position to infinity (speed control). */

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */


  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {

    print_values(wb_keyboard_get_key());
    //print_values('I');
    //rotate_robot(1.57);


  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

double find_fitness(int * path) {
  double final_x = get_gps(0);
  double final_z = get_gps(2);
  double dist_from_ball = pow((pow(final_x - BALL_X_Z, 2) + pow(final_z - BALL_X_Z, 2)), .5);
  printf("final distance away: %f\n", dist_from_ball);
  int num_turns = 0;
  for (int i = 0; i < PATH_LENGTH; i++) {
    if (path[i] > 0) {
      num_turns++;
    }
  }
  double fitness = 1/dist_from_ball - (double)num_turns;
  printf("fitness: %f\n-----------------------------\n", fitness);
  return fitness;
}

void follow_path(int * path) {
  int dir = RT;
  for (int i = 0; i < PATH_LENGTH; i++) {
    //printf("|%d ", path[i]);
    if (path[i] == 0) {
      //printf("(to %d)|\n", dir);
      go_straight(NO_PRINT);
    }
    else if (path[i] == 1) {
      dir = (dir + 1) % 4;
      //printf("(to %d)|\n", dir);
      rotate_robot(PI/2, NO_PRINT);
    }
    else {
      dir = (dir + 3) % 4;
      //printf("(to %d)|\n", dir);
      rotate_robot(-PI/2, NO_PRINT);
    }
  }

  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

}

int * create_path() {
  static int path[PATH_LENGTH];
  bool turn = false;
  int action = 0;
  int dir = RT;
  double x_coord = get_gps(0);
  double z_coord = get_gps(2);

  srand((unsigned)time(NULL));
  //srand(0);

  for (int i = 0; i < PATH_LENGTH; i++) {
    action = rand();
    turn = false;
    if (action % 2 == 0 || path[i-1] > 0) {
      //going straight
      // printf("(%d, %f, %f)", dir, get_gps(0), get_gps(2));
      // Avoid walls
      if ((dir == RT && (z_coord - .75 < -1.2)) ||
          (dir == LT && (z_coord + .75 >  1.2)) ||
          (dir == UP && (x_coord - .75 < -1.2)) ||
          (dir == DN && (x_coord + .75 >  1.2))) {
        turn = true;
      }
      else {
        path[i] = 0;
        //printf("%d", path[i]);
        if (dir == RT) {
          z_coord -= .25;
        }
        else if (dir == LT) {
          z_coord += .25;
        }
        else if (dir == UP) {
          x_coord -= .25;
        }
        else {
          x_coord += .25;
        }
        action = 0;
      }
    }
    if (action % 2 == 1 || turn) {
      //turning
      if (rand() % 2 == 0) {
        //turn left
        path[i] = 1;
        dir = (dir + 1) % 4;
        //printf("%d", path[i]);
      }
      else {
        //turn right
        path[i] = 2;
        dir = (dir + 3) % 4;
        //printf("%d", path[i]);
      }
    }
  }
  //printf("\n");
  
  return path;
}

void go_straight(bool print) {
  double curr_x_gps = get_gps(0);
  double curr_z_gps = get_gps(2);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  while (fabs(get_gps(0) - curr_x_gps) < .25 && fabs(get_gps(2) - curr_z_gps) < .25) {
    if (print) {
      printf("Going straight\n");
      print_values('G');
      print_values('I');
    }
    wb_robot_step(TIME_STEP);
    wb_motor_set_velocity(left_motor, STR_WHEEL_VEL);
    wb_motor_set_velocity(right_motor, STR_WHEEL_VEL);
    if (print) {
      printf("--------\n");
    }
  }
}

void rotate_robot(double pos, bool print) {
  //double curr_y_pos = get_imu(2);
  //double targ_y_pos = fmod((curr_y_pos + pos),(2 * PI));
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  double left_vel = 0;
  double right_vel = 0;

  if (pos < 0) {
    left_vel = ROT_WHEEL_VEL;
    right_vel = -ROT_WHEEL_VEL;
  }
  else {
    left_vel = -ROT_WHEEL_VEL;
    right_vel = ROT_WHEEL_VEL;
  }
  wb_motor_set_velocity(left_motor, left_vel);
  wb_motor_set_velocity(right_motor, right_vel);

  // while (fabs(curr_y_pos - targ_y_pos) > ROT_THR) {
  //   wb_robot_step(TIME_STEP);
  //   if (print) {
  //     print_values('G');
  //     print_values('I');
  //     printf("Rotating:\ncurrent: %.3f\ntarget: %.3f\n--------\n", curr_y_pos, targ_y_pos);
  //   }
  //   curr_y_pos = get_imu(2);
  // }

  for (int i = 0; i < 8; i++) {
    wb_robot_step(TIME_STEP);
  }

  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

void print_path(int * p) {
  for (int i = 0; i < PATH_LENGTH; i++) {
    printf("%d", p[i]);
  }
  printf("\n");
}

double get_imu(int axis) {
  const double *imu_vals = wb_inertial_unit_get_roll_pitch_yaw(imu);
  return imu_vals[axis];
}

double get_gps(int coord) {
  const double *gps_vals = wb_gps_get_values(gps);
  return gps_vals[coord];
}

void reset_robot() {
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("GPS_ROBOT");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");

  // reset the robot
  const double INITIAL_TRANS[3] = { 1.1, 0, 1.1 };
  const double INITIAL_ROT[4] = { 0, 1, 0, 0 };
  wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL_TRANS);
  wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
  wb_supervisor_simulation_reset_physics();
}

void init_robot() {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /* get and enable GPS device */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  /* get and enable IMU */
  imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu, TIME_STEP);

  /* get and enable Accel */
  acc = wb_robot_get_device("acc");
  wb_accelerometer_enable(acc, TIME_STEP);

  /* get and enable Gyro */
  gyr = wb_robot_get_device("gyr");
  wb_gyro_enable(gyr, TIME_STEP);
  
  wb_keyboard_enable(500);
  
  printf("Press 'g' to read the GPS\n");
  printf("Press 'i' to read the IMU\n");
  printf("Press 'a' to read the Accelerometer\n");
  printf("Press 'y' to read the Gyro\n-----------------------------\n");

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");

}

void print_values(int key) {
  switch(key) {
    case 'G': {
      const double *gps_values = wb_gps_get_values(gps);
      printf("Using the GPS device: %.3f %.3f %.3f\n", gps_values[0], gps_values[1], gps_values[2]);
      break;
    }
    case 'I': {
      const double *imu_values = wb_inertial_unit_get_roll_pitch_yaw(imu);
      printf("Using the IMU device: %.3f %.3f %.3f\n", imu_values[0], imu_values[1], imu_values[2]);
      break;
    }
    case 'A': {
      const double *acc_values = wb_accelerometer_get_values(acc);
      printf("Using the Accel device: %.3f %.3f %.3f\n", acc_values[0], acc_values[1], acc_values[2]);
      break;
    }
    case 'Y': {
      const double *gyr_values = wb_gyro_get_values(gyr);
      printf("Using the Gyro device: %.3f %.3f %.3f\n", gyr_values[0], gyr_values[1], gyr_values[2]);
      break;
    }
    default:
      break;
  }
}