
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
#define PATH_LENGTH_TOT 80
#define PI 3.14159
#define PRINT true
#define NO_PRINT false
#define ROT_WHEEL_VEL 5
#define STR_WHEEL_VEL 10
#define ROT_THR .1
#define BALL_X_Z -.6
#define PATH_LEN_LOW 16
#define PATH_LEN_HIGH 80
#define POPULATION 32
#define MAX_GENERATIONS 100

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
void create_path(char * path, int path_len);
void follow_path(char * path, int path_len);
void print_path(char * path, int path_len);
void reset_robot();
double find_fitness(char * path, int path_len, bool print);
void print_population(char (*population)[PATH_LENGTH_TOT], char * path_lengths, double * fitness_vals);
void sort_fitness(char (*population)[PATH_LENGTH_TOT], char * path_lengths, double * fitness_vals);
int * breed_parents(char (*population)[PATH_LENGTH_TOT],
                   char * child1,
                   char * child2,
                   char * path_lengths,
                   int first_parent,
                   int second_parent);
void mutate_pop(char (*population)[PATH_LENGTH_TOT], char * path_lengths, double * fitness_vals);

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
  int pathLength;
  char population[POPULATION][PATH_LENGTH_TOT] = {{0}};
  char path_lengths[POPULATION] = {0};
  double fitness_vals[POPULATION] = {0};
  char path[PATH_LENGTH_TOT] = {0};
  char child1[PATH_LENGTH_TOT] = {0};
  char child2[PATH_LENGTH_TOT] = {0};
  int * new_lens_p;

  srand((unsigned)time(NULL));

  for (int i = 0; i < PATH_LENGTH_TOT; i++)
  {
    path[i] = 7;
    for (int j = 0; j < POPULATION; j++)
    {
      population[j][i] = 7;
    }
  }
  
  print_population(population, path_lengths, fitness_vals);

  for (int i = 0; i < POPULATION; i++) {
    printf("iteration %d of %d\n", i, POPULATION);
    pathLength = (rand() % (PATH_LEN_HIGH - PATH_LEN_LOW - 1)) + PATH_LEN_LOW;
    path_lengths[i] = pathLength;
    // printf("path length: %d\n", pathLength);
    create_path(path, pathLength);
    for (int j = 0; j < PATH_LENGTH_TOT; j++) {
      population[i][j] = path[j];
    }
    // print_path(path, PATH_LENGTH_TOT);
    follow_path(path, pathLength);
    fitness_vals[i] = find_fitness(path, pathLength, NO_PRINT);
    reset_robot();
    wb_robot_step(500);
  }

  print_population(population, path_lengths, fitness_vals);
  sort_fitness(population, path_lengths, fitness_vals);
  print_population(population, path_lengths, fitness_vals);

  for (int generation = 0; generation < MAX_GENERATIONS; generation++) {
    //output info on this generation
    printf("-----------------------------\n");
    printf("Generation: %d\n", generation);

    //only print best member
    // printf("fitness: %f | path length: %d | path: ", fitness_vals[0], path_lengths[0]);
    // print_path(population[0], PATH_LENGTH_TOT);

    //print entire pop
    print_population(population, path_lengths, fitness_vals);

    printf("-----------------------------\n");
    // replace the bottom half of parents
    for (int i = POPULATION / 2; i < POPULATION; i+=2) {
      int first_parent = rand() % (POPULATION / 4);
      int second_parent = first_parent;
      while (second_parent == first_parent) {
        second_parent = rand() % (POPULATION / 4);
      }

      printf("Breeding parents %d and %d\n", first_parent, second_parent);
      printf("fitness: %09.6f | path length: %d | path: ", fitness_vals[first_parent], path_lengths[first_parent]);
      print_path(population[first_parent], PATH_LENGTH_TOT);
      printf("fitness: %09.6f | path length: %d | path: ", fitness_vals[second_parent], path_lengths[second_parent]);
      print_path(population[second_parent], PATH_LENGTH_TOT);

      new_lens_p = breed_parents(population, child1, child2, path_lengths, first_parent, second_parent);
      for (int j = 0; j < PATH_LENGTH_TOT; j++) {
        population[i][j] = child1[j];
        population[i+1][j] = child2[j];
      }
      path_lengths[i] = new_lens_p[0];
      path_lengths[i+1] = new_lens_p[1];
      follow_path(population[i], path_lengths[i]);
      fitness_vals[i] = find_fitness(population[i], path_lengths[i], NO_PRINT);
      reset_robot();
      follow_path(population[i+1], path_lengths[i+1]);
      fitness_vals[i+1] = find_fitness(population[i+1], path_lengths[i+1], NO_PRINT);
      printf("child 1: fitness: %09.6f | path length: %d | path: ", fitness_vals[i], path_lengths[i]);
      print_path(child1, PATH_LENGTH_TOT);
      printf("child 2: fitness: %09.6f | path length: %d | path: ", fitness_vals[i+1], path_lengths[i+1]);
      print_path(child2, PATH_LENGTH_TOT);
      reset_robot();
    }

    printf("\npre mutate: \n");
    print_population(population, path_lengths, fitness_vals);

    //mutate
    mutate_pop(population, path_lengths, fitness_vals);
    sort_fitness(population, path_lengths, fitness_vals);

    printf("--------------------------------------------END OF GENERATION--------------------------------------------");
  }

  // go to best path
  follow_path(population[0], path_lengths[0]);
  printf("\nFinal Population:\n");
  print_population(population, path_lengths, fitness_vals);


  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  // while (wb_robot_step(TIME_STEP) != -1) {

  //   print_values(wb_keyboard_get_key());

  // };

  /* This is necessary to cleanup webots resources */
  wb_robot_step(1000);
  wb_robot_cleanup();

  return 0;
}

void mutate_pop(char (*population)[PATH_LENGTH_TOT], char * path_lengths, double * fitness_vals) {

  int accum_action = 0;
  int mut_cut = 20;
  int old_action = 0;
  int new_action = 0;

  for (int indiv = 0; indiv < POPULATION; indiv++) {
    for (int action = 0; action < path_lengths[indiv]; action++) {
      accum_action++;
      if (accum_action % mut_cut == 0) {
        if (rand() % 100 < 5) {
          old_action = population[indiv][action];
          new_action = (old_action + 1) % 3;
          printf("MUTATION: indiv %d, action %d, old action %d, new action %d\n", indiv, action, old_action, new_action);
          population[indiv][action] = new_action;
          follow_path(population[indiv], path_lengths[indiv]);
          fitness_vals[indiv] = find_fitness(population[indiv], path_lengths[indiv], NO_PRINT);
        }
      }
    }
  }

}

int * breed_parents(char (*population)[PATH_LENGTH_TOT],
                   char * child1,
                   char * child2,
                   char * path_lengths,
                   int first_parent,
                   int second_parent) {
  
  int p1_path_len = path_lengths[first_parent];
  int p2_path_len = path_lengths[second_parent];
  int p1_cutoff = rand() % p1_path_len;
  int p2_cutoff = rand() % p2_path_len;
  static int new_lens[2];
  new_lens[0] = p1_cutoff + p2_path_len - p2_cutoff;
  new_lens[1] = p2_cutoff + p1_path_len - p1_cutoff;
  while (new_lens[0] > 80 || new_lens[1] > 80) {
    p1_cutoff = rand() % p1_path_len;
    p2_cutoff = rand() % p2_path_len;
    new_lens[0] = p1_cutoff + p2_path_len - p2_cutoff;
    new_lens[1] = p2_cutoff + p1_path_len - p1_cutoff;
  }
  
  printf("p1 len: %d, p1 cut: %d, p2 len : %d, p2 cut: %d\n", p1_path_len, p1_cutoff, p2_path_len, p2_cutoff);
  // for (int i = 0; i < PATH_LENGTH_TOT; i++) {
  //   if (i < p1_cutoff) {
  //     child1[i] = population[first_parent][i];
  //   }
  //   else if (i < new_lens[0]) {
  //     child1[i] = population[second_parent][i + p2_cutoff - p1_cutoff];
  //   }
  //   else {
  //     child1[i] = 7;
  //   }

  //   if (i < p2_cutoff) {
  //     child2[i] = population[second_parent][i];
  //   }
  //   else if (i < new_lens[1]) {
  //     child2[i] = population[first_parent][i + p1_cutoff - p2_cutoff];
  //   }
  //   else {
  //     child2[i] = 7;
  //   }
  // }

  int fp_index = p2_cutoff;
  int sp_index = p1_cutoff;
  for (int fp = 0; fp < p1_path_len; fp++) {
    if (fp < p1_cutoff) {
      child1[fp] = population[first_parent][fp];
    }
    else {
      child2[fp_index] = population[first_parent][fp];
      fp_index++;
    }
  }
  for (int sp = 0; sp < p2_path_len; sp++) {
    if (sp < p2_cutoff) {
      child2[sp] = population[second_parent][sp];
    }
    else {
      child1[sp_index] = population[second_parent][sp];
      sp_index++;
    }
  }

  for (int pad = p1_cutoff + p2_path_len - p2_cutoff; pad < PATH_LENGTH_TOT; pad++) {
    child1[pad] = 7;
  }
  for (int pad = p2_cutoff + p1_path_len - p1_cutoff; pad < PATH_LENGTH_TOT; pad++) {
    child2[pad] = 7;
  }

  return new_lens;

}

// bubble sort
void sort_fitness(char (*population)[PATH_LENGTH_TOT], char * path_lengths, double * fitness_vals) {
  double temp_fitness, temp_plength;
  char temp_member[PATH_LENGTH_TOT] = {0};
  for (int i = 0; i < POPULATION; i++) {
    for (int j = i + 1; j < POPULATION; j++) {
      if (fitness_vals[i] < fitness_vals[j]) {
        temp_fitness = fitness_vals[i];
        temp_plength = path_lengths[i];
        // temp_member = population[i];
        fitness_vals[i] = fitness_vals[j];
        path_lengths[i] = path_lengths[j];
        for (int k = 0; k < PATH_LENGTH_TOT; k++) {
          temp_member[k] = population[i][k];
          population[i][k] = population[j][k];
          population[j][k] = temp_member[k];
        }
        fitness_vals[j] = temp_fitness;
        path_lengths[j] = temp_plength;
      }
    }
  }
}

void print_population(char (*population)[PATH_LENGTH_TOT], char * path_lengths, double * fitness_vals) {
  printf("population:\n");
  for (int i = 0; i < POPULATION; i++) {
    printf("fitness: %09.6f | path length: %d | path: ", fitness_vals[i], path_lengths[i]);
    print_path(population[i], PATH_LENGTH_TOT);
  }
}

double find_fitness(char * path, int path_len, bool print) {
  double final_x = get_gps(0);
  double final_z = get_gps(2);
  double dist_from_ball = pow((pow(final_x - BALL_X_Z, 2) + pow(final_z - BALL_X_Z, 2)), .5);
  if (print) {
    printf("final distance away: %f\n", dist_from_ball);
  }
  int num_turns = 0;
  for (int i = 0; i < path_len; i++) {
    if (path[i] > 0) {
      num_turns++;
    }
  }
  // double fitness = 1/dist_from_ball - (double)num_turns;
  double fitness = PATH_LENGTH_TOT / (dist_from_ball + 1) - num_turns;// 1 / (num_turns + 1);
  if (print) {
    printf("fitness: %f\n-----------------------------\n", fitness);
  }
  return fitness;
}

void follow_path(char * path, int path_len) {
  int dir = RT;
  for (int i = 0; i < path_len; i++) {
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

void create_path(char *path, int path_len) {
  int action = 0;
  int dir = RT;
  double x_coord = get_gps(0);
  double z_coord = get_gps(2);

  // srand((unsigned)time(NULL));
  //srand(0);

  for (int i = 0; i < PATH_LENGTH_TOT; i++) {
    if (i < path_len) {
      action = rand();
      if (action % 3 == 0) {
        //going straight
        // printf("(%d, %f, %f)", dir, get_gps(0), get_gps(2));
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
      else if (action % 3 == 1) {
        //turning left
        path[i] = 1;
        dir = (dir + 1) % 4;
        //printf("%d", path[i]);
      }
      else {
        //turning right
        path[i] = 2;
        dir = (dir + 3) % 4;
        //printf("%d", path[i]);
      }
    }
    else {
      path[i] = 7;
    }
  }
  //printf("\n");
}

void go_straight(bool print) {

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, STR_WHEEL_VEL);
  wb_motor_set_velocity(right_motor, STR_WHEEL_VEL);
  for (int i = 0; i < 16; i++) {
    wb_robot_step(TIME_STEP);
  }

  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
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

  for (int i = 0; i < 8; i++) {
    wb_robot_step(TIME_STEP);
  }

  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

void print_path(char * path, int path_len) {
  for (int i = 0; i < path_len; i++) {
    printf("%d", path[i]);
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
