/*
 * File:          3hv_gps.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/device.h>
#include <webots/motor.h>

#include <stdio.h>
#include <stdlib.h>
#include <defines.h>
#include <time.h>


// external function definitions
extern void reset_robot(struct Device devices[], int num_devices);
extern struct MotorInfo enable_all_gps_devices(struct Device devices[], int num_devices);
extern double get_gps_val(char * name, int coord, struct Device devices[], int num_devices);
extern WbDeviceTag get_tag_from_device_name(const char * name, struct Device devices[], int num_devices);
extern void print_population(struct PathSet population[], int num_devices);
extern void print_population_member(struct PathSet population[], int num_devices, int member);

extern void create_path(struct PathSet population[], int num_devices);
extern void create_movement_iter(int * moves, int num_devices);
extern void rotate_joint(int index, double relAngle);
extern void rotate_all_joints(double * relAngles, int * indexes, int num_motors);
extern void decode_and_move(struct PathSet member, int * indexes, int num_devices);

extern double find_fitness(struct PathSet member, struct Device devices[], int num_devices);
extern void sort_by_fitness(struct PathSet population[], int num_devices);
extern void breed_parents(struct PathSet population[], int num_devices, int p1_ind, int p2_ind, int c1_ind, int c2_ind);
extern void mutate_pop(struct PathSet population[], struct Device devices[], struct MotorInfo motors, int num_tot_devices);
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  srand((unsigned)time(NULL));

  int n_devices = wb_robot_get_number_of_devices();
  int i;
  struct Device devices[n_devices];
  for(i = 0; i < n_devices; i++) {
    // get device info
    WbDeviceTag tag = wb_robot_get_device_by_index(i);
    const char *name = wb_device_get_name(tag);

    // do something with the device
    printf("Device #%d name = %s\n", i, name);
    devices[i].tag = tag;
    devices[i].name = name;
  }

  // enable gps devices and get motor info
  struct MotorInfo motors = enable_all_gps_devices(devices, n_devices);
  
  reset_robot(devices, n_devices);

  // create population
  struct PathSet population[POPULATION_SIZE];
  for (int h = 0; h < POPULATION_SIZE; h++) {
    population[h].path = malloc(sizeof(int*) * PATH_LENGTH_TOT);
    for (int k = 0; k < PATH_LENGTH_TOT; k++) {
      population[h].path[k] = malloc(sizeof(int) * motors.num_motors);
    } 
  }

  // initialize population
  for (int member = 0; member < POPULATION_SIZE; member++) {
    int pathLength = (rand() % (PATH_LEN_HIGH - PATH_LEN_LOW - 1)) + PATH_LEN_LOW;
    population[member].path_length = pathLength;
    population[member].fitness = 0.0;
    for (int path_i = 0; path_i < PATH_LENGTH_TOT; path_i++) {
      for (int motor = 0; motor < motors.num_motors; motor++) {
        population[member].path[path_i][motor] = 7;
      }
    }
  }

  // Create initial population and show it
  create_path(population, motors.num_motors);
  wb_robot_step(1000);
  print_population(population, motors.num_motors);
  wb_robot_step(1000);  
  
  // run through population and get the fitness values
  for (int member = 0; member < POPULATION_SIZE; member++)
  {
    decode_and_move(population[member], motors.motor_indexes, motors.num_motors);
    wb_robot_step(500);
    population[member].fitness = find_fitness(population[member], devices, n_devices);
    reset_robot(devices, n_devices);
    wb_robot_step(1000);
  }

  sort_by_fitness(population, n_devices);
  print_population(population, motors.num_motors);

  // double x = get_gps_val("back_gps", X_GPS, devices, n_devices);
  // double y = get_gps_val("back_gps", Y_GPS, devices, n_devices);
  // double z = get_gps_val("back_gps", Z_GPS, devices, n_devices);
  // printf("x: %f, y: %f, z: %f\n", x, y, z);

  // loop through generations
  for (int gen = 0; gen < MAX_GENERATIONS; gen++) {
    //output info on this generation
    printf("-----------------------------\n");
    printf("Generation: %d\n", gen);
    //print entire pop
    print_population(population, motors.num_motors);

    for (int dead = POPULATION_SIZE/2; dead < POPULATION_SIZE; dead+=2) {
      int p1_ind = rand() % (POPULATION_SIZE / 4);
      int p2_ind = p1_ind;
      // loop to make sure they're not the same parent
      while (p1_ind == p2_ind) {
        p2_ind = rand() % (POPULATION_SIZE / 4);
      }

      printf("Breeding parents %d and %d\n", p1_ind, p2_ind);
      print_population_member(population, motors.num_motors, p1_ind);
      print_population_member(population, motors.num_motors, p2_ind);
      breed_parents(population, motors.num_motors, p1_ind, p2_ind, dead, dead+1);

      decode_and_move(population[dead], motors.motor_indexes, motors.num_motors);
      wb_robot_step(500);
      population[dead].fitness = find_fitness(population[dead], devices, n_devices);
      reset_robot(devices, n_devices);
      wb_robot_step(1000);

      decode_and_move(population[dead+1], motors.motor_indexes, motors.num_motors);
      wb_robot_step(500);
      population[dead+1].fitness = find_fitness(population[dead+1], devices, n_devices);
      reset_robot(devices, n_devices);
      wb_robot_step(1000);

      printf("Child 1:\n");
      print_population_member(population, motors.num_motors, dead);
      printf("Child 2:\n");
      print_population_member(population, motors.num_motors, dead+1);
    }

    printf("pre-mutate:\n");
    print_population(population, motors.num_motors);
    mutate_pop(population, devices, motors, n_devices);
    sort_by_fitness(population, motors.num_motors);

    // end of generation
  }
  print_population(population, motors.num_motors);
  // reset_robot(devices, n_devices);

  // move to best position
  decode_and_move(population[0], motors.motor_indexes, motors.num_motors);

  /* Enter your cleanup code here */
  wb_robot_step(1000);

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
