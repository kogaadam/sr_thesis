// webots includes
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/device.h>
#include <webots/motor.h>
#include <webots/supervisor.h>

// general includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// project includes
#include <defines.h>

// function definitions
void reset_robot(struct Device devices[], int num_devices);
struct MotorInfo enable_all_gps_devices(struct Device devices[], int num_devices);
double get_gps_val(char * name, int coord, struct Device devices[], int num_devices);
WbDeviceTag get_tag_from_device_name(const char * name, struct Device devices[], int num_devices);
void print_population(struct PathSet population[], int num_devices);
void print_population_member(struct PathSet population[], int num_devices, int member);

// reset the robot to lying flat on its stomach
void reset_robot(struct Device devices[], int num_devices) {

  // move all motors back
  for (int i = 0; i < num_devices; i++) {
    // make sure its not a gps device
    if (strstr(devices[i].name, "gps") == NULL) {
       wb_motor_set_position(devices[i].tag, 0.0);
    }
  }

  wb_robot_step(1000);

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("HUM_ROBOT");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");

  // reset the robot
  const double INITIAL_TRANS[3] = { 0, 0.08, 0 };
  const double INITIAL_ROT[4] = { 1, 0, 0, 1.57 };
  // const double INITIAL_TRANS[3] = { 0, 1, 0 };
  // const double INITIAL_ROT[4] = { 1, 0, 0, 0 };
  wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL_TRANS);
  wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
  wb_supervisor_simulation_reset_physics();
}

// enable all gps devices
//   - assumes all gps devices have "gps" in their names
//   - returns list of indexes of motors
struct MotorInfo enable_all_gps_devices(struct Device devices[], int num_devices) {
  int num_motors = 0;
  for (int i = 0; i < num_devices; i++) {
    if (strstr(devices[i].name, "gps") != NULL) {
      wb_gps_enable(devices[i].tag, TIME_STEP);
      printf("GPS device enabled: %s\n", devices[i].name);
    }
    else
      num_motors++;
  }
  // int motor_indexes[num_motors];
  struct MotorInfo motors;
  motors.num_motors = num_motors;
  motors.motor_indexes = malloc(sizeof(int) * num_motors);
  int m_index = 0;
  for (int i = 0; i < num_devices; i++) {
    if (strstr(devices[i].name, "gps") == NULL) {
      motors.motor_indexes[m_index] = i;
      m_index++;
    }
  }
  return motors;
}

// get gps value from specific device
double get_gps_val(char * name, int coord, struct Device devices[], int num_devices) {
  WbDeviceTag gps = get_tag_from_device_name(name, devices, num_devices);
  const double * gps_vals = wb_gps_get_values(gps);
  return gps_vals[coord];
}

// get the tag associated with the provided device name
WbDeviceTag get_tag_from_device_name(const char * name, struct Device devices[], int num_devices) {
  for (int i = 0; i < num_devices; i++) {
    if (strcmp(devices[i].name, name) == 0) {
      return devices[i].tag;
    }
  }
  return 0;
}


// print population nicely
void print_population(struct PathSet population[], int num_devices) {
  time_t t;
  time(&t);
  printf("Printing full population, %s\n", ctime(&t));
  for (int member = 0; member < POPULATION_SIZE; member++) {
    printf("\nMember %d, Path Length = %d, Fitness = %.5f:\n", member,
                                                             population[member].path_length,
                                                             population[member].fitness);
    for (int path_i = 0; path_i < PATH_LENGTH_TOT; path_i++) {
      for (int motor = 0; motor < num_devices; motor++) {
        printf("%d", population[member].path[path_i][motor]);
      }
      printf(" | ");
      if (path_i % 10 == 9)
        printf("\n");
    }
  }
  printf("\n\n------------------------------------------------------------------------\n");
}

// print member of population nicely
void print_population_member(struct PathSet population[], int num_devices, int member) {
  printf("\nMember %d, Path Length = %d, Fitness = %.5f:\n", member,
                                                            population[member].path_length,
                                                            population[member].fitness);
  for (int path_i = 0; path_i < PATH_LENGTH_TOT; path_i++) {
    for (int motor = 0; motor < num_devices; motor++) {
      printf("%d", population[member].path[path_i][motor]);
    }
    printf(" | ");
    if (path_i % 10 == 9)
      printf("\n");
  }
  printf("\n\n------------------------------------------------------------------------\n");
}