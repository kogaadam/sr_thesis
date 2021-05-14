// webots includes
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/device.h>
#include <webots/motor.h>
#include <webots/supervisor.h>

// general includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// project includes
#include <defines.h>

// function definitions
void create_path(struct PathSet population[], int num_devices);
void create_movement_iter(int * moves, int num_devices);
void rotate_joint(int index, double relAngle);
void rotate_all_joints(double * relAngles, int * indexes, int num_motors);
void decode_and_move(struct PathSet member, int * indexes, int num_devices);

void create_path(struct PathSet population[], int num_devices) {
    for (int member = 0; member < POPULATION_SIZE; member++) {
        for (int path_i = 0; path_i < population[member].path_length; path_i++) {
            create_movement_iter(population[member].path[path_i], num_devices);
        }
    }
    
}

void create_movement_iter(int * moves, int num_devices) {
    for (int i = 0; i < num_devices; i++)
    {
        moves[i] = rand() % 3;
    }
}

// rotate all joints by the given relative angles
void rotate_all_joints(double * relAngles, int * indexes, int num_motors) {
    for (int i = 0; i < num_motors; i++) {
        rotate_joint(indexes[i], relAngles[i]);
    }
}

// rotate a singular joint forward or backward
//     args - index: index of device being moved
void rotate_joint(int index, double relAngle) {
    WbDeviceTag tag = wb_robot_get_device_by_index(index);
    double min_pos = -1.0;//wb_motor_get_min_position(tag);
    double max_pos = 1.0;//wb_motor_get_max_position(tag);
    double curr_pos = wb_motor_get_target_position(tag);
    if (curr_pos + relAngle < min_pos || curr_pos + relAngle > max_pos)
        // printf("ERROR: out of range\n");
        ;
    else
        wb_motor_set_position(tag, curr_pos + relAngle);
}

void decode_and_move(struct PathSet member, int * indexes, int num_devices) {
    for (int path_i = 0; path_i < member.path_length; path_i++) {
        double relAngles[num_devices];
        for (int motor = 0; motor < num_devices; motor++) {
            if (member.path[path_i][motor] == 1) {
                // move back
                relAngles[motor] = -MOTOR_STEP_ANGLE;
            }
            else if (member.path[path_i][motor] == 2) {
                // move forward
                relAngles[motor] = MOTOR_STEP_ANGLE;
            }
            else {
                relAngles[motor] = 0;
            }
        }
        rotate_all_joints(relAngles, indexes, num_devices);
        wb_robot_step(TIME_STEP*5);
    }
}