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

// external functions
extern double get_gps_val(char * name, int coord, struct Device devices[], int num_devices);
extern void decode_and_move(struct PathSet member, int * indexes, int num_devices);
extern void reset_robot(struct Device devices[], int num_devices);

// function definitions
double find_fitness(struct PathSet member, struct Device devices[], int num_devices);
void sort_by_fitness(struct PathSet population[], int num_devices);
void breed_parents(struct PathSet population[], int num_devices, int p1_ind, int p2_ind, int c1_ind, int c2_ind);
void mutate_pop(struct PathSet population[], struct Device devices[], struct MotorInfo motors, int num_tot_devices);

double find_fitness(struct PathSet member, struct Device devices[], int num_devices) {
    double final_x = get_gps_val("back_gps", X_GPS, devices, num_devices);
    double final_y = get_gps_val("back_gps", Y_GPS, devices, num_devices);
    double final_z = get_gps_val("back_gps", Z_GPS, devices, num_devices);
    double lateral_dist = pow((pow(final_x - ST_X, 2) + pow(final_z - ST_Z, 2)), .5);
    double vertical_dist = final_y - ST_Y;
    // double fitness = vertical_dist * PATH_LENGTH_TOT / member.path_length;
    double fitness = pow(vertical_dist, 3.0) * PATH_LENGTH_TOT / member.path_length;
    printf("fitness: %f, vert: %f, len: %d\n", fitness, vertical_dist, member.path_length);
    return fitness;
}

void sort_by_fitness(struct PathSet population[], int num_devices) {
    struct PathSet temp_member;
    for (int memb1 = 0; memb1 < POPULATION_SIZE; memb1++) {
        for (int memb2 = memb1+1; memb2 < POPULATION_SIZE; memb2++) {
            if (population[memb1].fitness < population[memb2].fitness) {
                temp_member = population[memb1];
                // memcpy(&temp_member, &population[memb1], sizeof(struct PathSet));
                population[memb1] = population[memb2];
                // memcpy(&population[memb1], &population[memb2], sizeof(struct PathSet));
                population[memb2] = temp_member;
                // memcpy(&population[memb2], &temp_member, sizeof(struct PathSet));
            }
        }
    }   
}


void breed_parents(struct PathSet population[], int num_devices, int p1_ind, int p2_ind, int c1_ind, int c2_ind) {
    int p1_path_len = population[p1_ind].path_length;
    int p2_path_len = population[p2_ind].path_length;
    int p1_cutoff = rand() % p1_path_len;
    int p2_cutoff = rand() % p2_path_len;
    int c1_len = p1_cutoff + p2_path_len - p2_cutoff;
    int c2_len = p2_cutoff + p1_path_len - p1_cutoff;
    // loop and make sure we're not over the limit
    while (c1_len > PATH_LENGTH_TOT || c2_len > PATH_LENGTH_TOT) {
        p1_cutoff = rand() % p1_path_len;
        p2_cutoff = rand() % p2_path_len;
        c1_len = p1_cutoff + p2_path_len - p2_cutoff;
        c2_len = p2_cutoff + p1_path_len - p1_cutoff;
    }

    printf("p1 len: %d, p1 cut: %d, p2 len : %d, p2 cut: %d\n", p1_path_len, p1_cutoff, p2_path_len, p2_cutoff);

    int p1c2_ind = p2_cutoff;
    int p2c1_ind = p1_cutoff;

    // get data from first parent
    for (int fp = 0; fp < p1_path_len; fp++) {
        // printf("%d, %d, %d\n", fp, m, p1c2_ind);
        if (fp < p1_cutoff) {
            for (int m = 0; m < num_devices; m++) {
                population[c1_ind].path[fp][m] = population[p1_ind].path[fp][m];
            }
        }
        else {
            for (int m = 0; m < num_devices; m++) {
                population[c2_ind].path[p1c2_ind][m] = population[p1_ind].path[fp][m];
            }
            p1c2_ind++;
        }
    }

    // get data from second parent
    for (int sp = 0; sp < p2_path_len; sp++) {
        // printf("%d, %d\n", sp, m);
        if (sp < p2_cutoff) {
            for (int m = 0; m < num_devices; m++) {
                population[c2_ind].path[sp][m] = population[p2_ind].path[sp][m];
            }
        }
        else {
            for (int m = 0; m < num_devices; m++) {
                population[c1_ind].path[p2c1_ind][m] = population[p2_ind].path[sp][m];
            }
            p2c1_ind++;
        }
    }

    // pad with non-values
    for (int pad = p1_cutoff + p2_path_len - p2_cutoff; pad < PATH_LENGTH_TOT; pad++) {
        for (int dev = 0; dev < num_devices; dev++) {
            population[c1_ind].path[pad][dev] = 7;
        }
    }
    for (int pad = p2_cutoff + p1_path_len - p1_cutoff; pad < PATH_LENGTH_TOT; pad++) {
        for (int dev = 0; dev < num_devices; dev++) {
            population[c2_ind].path[pad][dev] = 7;
        }
    }
    // update path lengths
    population[c1_ind].path_length = c1_len;
    population[c2_ind].path_length = c2_len;
}


void mutate_pop(struct PathSet population[], struct Device devices[], struct MotorInfo motors, int num_tot_devices) {
    int action_index = 0;
    bool did_mutation = false;
    for (int member = 0; member < POPULATION_SIZE; member++) {
        for (int path = 0; path < population[member].path_length; path++) {
            if (action_index % MUTATION_FREQ == 0) {
                if (rand() % 100 < MUTATION_RATE) {
                    printf("doing mutation on member %d, path %d\n", member, path);
                    did_mutation = true;
                    for (int motor = 0; motor < motors.num_motors; motor++) {
                        population[member].path[path][motor] = (population[member].path[path][motor] + 1) % 3;
                    }
                }
            }
            action_index++;
        }
        if (did_mutation == true) {
            printf("moving after mutation\n");
            decode_and_move(population[member], motors.motor_indexes, motors.num_motors);
            wb_robot_step(500);
            population[member].fitness = find_fitness(population[member], devices, num_tot_devices);
            reset_robot(devices, num_tot_devices);
            wb_robot_step(1000);
        }
        did_mutation = false;
    }
}