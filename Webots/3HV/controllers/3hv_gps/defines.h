

// general constants
#define TIME_STEP 64

// GA constants
#define PATH_LENGTH_TOT 80
#define PATH_LEN_LOW 16
#define PATH_LEN_HIGH 80
#define POPULATION_SIZE 16
#define MOTOR_STEP_ANGLE .1
#define NUM_ITER 1
#define X_GPS 0
#define Y_GPS 1
#define Z_GPS 2
#define ST_X 0
#define ST_Y .25
#define ST_Z 0
#define MAX_GENERATIONS 50
#define MUTATION_FREQ 20
#define MUTATION_RATE 20

// general structs
struct Device {
  WbDeviceTag tag;
  const char * name;
};

struct MotorInfo {
  int * motor_indexes;
  int num_motors;
};

struct PathSet {
  int ** path;
  int path_length;
  double fitness;
};