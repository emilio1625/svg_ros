#ifndef SVG_ROS_STRUCTURES_H
#define SVG_ROS_STRUCTURES_H
// *****************************************************************************
// * Structures.                                                               *
// *****************************************************************************

typedef struct _coord {
    float xc, yc, anglec;
} coord;

typedef struct _Behavior {
    float xc, yc, status;
    int flg_vector;
} Behavior;

typedef struct _AdvanceAngle {
    float distance, angle;
} AdvanceAngle;


typedef struct _Inputs {

    char path[256];
    char file[256];
    char environment[100];
    char sensor[100];
    float xo, yo, xd, yd;
    float angle_robot;
    float theta_sensor;
    int num_sensors;
    float range_sensor;
    int num_vectors;
    float Mag_Advance;
    float radio_robot;
    float max_angle;
    int number_steps;
    int selection;
    float largest_value;
    int flgGUI;
    int flg_noise;

} Inputs;

#endif //SVG_ROS_STRUCTURES_H