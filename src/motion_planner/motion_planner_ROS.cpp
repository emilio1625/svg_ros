/********************************************************************************
 *                                                                               *
 * motion_planner_ROS.cpp *
 *  =================== *
 *                                                                               *
 *  Description: * It controls the movement of a robot using state machines.
 *  It uses a real robot or a simulated one.
 *
 *                                                                               *
 *                               J. Savage *
 *                                                                               *
 *                               FI-UNAM 2017 *
 ********************************************************************************/

// System include files
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <potential_fields/arbiter.h>
// Robotics include files

#include "utilities/constants.h"
#include "utilities/structures.h"
#include "simulator/simulation.h"
#include "utilities/utilities.h"
#include "utilities/inputs.h"
#include "state_machines/state_machine_avoidance.h"
#include "state_machines/state_machine_destination.h"
#include "state_machines/state_machine_avoidance_destination.h"
#include "state_machines/state_machine_follow_wall.h"
#include "state_machines/state_machine_surround_obstacle.h"
#include "potential_fields/potential_fields.h"

#include "ros/ros.h"
#include "ROSutilities/receiveInputs.h"
#include "ROSutilities/ros_util.h"
#include "ROSutilities/over_state.h"

float K_GOAL = static_cast<float>(CNT_GOAL * MAG_ADVANCE);

// it moves the robot from the origen to the destination using state machines
int go_to(int flag_environment, int flag_laser, int flag_light) {

    Inputs inputs;
    char file_obs[250];
    char world_file[250];
    char new_str[255];
    float theta = 0;
    int j;
    int i = 0, k;
    FILE *fpw;
    Raw observations;
    int num_obs = 1;
    float distance1;
    coord coord_robot;
    coord coord_dest;
    int flg, flag;
    int debug = 0;
    float xmv, ymv, thetamv;
    int quantized_obs;
    int quantized_attraction;
    int quantized_intensity;
    AdvanceAngle mov_vector;
    int next_state;
    int next_state_avoidance = 0, next_state_destination = 0,
            next_state_avoidance_destination = 0, next_state_follow_wall = 0,
            next_state_surround_obstacle = 0;
    int state = 0;
    AdvanceAngle DistTheta;
    AdvanceAngle mov_vector_avoidance;
    AdvanceAngle mov_vector_destination;
    AdvanceAngle mov_vector_avoidance_destination;
    AdvanceAngle mov_vector_follow_wall;
    AdvanceAngle mov_vector_surround_obstacle;
    AdvanceAngle mov_vector_potential_fields;
    int selection = 3;
    float largest_value;


    // get inputs from GUI
    std::string inputs_node_name = "receiveInputs";
    ROS_INFO("Waiting data from the GUI");
    flag = 1;
    while (flag) {
        flag = receiveInputs(&inputs, "receiveInputs");
        // flag=receiveInputs(&inputs,inputs_node_name);
        usleep(10000);
    }

    strcpy(inputs.sensor, "laser");

    // it opens the observation's sensor file to be plot by the graphical
    // interface
    sprintf(file_obs, "%s%s.raw", inputs.path, inputs.environment);
    // printf("\nobservations file: %s\n",file_obs);
    if ((fpw = fopen(file_obs, "w")) == NULL) {
        printf("File %s can not be open\n", file_obs);
        return (0);
    }

    fprintf(fpw, "( radio_robot %f )\n", inputs.radio_robot);
    sprintf(new_str, "( radio_robot %f )\n", inputs.radio_robot);
    string_to_gui(new_str);
    memset(new_str, 0, sizeof new_str);
    // printf("Radio robot %f \n",inputs.radio_robot);

    coord_robot.xc = inputs.xo;
    coord_robot.yc = inputs.yo;
    coord_robot.anglec = inputs.angle_robot;
    coord_dest.xc = inputs.xd;
    coord_dest.yc = inputs.yd;
    fprintf(fpw, "( destination %f %f )\n", inputs.xd, inputs.yd);

    selection = inputs.selection;
    largest_value = inputs.largest_value;

    flg = 0;

    K_GOAL = static_cast<float>(CNT_GOAL * inputs.Mag_Advance);


    // potential fields
    int nestados = 0, stflag=0;
    float d = 0.05;
    float e = 2;
    float d0 = inputs.largest_value * 0.5f;
    float eta = 0.0005;
    float params[] = {d, e, d0, eta};
    coord coords[2];


    // it moves the robot to the final destination
    while (flg == 0) {

        printf("\n\n ********************** STEP %d "
               "**************************************\n",
               num_obs);
        printf("Pose robot x:%f y:%f angle:%f \n", coord_robot.xc, coord_robot.yc,
               coord_robot.anglec);

        // Get the position of the light source from the real robot or the simulate
        // one
        if (flag_light)
            get_destination(&coord_robot, &coord_dest, &quantized_intensity,
                            &quantized_attraction);
        else
            get_destination_real(&quantized_intensity, &quantized_attraction);

        printf("Quantized intensity %d \n", quantized_intensity);
        printf("Quantized destination %d \n", quantized_attraction);
        printf("Destination x:%f y:%f\n", coord_dest.xc, coord_dest.yc);

        distance1 = distance(coord_robot, coord_dest);
        printf("Distance destination %f threshold %f\n", distance1, K_GOAL);

        // it checks if the robot reached its destination
        if (distance1 < K_GOAL) {
            printf("Reached distance destination %f\n", distance1);
            flg = 1;
            break;
        }

        // it gets laser observations from the simulator or the real robot
        GetObservations(&coord_robot, &observations, largest_value);

        // it send the robot position to the base module either the simulator or the
        // real robot
        SendPositionMVServer(coord_robot);

        // it saves the robot's position to be plotted by the GUI
        fprintf(fpw, "( robot Justina %f %f %f )\n", coord_robot.xc, coord_robot.yc,
                coord_robot.anglec);
        sprintf(new_str, "( robot Justina %f %f %f )\n", coord_robot.xc,
                coord_robot.yc, coord_robot.anglec);
        string_to_gui(new_str); // send plotting data to gui
        memset(new_str, 0, sizeof new_str);

        // it saves the sensor data to be plot by Python/TK
        // write_obs_sensor(fpw,observations,inputs.sensor,inputs.num_sensors,inputs.theta_sensor,inputs.range_sensor);
        write_obs_sensor_pos(fpw, observations, inputs.sensor, inputs.num_sensors,
                             inputs.theta_sensor, inputs.range_sensor, coord_robot);

        if (flag_laser)
            // It quantizes the laser inputs
            quantized_obs = quantize_inputs(observations, inputs.num_sensors);
        else
            // It quantizes the real laser inputs ( the real laser has wrong data  in
            // each scan )
            quantized_obs =
                    quantize_inputs_real(observations, inputs.num_sensors, largest_value);

        // printf("quantized inputs %d\n",quantized_obs);

        // it executes the desired behaviour
        switch (selection) {

            case 1:
                // It calculates the robot's movement using an state machine avoidance
                state = next_state_avoidance;
                mov_vector_avoidance =
                        state_machine_avoidance(quantized_obs, state, &next_state_avoidance,
                                                inputs.Mag_Advance, inputs.max_angle);
                // printf("movement avoidance: angle  %f distance
                // %f\n",mov_vector.angle,mov_vector.distance);
                DistTheta.angle = mov_vector_avoidance.angle;
                DistTheta.distance = mov_vector_avoidance.distance;

                break;

            case 2:
                // It calculates the robot's movement using an state machine destination
                state = next_state_destination;
                mov_vector_destination = state_machine_destination(
                        quantized_attraction, quantized_intensity, state,
                        &next_state_destination, inputs.Mag_Advance, inputs.max_angle);

                // printf("movement destination: angle  %f distance
                // %f\n",mov_vector.angle,mov_vector.distance);
                DistTheta.angle = mov_vector_destination.angle;
                DistTheta.distance = mov_vector_destination.distance;
                break;

            case 3:
                // It calculates the robot's movement using an state machine avoidance
                // destination
                state = next_state_avoidance_destination;
                mov_vector_avoidance_destination = state_machine_avoidance_destination(
                        quantized_obs, quantized_attraction, quantized_intensity, state,
                        &next_state_avoidance_destination, inputs.Mag_Advance,
                        inputs.max_angle);
                DistTheta.angle = mov_vector_avoidance_destination.angle;
                DistTheta.distance = mov_vector_avoidance_destination.distance;
                // printf("movement avoidance destination: angle  %f distance %f\n",
                // mov_vector_avoidance_destination.angle,mov_vector_avoidance_destination.distance);
                break;

            case 4:
                // It calculates the robot's movement using an state machine made that
                // surrounds obstacles
                state = next_state_surround_obstacle;
                mov_vector_surround_obstacle = state_machine_surround_obstacle(
                        quantized_obs, state, &next_state_surround_obstacle,
                        inputs.Mag_Advance, inputs.max_angle);
                DistTheta.angle = mov_vector_surround_obstacle.angle;
                DistTheta.distance = mov_vector_surround_obstacle.distance;
                // printf("movement avoidance destination: angle  %f distance %f\n",
                // mov_vector_avoidance_destination.angle,mov_vector_avoidance_destination.distance);
                break;

            case 5:
                // It calculates the robot's movement using an state machine made that
                // follows walls
                state = next_state_follow_wall;
                mov_vector_follow_wall = state_machine_follow_wall(
                        quantized_obs, state, &next_state_follow_wall, inputs.Mag_Advance,
                        inputs.max_angle);
                DistTheta.angle = mov_vector_follow_wall.angle;
                DistTheta.distance = mov_vector_follow_wall.distance;
                // printf("movement avoidance destination: angle  %f distance %f\n",
                // mov_vector_avoidance_destination.angle,mov_vector_avoidance_destination.distance);
                break;

            case 6:
                // It calculates the robot's movement using potential fields, if fails then uses an state
                // machine made that surrounds obstacles
                coords[0] = coord_robot;
                coords[1] = coord_dest;
                state = next_state_surround_obstacle;
                mov_vector_potential_fields = gen_next_pos(params, coords, inputs, observations, &stflag);
                printf("cond 1 %i", (nestados > 0 && nestados <= 4));
                printf("cond 2 %i", ((mov_vector_potential_fields.distance == -1.0f) && (mov_vector_potential_fields.angle == -1.0f)));
                if ((nestados > 0 && nestados <= 4) || stflag) {
                    mov_vector_potential_fields = state_machine_surround_obstacle(
                            quantized_obs, state, &next_state_surround_obstacle, inputs.Mag_Advance,
                            inputs.max_angle);
                    nestados ++;
                }
                if (nestados > 4) {
                    nestados = 0;
                    stflag = 0;
                }
                printf("nestados %i\n", nestados);
                DistTheta.angle = mov_vector_potential_fields.angle;
                DistTheta.distance = mov_vector_potential_fields.distance;
                printf("dist %f, ang %f", DistTheta.distance, DistTheta.angle);
                // printf("movement avoidance destination: angle  %f distance %f\n",
                // mov_vector_avoidance_destination.angle,mov_vector_avoidance_destination.distance);
                break;

            default:
                printf("Selection %d not defined\n", selection);
                flg = 1;
                DistTheta.angle = 0.0;
                DistTheta.distance = 0.0;
                break;
        }

        // it moves the robot
        mvrobot(DistTheta, &coord_robot, flag_environment);

        write_obs_sensor_pos(fpw, observations, inputs.sensor, inputs.num_sensors,
                             inputs.theta_sensor, inputs.range_sensor, coord_robot);

        num_obs++;
        if (num_obs > inputs.number_steps)
            flg = 1;

        // usleep(10000);
    }

    fclose(fpw);
    return (num_obs);
}

// Main program
int main(int argc, char *argv[]) {

    int num_steps;
    int flg = 1;
    int flag_environment = 0;
    int flag_laser;
    int flag_light;

    if (argc == 1) {
        printf("%s\n", "Please provide an environment flag [0 real] [1 simulator]");
        return 0;
    } else {
        flag_environment = atoi(argv[2]);
        flag_laser = atoi(argv[4]);
        flag_light = atoi(argv[6]);
    }

    ros::init(argc, argv, "motion_planner_client");
    ROS_INFO("Motion planner node started");

    while (flg) {
        num_steps = go_to(flag_environment, flag_laser, flag_light);
        over(); // send IS_OVER signal
        usleep(10000);
    }

    return 0;
}
