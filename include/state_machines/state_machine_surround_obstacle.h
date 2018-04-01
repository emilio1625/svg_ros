//
// Created by emilio1625 on 13/03/18.
//

#ifndef SVG_ROS_STATE_MACHINE_SURROUND_OBSTACLE_H
#define SVG_ROS_STATE_MACHINE_SURROUND_OBSTACLE_H


// State Machine
AdvanceAngle state_machine_surround_obstacle(int obs, int state, int *next_state, float Mag_Advance, float max_angle) {

    AdvanceAngle gen_vector;
    // printf("Variable obs: %d\n", obs); // Para depurar valor de obs
    switch (state) {
        case 0:
            gen_vector = generate_output(FORWARD, Mag_Advance, max_angle);
            printf("Present State: %d Forward\n", state);
            if (obs == 3)
                *next_state = 1;
            else
                *next_state = 0;
            break;
        case 1:
            gen_vector = generate_output(LEFT, Mag_Advance, max_angle);
            printf("Present State: %d LEFT\n", state);
            if ((obs & 2) == 2)
                *next_state = 1;
            else
                *next_state = 2;
            break;
        case 2:
            gen_vector = generate_output(FORWARD, Mag_Advance, max_angle);
            printf("Present State: %d FORWARD\n", state);
            if ((obs & 1) == 0)
                *next_state = 3;
            else if (obs == 1)
                *next_state = 2;
            else
                *next_state = 1;
            break;
        case 3:
            gen_vector = generate_output(RIGHT, Mag_Advance, max_angle);
            printf("Present State: %d RIGHT\n", state);
            *next_state = 4;
            break;
        case 4:
            gen_vector = generate_output(FORWARD, Mag_Advance, max_angle);
            printf("Present State: %d FORWARD\n", state);
            if ((obs & 1) == 1)
                *next_state = 2;
            else
                *next_state = 3;
            break;
        default:
            printf("DEFAULT VALUE!!!!!!\n");
    }

    printf("Next State: %d\n", *next_state);
    return gen_vector;

}


#endif //SVG_ROS_STATE_MACHINE_SURROUND_OBSTACLE_H
