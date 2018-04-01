//
// Created by emilio1625 on 13/03/18.
//

#ifndef SVG_ROS_STATE_MACHINE_FOLLOW_WALL_H
#define SVG_ROS_STATE_MACHINE_FOLLOW_WALL_H

// State Machine
AdvanceAngle state_machine_follow_wall(int obs, int state, int *next_state,
                                       float Mag_Advance, float max_angle) {

    AdvanceAngle gen_vector;
    printf("obs: %d\n", obs);

    switch (state) {
        case 0:
            gen_vector = generate_output(FORWARD, Mag_Advance, max_angle);
            printf("Present State: %d FORWARD\n", state);
            if (obs != 0)
                *next_state = 1;
            else
                *next_state = 0;
            break;
        case 1:
            gen_vector = generate_output(BACKWARD, Mag_Advance, max_angle);
            printf("Present State: %d BACKWARD\n", state);
            *next_state = 2;
            break;
        case 2:
            gen_vector = generate_output(RIGHT, Mag_Advance, max_angle);
            printf("Present State: %d RIGHT\n", state);
            *next_state = 0;
            break;
        default:
            printf("State %d not defined used", state);
            gen_vector = generate_output(STOP, Mag_Advance, max_angle);
            next_state = 0;
            break;
    }

    printf("Next State: %d\n", *next_state);
    return gen_vector;
}

#endif //SVG_ROS_STATE_MACHINE_FOLLOW_WALL_H
