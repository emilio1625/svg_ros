#ifndef SVG_ROS_ARBITER_H
#define SVG_ROS_ARBITER_H

#include <simulator/simulation.h>
#include "utilities/structures.h"
#include "potential_fields.h"
#include <cmath>

AdvanceAngle gen_next_pos(float params[], coord coords[], Inputs inputs, Raw observations) {
    static coord last_pos;
    float d = params[0], e = params[1], d0 = params[2], eta = params[3];
    coord robot = coords[0], dest = coords[1];
    coord obs = get_obs_location(inputs.num_sensors, inputs.theta_sensor, inputs.range_sensor, inputs.largest_value, observations, robot);
/*
    printf("obstacle: %f, %f\n", obs.xc, obs.yc);
*/
    coord coords2[3] = {robot, obs, dest};
    coord totf = get_total_force(d, e, d0, eta, coords2);
    coord next_pos = get_next_position(d, robot, totf);
    AdvanceAngle displacement = get_displacement_vector(robot, next_pos);
    /*printf("max angle: %f \n", inputs.max_angle);

    printf("displacement: %f, %f\n", displacement.distance, displacement.angle);*/

    if (displacement.distance > inputs.Mag_Advance)
        displacement.distance = inputs.Mag_Advance;

    /*printf("displacement arbiter: %f, %f\n", displacement.distance, displacement.angle);
    printf("next_pos: %f, %f\n", next_pos.xc, next_pos.yc);
    printf("last_pos: %f, %f\n", last_pos.xc, last_pos.yc);
    printf("robot_pos: %f, %f\n", robot.xc, robot.yc);
    getchar();*/

    last_pos = robot;
    return displacement;
}

#endif //SVG_ROS_ARBITER_H
