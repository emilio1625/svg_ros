#ifndef SVG_ROS_POTENTIAL_FIELDS_H
#define SVG_ROS_POTENTIAL_FIELDS_H

#include <cmath>
#include <cstdio>
#include "utilities/structures.h"

float mod(coord v) {
    return std::sqrt((v.xc * v.xc) + (v.yc * v.yc));
}

coord get_attractive_force(float d, float e, coord robot, coord dest) {
    coord diff = {(robot.xc - dest.xc), (robot.yc - dest.yc), 0.0};
    float moddiff = mod(diff);

    // parabolic field type

    coord attf = {e * diff.xc, e * diff.yc, 0.0};

    if (moddiff > d) { // conic field
        attf.xc = attf.xc / moddiff;
        attf.yc = attf.yc / moddiff;
    }

    printf("attractive force: %f, %f, mod: %f\n", attf.xc, attf.yc, mod(attf));

    return attf;
}

coord get_repulsive_force(float d0, float eta, coord robot, coord obs) {
    coord diff = {(robot.xc - obs.xc), (robot.yc - obs.yc), 0.0};
    float moddiff = mod(diff);
    float factor = -eta * (1 / moddiff - 1 / d0);
    coord repf = {factor * diff.xc / std::pow(moddiff, 3), factor * diff.yc / std::pow(moddiff, 3), 0.0};

    printf("repulsive force: %f, %f, mod: %f\n", repf.xc, repf.yc, mod(repf));

    return repf;
}

/**
 * get_obs_location
 * Calcula la posición del obstaculo más cercano
 * Toma el sensor con el valor más bajo para calcular la posición del obstáculo
 * @param num_sensors es el número de sensores
 * @param theta_sensor  es el angulo de inicio de los sensors (usualmente vale -(range_sensor/2))
 * @param range_sensor es el angulo que abarcan todos los sensores (de -(range_sensor/2) a (range_sensor/2))
 * @param observations contiene las mediciones de cada sensor
 * @return obs coordenadas del obtaculo
 */
coord get_obs_location(int num_sensors, float theta_sensor, float range_sensor, float largest_value, Raw observations, coord robot) {
    float ang_btw_sensors = range_sensor / (num_sensors - 1);
    float ang_sensor;
    int nearest_sensor_idx = 0;
    float nearest_sensor_value = LARGEST_DISTANCE_SENSORS;
    coord obs = {};
    for (int i = 0; i < num_sensors; i++) {
        if (observations.sensors[i] < nearest_sensor_value) {
            nearest_sensor_value = observations.sensors[i];
            nearest_sensor_idx = i;
        }
    }
    if (nearest_sensor_value > (largest_value * 0.9)) // 90% threshold
        nearest_sensor_value = LARGEST_DISTANCE_SENSORS;
    ang_sensor = theta_sensor + nearest_sensor_idx * ang_btw_sensors;
    //printf("ang sensor: %f\n", ang_sensor);
    obs.xc = float (robot.xc + nearest_sensor_value * cos(ang_sensor) * cos(robot.anglec));
    obs.yc = float (robot.yc + nearest_sensor_value * sin(ang_sensor) * sin(robot.anglec));
    //printf("sensor: %i, value: %f, obs(%f, %f), robot(%f, %f)\n", nearest_sensor_idx, nearest_sensor_value, obs.xc, obs.yc, robot.xc, robot.yc);
    return obs;
}

coord get_total_force(float d, float e, float d0, float eta, coord coords[]) {
    coord robot = coords[0], obs = coords[1], dest = coords[2];
    //printf("robot(%f, %f), dest(%f, %f)\n, obst(%f, %f)\n", robot.xc, robot.yc, dest.xc, dest.yc, obs.xc, obs.yc);
    coord diff = {(robot.xc - obs.xc), (robot.yc - obs.yc), 0.0};
    coord attf = get_attractive_force(d, e, robot, dest);
    coord repf = get_repulsive_force(d0, eta, robot, obs);
    //printf("attf(%f, %f), repf(%f, %f)\n, diff(%f, %f)\n", attf.xc, attf.yc, repf.xc, repf.yc, diff.xc, diff.yc);
    float moddiff = mod(diff), modtotf = 0.0;
    coord totf = moddiff <= d0 ? (coord) {attf.xc + repf.xc, attf.yc + repf.yc} : attf;
    modtotf = mod(totf);
    //printf("totf %f, %f modtof %f", totf.xc, totf.yc, modtotf);
    totf.xc = totf.xc / modtotf;
    totf.yc = totf.yc / modtotf;
    //printf("totf %f, %f", totf.xc, totf.yc);
    return totf;
}

coord get_next_position(float d, coord robot, coord totf) {
    return (coord) {robot.xc - d * totf.xc, robot.yc - d * totf.yc, 0.0};
}

AdvanceAngle get_displacement_vector(coord robot, coord next_pos) {
    AdvanceAngle displacement;
    coord diff = {next_pos.xc - robot.xc, next_pos.yc - robot.yc, 0.0};
    displacement.distance = mod(diff);
    displacement.angle = std::atan2(diff.yc, diff.xc) - robot.anglec;

    return displacement;
}

/*void print_inputs(Inputs *inputs) {
    printf("Inputs:\n");
    printf("path: %s \n", inputs->path);
    printf("file: %s \n", inputs->file);
    printf("environment: %s \n", inputs->environment);
    printf("sensor: %s \n", inputs->sensor);
    printf("xo: %f \n", inputs->xo);
    printf("yo: %f \n", inputs->yo);
    printf("xd: %f \n", inputs->xd);
    printf("yd: %f \n", inputs->yd);
    printf("angle_robot: %f \n", inputs->angle_robot);
    printf("theta_sensor: %f \n", inputs->theta_sensor);
    printf("num_sensors: %i \n", inputs->num_sensors);
    printf("range_sensor: %f \n", inputs->range_sensor);
    printf("num_vectors: %i \n", inputs->num_vectors);
    printf("Mag_Advance: %f \n", inputs->Mag_Advance);
    printf("radio_robot: %f \n", inputs->radio_robot);
    printf("max_angle: %f \n", inputs->max_angle);
    printf("number_steps: %i \n", inputs->number_steps);
    printf("selection: %i \n", inputs->selection);
    printf("largest_value: %f \n", inputs->largest_value);
    printf("flgGUI: %i \n", inputs->flgGUI);
    printf("flg_noise: %i \n", inputs->flg_noise);


}*/
/*
void print_observations(Raw *observations) {
    printf("Observations :\n");
    printf("flag: %i \n", observations->flag);
    printf("region: %i \n", observations->region);
    printf("theta: %f \n", observations->theta);
    printf("x: %f \n", observations->x);
    printf("y: %f \n", observations->y);
    for (int i = 0; i < 5; i++) {
        printf("sensor %i: %f \n", i, observations->sensors[i]);
    }
}*/

#endif //SVG_ROS_POTENTIAL_FIELDS_H
