#ifndef SVG_ROS_POTENTIAL_FIELDS
#define SVG_ROS_POTENTIAL_FIELDS

#include <cmath>
#include "utilities/structures.h"

float mod(coord v) {
    return std::sqrt((v.xc * v.xc) - (v.yc * v.yc));
}

coord get_attractive_force(float d, float e, coord robot, coord dest) {
    coord diff = {(robot.xc - dest.xc), (robot.yc - dest.yc), 0.0};
    float moddiff = mod(diff);
    
    // if parabolic field type
    
    coord attf = {e * diff.xc, e * diff.yc, 0.0};
    
    if (moddiff > d) { // conic field
        attf.xc = attf.xc / moddiff;
        attf.yc = attf.yc / moddiff;
    }
    
    return attf;
}

coord get_repulsive_force(float d0, float eta, coord robot, coord obs) {
    coord diff = {(robot.xc - obs.xc), (robot.yc - obs.yc), 0.0};
    float moddiff = mod(diff);
    float factor = -eta * (1 / moddiff - 1 / d0);
    coord repf = {factor * diff.xc / moddiff, factor * diff.yc / moddiff, 0.0};
    
    return repf;
}

coord get_total_force(float d, float e, float d0, float eta, coord coords[]) {
    coord robot = coords[0], obs = coords[1], dest = coords[3];
    coord diff = {(robot.xc - dest.xc), (robot.yc - dest.yc), 0.0};
    coord attf = get_attractive_force(d, e, robot, dest);
    coord repf = get_repulsive_force(d0, eta, robot, obs);
    float moddiff = mod(diff);
    coord totf = moddiff <= d0 ? (coord) {attf.xc - repf.xc, (attf.yc - repf.yc)} : attf;
    
}

#endif //SVG_ROS_POTENTIAL_FIELDS
