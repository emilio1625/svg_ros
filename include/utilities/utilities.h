#ifndef SVG_ROS_UTILITIES_H
#define SVG_ROS_UTILITIES_H
/****************************************************
* utilities.h                                       *
*                         			    *
*                               J. Savage           *
*                                                   *
*                               FI-UNAM 2015        *
*****************************************************/

#include <vector>
#include <string>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "svg_ros/LightSrv.h"
#include "svg_ros/MVServ.h"
#include "svg_ros/SensorSrv.h"
#include "svg_ros/ReadySrv.h"
#include "svg_ros/LightRealSrv.h"

int ready_gui = 0;


//string to values
int get_mv_params(std::string str, float *a, float *b) {
    using namespace boost::algorithm;
    std::vector<std::string> tokens;

    split(tokens, str, is_any_of(" ")); // here it is

    *a = atof(tokens.at(1).c_str());
    *b = atof(tokens.at(2).c_str());
    return 0;
}


int string_to_gui(char *s) {
    //Sends string s to gui
    //ros::NodeHandle n;
    //ros::ServiceClient client = n.serviceClient<svg_ros::MVServ> ("mv_gui_comm");
    //svg_ros::MVServ srv;
    //srv.request.param=s;

    static int flg = 1;
    static ros::NodeHandle n;
    static ros::ServiceClient client;
    static svg_ros::MVServ srv;


    if (flg == 1) {

        client = n.serviceClient<svg_ros::MVServ>("mv_gui_comm");
        flg = 0;
    }


    srv.request.param = s;
    if (client.call(srv)) {
        //ROS_INFO("success string_to_gui: %s", srv.response.answer.c_str());
        //ROS_INFO("success string_to_gui");
        //std::cout << srv.response.answer.c_str() << std::endl;
        return 0;
    } else {
        ROS_ERROR("Failed to call service mv_gui_comm");
        printf("Failed to call service mv_gui_comm");
        return 1;
    }
}
//unused at the moment:
// int  get_mv_params2(std::string str,float *a, float* b){
//   float x,y;
//   char * cstr = new char [str.length()+1];
//   std::strcpy (cstr, str.c_str());
//   // cstr now contains a c-string copy of str
//   char * p = std::strtok (cstr," ");
//   int i=0;
//   while (p!=0 && i<=2)
//   {
//     //std::cout << p << '\n';
//     p = std::strtok(NULL," ");
//     if(i==1){
//       x=atof(p);
//     }
//     else if(i==2){
//       y=atof(p);
//     }
//     i++;
//   }
//   delete[] cstr;
//   *a=x;
//   *b=y;
//   return 0;
// }

//communication with robot's base 
int ROS_mvrobot(std::string s, float *answer_distance, float *answer_angle) {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<svg_ros::MVServ>("move_robot");
    svg_ros::MVServ srv;

    srv.request.param = s;
    if (client.call(srv)) {
        //ROS_INFO("success: %s", srv.response.answer.c_str());
        //std::cout << srv.response.answer.c_str() << std::endl;
    } else {
        ROS_ERROR("Failed to call service move_robot");
        printf("Failed to call service move_robot");
        return 1;
    }
    get_mv_params(srv.response.answer.c_str(), answer_distance, answer_angle);
    //printf("xxx mv params: %f %f\n", *answer_distance,*answer_angle);


    return 0;

}

int ROS_mvrobot_turtle(std::string s, float *answer_distance, float *answer_angle) {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<svg_ros::MVServ>("move_robot_turtle");
    svg_ros::MVServ srv;

    srv.request.param = s;
    if (client.call(srv)) {
        //ROS_INFO("success: %s", srv.response.answer.c_str());
        //std::cout << srv.response.answer.c_str() << std::endl;
    } else {
        ROS_ERROR("Failed to call service move_robot");
        printf("Failed to call service move_robot turtle");
        return 1;
    }
    get_mv_params(srv.response.answer.c_str(), answer_distance, answer_angle);
    //printf("xxx mv params: %f %f\n", *answer_distance,*answer_angle);


    return 0;

}


// it gets data from the simulator or the real robot
int GetObservations(coord *coord_robot, Raw *observations, float largest_value) {

    static ros::NodeHandle n;
    static ros::ServiceClient client;
    static svg_ros::SensorSrv srv;
    static int flg = 1;

    if (flg == 1) {

        client = n.serviceClient<svg_ros::SensorSrv>("send_position");
        flg = 0;
    }


    srv.request.coord_x = coord_robot->xc;
    srv.request.coord_y = coord_robot->yc;
    srv.request.coord_ang = coord_robot->anglec;
    //printf("\n send_position x %f y %f angle %f\n",coord_robot->xc,coord_robot->yc,coord_robot->anglec);

    if (client.call(srv)) {
        //ROS_INFO("success send position: %s", srv.response.sensor.c_str());
    } else {
        ROS_ERROR("Failed to call service send_position");
        return 1;
    }
    observations->flag = srv.response.flag;
    observations->region = srv.response.region;
    observations->x = srv.response.x;
    observations->y = srv.response.y;
    observations->theta = srv.response.theta;

    for (int j = 0; j < srv.response.num_sensors; j++) {
        if (srv.response.sensors[j] > largest_value)
            observations->sensors[j] = largest_value;
        else
            observations->sensors[j] = srv.response.sensors[j];
        printf("observations->sensors[%d] %f srv.response.sensors[%d] %f\n", j, observations->sensors[j], j,
               srv.response.sensors[j]);
    }

    //printf("Answer GetObservations: %s \n", srv.response.sensor.c_str());

    return 0;

}

// it send the robot position to the base module either the simulator or the real robot
int SendPositionMVServer(coord coord_robot) {

    static ros::NodeHandle n;
    static ros::ServiceClient client;
    static svg_ros::MVServ srv;
    static int flg = 1;

    if (flg == 1) {

        client = n.serviceClient<svg_ros::MVServ>("send_position_mv");
        flg = 0;
    }

    srv.request.coord_x = coord_robot.xc;
    srv.request.coord_y = coord_robot.yc;
    srv.request.coord_ang = coord_robot.anglec;
    //printf("\n send_position MVServer x %f y %f angle %f\n",coord_robot.xc,coord_robot.yc,coord_robot.anglec);

    if (client.call(srv)) {
        //ROS_INFO("success send position MVServer: %s", srv.response.answer.c_str());
    } else {
        ROS_ERROR("Failed to call service send_position");
        return 1;
    }

    //printf("Answer SendPositionMVServer: %s \n", srv.response.answer.c_str());

    return 0;

}


//gets destination coordinates and quantized values via ROS communication with light_node
int get_destination(coord *coord_robot, coord *coord_dest, int *quantized_intensity = NULL,
                    int *quantized_attraction = NULL) {

    //ros::NodeHandle n;
    //ros::ServiceClient client = n.serviceClient<svg_ros::LightSrv> ("send_destination");
    //svg_ros::LightSrv srv;

    static int flg = 1;
    static ros::NodeHandle n;
    static ros::ServiceClient client;
    static svg_ros::LightSrv srv;

    if (flg == 1) {

        client = n.serviceClient<svg_ros::LightSrv>("send_destination");
        flg = 0;
    }

    srv.request.req = 1;
    srv.request.coord_x = coord_robot->xc;
    srv.request.coord_y = coord_robot->yc;
    srv.request.coord_ang = coord_robot->anglec;
    if (client.call(srv)) {
        //ROS_INFO("success send_destination: %f %f", srv.response.x,srv.response.y);
    } else {
        ROS_ERROR("Failed to call service send_destination");
        return 1;
    }
    //either get dest coords or quantized values
    /*if(srv.response.flag_dest){
      coord_dest->xc=srv.response.x;
      coord_dest->yc=srv.response.y;
    }else{*/
    *quantized_intensity = srv.response.quantized_intensity;
    *quantized_attraction = srv.response.quantized_attraction;
    //}

    //printf("Answer get_destination light sensor: %d %d\n", srv.response.quantized_attraction,srv.response.quantized_intensity);

    return 0;

}


int get_destination_real(int *quantized_intensity = NULL, int *quantized_attraction = NULL) {

    //ros::NodeHandle n;
    //ros::ServiceClient client = n.serviceClient<svg_ros::LightSrv> ("send_destination");
    //svg_ros::LightSrv srv;

    static int flg = 1;
    static ros::NodeHandle n;
    static ros::ServiceClient client;
    static svg_ros::LightRealSrv srv;

    if (flg == 1) {

        client = n.serviceClient<svg_ros::LightRealSrv>("Light_Srv");
        flg = 0;
    }

    srv.request.req = 1;
    if (client.call(srv)) {
        //ROS_INFO("success send_destination: %f %f", srv.response.x,srv.response.y);
    } else {
        ROS_ERROR("Failed to call service LightSrv");
        return 1;
    }

    *quantized_intensity = srv.response.quantized_intensity;
    *quantized_attraction = srv.response.quantized_attraction;

    return 0;

}

// it writes the sensors' readings to be plot by Tk
int write_obs_sensor(FILE *fpw, Raw observations, char *sensor, int num_sensors, float start_angle, float range) {

    int j;
    char aux[250], aux1[4];
    fprintf(fpw, "( sensor %s %d %f %f", sensor, num_sensors, range, start_angle);
    sprintf(aux, "( sensor %s %d %f %f", sensor, num_sensors, range, start_angle);

    for (j = 0; j < num_sensors; j++) {
        char aux2[20];
        fprintf(fpw, " %f", observations.sensors[j]);
        sprintf(aux2, " %f", observations.sensors[j]);
        strcat(aux, aux2);
        printf(" %f", observations.sensors[j]);
    }

    fprintf(fpw, " )\n");
    sprintf(aux1, " )\n");
    strcat(aux, aux1);

    //send plotting data to gui
    //printf("send plotting data to gui %s",aux);

    string_to_gui(aux);

    return (1);

}


// it writes the sensors' readings and position to be plot by Tk
int write_obs_sensor_pos(FILE *fpw, Raw observations, char *sensor, int num_sensors, float start_angle, float range,
                         coord coord_robot) {

    int j;
    char aux[2000], aux1[4];
    fprintf(fpw, "( sensor %s %d %f %f", sensor, num_sensors, range, start_angle);
    sprintf(aux, "( sensor %s %f %f %f %d %f %f", sensor, coord_robot.xc, coord_robot.yc, coord_robot.anglec,
            num_sensors, range, start_angle);


    for (j = 0; j < num_sensors; j++) {
        char aux2[20];
        fprintf(fpw, " %f", observations.sensors[j]);
        sprintf(aux2, " %f", observations.sensors[j]);
        strcat(aux, aux2);
        //printf(" %f",observations.sensors[j]);
    }

    fprintf(fpw, " )\n");
    sprintf(aux1, " )\n");
    strcat(aux, aux1);

    //send plotting data to gui
    //printf("send plotting data to gui %s",aux);

    string_to_gui(aux);

    return (1);

}


float magnitude(coord vector) {

    float magnitude;

    magnitude = (float) sqrt((vector.xc * vector.xc) + (vector.yc * vector.yc));
    //printf("magnitude: %f\n",magnitude);
    return (magnitude);
}


coord dif_vectors(coord vector1, coord vector2) {

    coord dif;

    dif.xc = vector1.xc - vector2.xc;
    dif.yc = vector1.yc - vector2.yc;
    return (dif);
}


/*
 // It quantizes the intensity 
int quantize_intensity(coord coord_robot,coord coor_destination){

 int value=0;
 coord attraction_force;
 float mag;

 attraction_force=dif_vectors(coord_robot,coor_destination);
 mag=magnitude(attraction_force);

 if(mag > K_INTENSITY){
        value = 0;
 }
 else{
         value = 1;
 }

 return value;
}
*/


// it gets the range from the laser sensors
void get_average_sensor(Raw observations, int start, int end, float *average) {
    int i;
    float sd = 0;

    //printf("range %d %d\n",start,end);
    for (i = start; i < end; i++) {
        sd = sd + observations.sensors[i];
        //printf("%d %f %f\n",i,sd,observations.sensors[i]); 
    }

    *average = sd / (float) (end - start);
    //printf(" average %f\n\n",*average);
}


// It quantizes the inputs
int quantize_inputs(Raw observations, int size) {

    int value = 0;
    int i;
    float left, right;
    int interval;


    interval = size / 2;
    get_average_sensor(observations, interval, size, &left);
    //printf("left sensor %f\n",left);
    get_average_sensor(observations, 0, interval, &right);
    //printf("right sensor %f\n",right);

    if (left < THRS_SENSOR) value = (value << 1) + 1;
    else value = (value << 1) + 0;

    if (right < THRS_SENSOR) value = (value << 1) + 1;
    else value = (value << 1) + 0;

    //printf("value %d\n",value);
    //value = value & 0xFFFFFFFF;
    //printf("value %x\n",value);

    return (value);

}

void get_average_sensor_real(Raw observations, int start, int end, float *average, float largest_value) {
    int i;
    float sd = 0;
    float distancia = largest_value;
    float bad_lecture = 0;
    //printf("range %d %d\n",start,end);
    for (i = start; i < end; i++) {

        //      printf("%f, ",observations.sensors[i] );
        if (observations.sensors[i] >= distancia)
            sd = sd + distancia;
        else if (observations.sensors[i] == 0) // Checks if a value is  NaN
            bad_lecture++;
        else
            sd = sd + observations.sensors[i];
        //printf("%d %f %f\n",i,sd,observations.sensors[i]); 
    }

    *average = sd / (((float) (end - start)) - bad_lecture);

// printf("\n%f/( (%d-%d) - %f ) = AVG %f  \n",sd,end,start,bad_lecture,*average );

    //printf(" average %f\n\n",*average);
/**average = 0;
  for(i=start; i < end;i++){
        if(observations.sensors[i] <= .3 )
          {
            count++;

            *average = 1;
            break;
          }
        //printf("%d %f %f\n",i,sd,observations.sensors[i]); 
 }*/


}


// It quantizes the inputs
int quantize_inputs_real(Raw observations, int size, float largest_value) {

    int value = 0;
    int i;
    float left, right;
    int interval;

    interval = size / 2;
    get_average_sensor_real(observations, interval, size, &left, largest_value);
    //printf("left sensor %f\n",left);
    get_average_sensor_real(observations, 0, interval, &right, largest_value);
    //printf("right sensor %f\n",right);

    //if( left < THRS_SENSOR) value = (value << 1) + 1;
    if ((left * 100 / largest_value) <= 98) value = (value << 1) + 1;
    else value = (value << 1) + 0;

// if( right < THRS_SENSOR) value = (value << 1) + 1;
    if ((right * 100 / largest_value) <= 98) value = (value << 1) + 1;
    else value = (value << 1) + 0;

    //printf("value %d\n",value);
    value = value & 0xFFFFFFFF;
    //printf("value %x\n",value);

    return (value);

}


// This function is used to calculate the rotation angle for the Mvto command
float get_angle(float ang, float c, float d, float X, float Y) {
    float x, y;

    x = c - X;
    y = d - Y;
    if ((x == 0) && (y == 0)) return (0);
    if (fabs(x) < 0.0001) return ((float) ((y < 0.0f) ? 3 * PI / 2 : PI / 2) - ang);
    else {
        if (x >= 0.0f && y >= 0.0f) return ((float) (atan(y / x) - ang));
        else if (x < 0.0f && y >= 0.0f) return ((float) (atan(y / x) + PI - ang));
        else if (x < 0.0f && y < 0.0f) return ((float) (atan(y / x) + PI - ang));
        else return ((float) (atan(y / x) + 2 * PI - ang));
    }
}


coord divide_vector_scalar(coord vector1, float cnt) {

    coord div;

    div.xc = vector1.xc / cnt;
    div.yc = vector1.yc / cnt;
    return (div);
}


// it gets an unit vector
coord get_unit_vector(Behavior Bvector) {

    float mag;
    coord unit_vector, vector;

    unit_vector.xc = 0;
    unit_vector.yc = 0;

    vector.xc = Bvector.xc;
    vector.yc = Bvector.yc;

    mag = magnitude(vector);

    if (mag != 0)
        unit_vector = divide_vector_scalar(vector, mag);

    return unit_vector;

}


// it returns the distance to closest poligon from the robot
int shs_distance_obstacle(float x1, float y1, float x2, float y2, float dist_advance) {

    int index;
    float distance;


    index = 0;
    robot_sensor.vertex[1].x = x1;
    robot_sensor.vertex[1].y = y1;
    robot_sensor.vertex[2].x = x2 + 0.0001;
    robot_sensor.vertex[2].y = y2 + 0.0001;

    //printf("check x1 %f y1 %f\n",robot_sensor.vertex[1].x,robot_sensor.vertex[1].y);
    ////printf("check x2 %f y2 %f\n",robot_sensor.vertex[2].x,robot_sensor.vertex[2].y);

    //m=(y2-y1)/(x2-x1);
    //b=(x2*y1-y2*x1)/(x2-x1);
    robot_sensor.line[1].m = (robot_sensor.vertex[2].y - robot_sensor.vertex[1].y) /
                             (robot_sensor.vertex[2].x - robot_sensor.vertex[1].x);
    robot_sensor.line[1].b = (robot_sensor.vertex[2].x * robot_sensor.vertex[1].y -
                              robot_sensor.vertex[2].y * robot_sensor.vertex[1].x) /
                             (robot_sensor.vertex[2].x - robot_sensor.vertex[1].x);




    // it finds the distance to the closest polygon
    distance = distance_polygon(num_polygons_wrl, polygons_wrl, robot_sensor, &index);

    //printf("distance_polygon %d  polygon name %s distance %f\n",index,polygons_wrl[index].name,distance);
    //printf("** check distance closests %f distance advance %f **\n",distance,dist_advance);

    if (distance > dist_advance) {
        return 0;
    } else {
        return 1;
    }

}


// it will move the robot the desire angle and distance
void mvrobot(AdvanceAngle DistTheta, coord *coord_robot, int flag_environment) {
    int flg = 0;
    float new_xmv, new_ymv, new_thetamv;
    float xc, yc;
    float speed;
    coord new_coord;
    float xmv, ymv, thetamv;
    float distance, angle1;
    float answer_distance;
    float answer_angle;
    char param[20];

    xmv = coord_robot->xc;
    ymv = coord_robot->yc;
    thetamv = coord_robot->anglec;

    angle1 = DistTheta.angle;
    //printf("angle1 %f\n",angle1);
    distance = DistTheta.distance;

    if (angle1 > 5.75f) angle1 = -(angle1 - 5.75f);

    sprintf(param, "mv %f %f", distance, angle1);

    if (flag_environment)
        ROS_mvrobot(param, &answer_distance, &answer_angle);
    else
        ROS_mvrobot_turtle(param, &answer_distance, &answer_angle);

    angle1 = answer_angle;
    distance = answer_distance;


    new_thetamv = thetamv + angle1;
    if (new_thetamv > 6.2832) new_thetamv = new_thetamv - (float) 6.2832;
    else if (new_thetamv < -0.0) new_thetamv = new_thetamv + (float) 6.2832;

    new_xmv = xmv + (float) distance * (float) cos((float) (new_thetamv));
    new_ymv = ymv + (float) distance * (float) sin((float) (new_thetamv));

    // it checks if the robot new position is inside an obstacle
    //flg = shs_distance_obstacle(xmv,ymv,new_xmv,new_ymv,.5);
    //printf("check inside %d x:%.3f, y:%.3f,  rad:%.3f\n",flg,new_xmv,new_ymv,new_thetamv);

    //if(flg==0){
    coord_robot->xc = new_xmv;
    coord_robot->yc = new_ymv;
    coord_robot->anglec = new_thetamv;
    //}
    //else{
    //printf("The robot remains with the previous position\n");
    //}

    //printf("mv x:%f, y:%f,  rad:%f\n",coord_robot->xc,coord_robot->yc,coord_robot->anglec);
}


float distance(coord vector, coord vector1) {
//distance between coordinates
    float x, x1, y, y1;
    float distance;

    x = vector.xc;
    x1 = vector1.xc;
    y = vector.yc;
    y1 = vector1.yc;

    // it calculates the distance
    distance = (float) sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    //printf("Distance: %f\n",distance);

    return (distance);
}

// it generates a robot's output
AdvanceAngle generate_output(int out, float advance, float angle) {

    AdvanceAngle output;

    switch (out) {

        case 0: // Stop
            output.distance = 0.0f;
            output.angle = 0.0f;
            //printf("STOP\n");
            break;

        case 1: // Forward
            output.distance = advance;
            output.angle = 0.0f;
            //printf("FORWARD\n");
            break;

        case 2: // backward
            output.distance = -advance;
            output.angle = 0.0f;
            //printf("BACKWARD\n");
            break;

        case 3:// Turn left
            output.distance = 0.0f;
            output.angle = angle;
            //printf("LEFT\n");
            break;

        case 4: // Turn right
            output.distance = 0.0f;
            output.angle = -angle;
            //printf("RIGHT\n");
            break;

        default:
            printf("Output %d not defined used ", out);
            output.distance = 0.0f;
            output.angle = 0.0f;
            break;
    }

    return (output);

}

#endif //SVG_ROS_UTILITIES_H


