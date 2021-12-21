#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <iostream>
#include <fstream>
#include <cmath>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>

#include <msg_generate/motor_msg.h>
#include "angle.h"
#include "dxltest.h"

#define PI 3.141592653589793
#define rad2deg (double)180.0/PI
#define deg2rad PI/(double)180

using namespace std;
using namespace Eigen;

class solve
{
public:

    ros::Publisher motor_pub;
    ros::Publisher angle_pub;
    ros::Publisher dxl_pub;

    double ang2pos(double angle);
    double pos2ang(double pos);
    double to360(double deg);
    double to180(double deg);

    void init_save();
    void update_motorangle();
    void motor_packet(int speed);

    void fk_solve(double *x, double *y, double *z, double *rx, double *ry, double *rz);
    int ik_solve(double pX, double pY, double pZ, double rX, double rY, double rZ);

    // Link Length [mm]
    double L1 = 77.595;
    double L2 = 300.0;
    double L3 = 220.0;
    double L4 = 126.0;

    double box_h = 98.5;
    double toolbox_h = 165.0;


    int g_DXL_ID_position[30] = {0, }; //output

    int correct_init[7] = {0, };
    int ang6 = 0, ang7 = 0;

private:

    //unsigned int g_DXL_ID_position[30] = {0, }; //output
    int g_DXL_ID_Save_position[30] = {0, };
    double InvLegAngle[12] = {0, };

    int position[23] = {0, };

    double th[6] = {0.0, };
    double _th[6] = {0.0, };

    //angle limit        0      1       2       3       4       5
    int Ang_Lm_max[6] = {-180,  -150,   -170,   -180,   100,    -180};
    int Ang_Lm_min[6] = {180,   -30,    -50,    180,    -140,   180};
};


#endif /* KINEMATICS_HPP_ */
