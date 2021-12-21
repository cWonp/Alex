#ifndef MOVE_ALEX_HPP_
#define MOVE_ALEX_HPP_

#include "ros/ros.h"
#include <iostream>
#include "cmath"
#include <eigen3/Eigen/Dense>
#include <fstream>

#include "spline.hpp"
#include "kinematics.hpp"

//..msg........................................................
#include "order.h"
#include "coortest.h"
#include "move2order.h"
#include <msg_generate/motor_msg.h>

ros::Subscriber order_sub;
ros::Publisher coor_pub;
ros::Publisher move2order_pub;

//..motion_type..............................................
#define PICK 0
#define MOVE 1
#define TOOL_CHANGE 2
#define TOOL 3

#define TOOL1 0
#define TOOL2 1
#define TOOL3 2
#define TOOL4 3

#define OPEN -43
#define CLOSE 299
//..var......................................................

solve kinematics;

struct pos
{
    double pX = 0;
    double pY = 0;
    double pZ = 0;
};

struct rot
{
    double rX = 0;
    double rY = 0;
    double rZ = 0;
};

struct parameters
{
    bool onoff = false;
    double ent = 500.0;

    //init
    double default_X = kinematics.L3;
    double default_Y = 0.0;
    double default_Z = kinematics.L1 + kinematics.L2 - kinematics.L4;
    double default_rX = 180.0;
    double default_rY = 0.0;
    double default_rZ = 180.0;


};

//..class run..................................................
class run : public spline
{
public:

    void run_robot(int t);
    void get_parameter();

    pos pos_from; //start coordinate
    pos pos_to; //end coordinate

    rot rot_from;
    rot rot_to;

    int motion_type = 0;
    int num_t = 0;
    int num_t_p = 0;

    spline X_pattern;   spline Y_pattern;   spline Z_pattern;
    spline rX_pattern;  spline rY_pattern;  spline rZ_pattern;
    spline ang6_pattern;

    int X_tool[4] = {-154, -154, -154, -154};
    int Y_tool[4] = {123, 47, -47, -123};
    int Z_tool[4] = {kinematics.toolbox_h - kinematics.box_h,
                     kinematics.toolbox_h - kinematics.box_h,
                     kinematics.toolbox_h - kinematics.box_h,
                     kinematics.toolbox_h - kinematics.box_h};

private:

    pos pos_now;
    rot rot_now;

    pos pos_via;

    bool via_point = false;
};

//..var..........................................................
parameters param;
run ALEX;

move_ALEX::move2order moveinfo;


//..func.......................................................
void order_callback(const order_ALEX::order::ConstPtr& msg);


#endif /* MOVE_ALEX_HPP_ */
