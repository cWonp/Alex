#include "../include/move_ALEX/move_ALEX.hpp"
using namespace std;

void timer_callback(const ros::TimerEvent&)
{
    static double t = 0.0;

    if(param.onoff)
    {
        ALEX.run_robot(t);
        t++;

        if(t > param.ent)
        {
            t = 0.0;
            param.onoff = false;
            moveinfo.end_flag = true;
            move2order_pub.publish(moveinfo);
        }
    }
    else
    {
        t = 0.0;
        param.onoff = false;
        moveinfo.end_flag = true;
        move2order_pub.publish(moveinfo);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_ALEX");
    ros::NodeHandle n;

    //..msg...............
    //sub
    order_sub = n.subscribe("order", 100, order_callback);
    //pub
    coor_pub = n.advertise<move_ALEX::coortest>("coor", 100);

    kinematics.angle_pub = n.advertise<alex_test::angle>("angle", 100);
    kinematics.motor_pub = n.advertise<msg_generate::motor_msg>("Dynamixel", 100);
    move2order_pub = n.advertise<move_ALEX::move2order>("moveinfo", 100);
    kinematics.dxl_pub = n.advertise<move_ALEX::dxltest>("dxltest", 100);

    //..timer.............
    ros::Timer timer = n.createTimer(ros::Duration(0.01), timer_callback);

    kinematics.init_save();
    ALEX.get_parameter();
    sleep(1);
    kinematics.ik_solve(param.default_X, param.default_Y, param.default_Z, param.default_rX, param.default_rY, param.default_rZ);
    kinematics.update_motorangle();
    kinematics.motor_packet(1023);
    
    cout<<"-----     default   -------"<<endl;
    cout<<"param.default_X ==  "<<param.default_X<<endl;
    cout<<"param.default_Y ==  "<<param.default_Y<<endl;
    cout<<"param.default_Z ==  "<<param.default_Z<<endl;
    cout<<"param.default_rX == "<<param.default_rX<<endl;
    cout<<"param.default_rY == "<<param.default_rY<<endl;
    cout<<"param.default_rZ == "<<param.default_rZ<<endl<<endl;
    

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}

//will be changed with vision callback
void order_callback(const order_ALEX::order::ConstPtr& msg)
{
    static int temp;
    temp = ALEX.num_t;

    param.ent = msg->ent;
    param.onoff = msg->run_flag;

    ALEX.pos_to.pX = msg->pX;
    ALEX.pos_to.pY = msg->pY;
    ALEX.pos_to.pZ = msg->pZ;

    ALEX.rot_to.rX = msg->rX;
    ALEX.rot_to.rY = msg->rY;
    ALEX.rot_to.rZ = msg->rZ;

    ALEX.motion_type = msg->type;
    ALEX.num_t = msg->toolnumber;

    kinematics.correct_init[0] = msg->correct_init_0;
    kinematics.correct_init[1] = msg->correct_init_1;
    kinematics.correct_init[2] = msg->correct_init_2;
    kinematics.correct_init[3] = msg->correct_init_3;
    kinematics.correct_init[4] = msg->correct_init_4;
    kinematics.correct_init[5] = msg->correct_init_5;
    kinematics.correct_init[6] = msg->correct_init_6;

    ALEX.X_tool[0] = msg->X_Tool_0;
    ALEX.X_tool[1] = msg->X_Tool_1;
    ALEX.X_tool[2] = msg->X_Tool_2;
    ALEX.X_tool[3] = msg->X_Tool_3;

    ALEX.Y_tool[0] = msg->Y_Tool_0;
    ALEX.Y_tool[1] = msg->Y_Tool_1;
    ALEX.Y_tool[2] = msg->Y_Tool_2;
    ALEX.Y_tool[3] = msg->Y_Tool_3;

    ALEX.Z_tool[0] = msg->Z_Tool_0;
    ALEX.Z_tool[1] = msg->Z_Tool_1;
    ALEX.Z_tool[2] = msg->Z_Tool_2;
    ALEX.Z_tool[3] = msg->Z_Tool_3;

    for(int i = 0; i < 4; i++)
    {
        cout<<endl<<"ALEX.X_TOOL["<<i<<"] === "<<ALEX.X_tool[i]<<endl;
        cout<<"ALEX.Y_TOOL["<<i<<"] === "<<ALEX.Y_tool[i]<<endl;
        cout<<"ALEX.Z_TOOL["<<i<<"] === "<<ALEX.Z_tool[i]<<endl;
    }
    cout<<",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,"<<endl;

    kinematics.motor_packet(400);

    //..check the point......................................................................................
    if(kinematics.ik_solve(msg->pX, msg->pY, msg->pZ, msg->rX, msg->rY, msg->rZ) == 1)
    {
        param.onoff = false;
        cout<<endl<<"**  THETA = NAN  **"<<endl<<endl;
    }
    if(kinematics.ik_solve(msg->pX, msg->pY, msg->pZ, msg->rX, msg->rY, msg->rZ) == 2)
    {
        param.onoff = false;
        cout<<endl<<"**  ANGLE LIMIT !!  **"<<endl<<endl;
    }
    if((msg->pX < 100.0 && msg->pX > -200.0) && (fabs(msg->pY) < 127) && (msg->pZ < 2))
    {
        param.onoff = false;
        cout<<endl<<"**  BOX POINT  **"<<endl<<endl;
    }
    if((sqrt(pow(msg->pX, 2)+pow(msg->pY, 2)+pow(msg->pZ, 2)) < kinematics.L1+kinematics.L2-kinematics.L3))
    {
        param.onoff = false;
        cout<<endl<<"**  LENGTH LIMIT  **"<<endl<<endl;
    }

    if(temp != ALEX.num_t)
        ALEX.num_t_p = temp;
}

void run::get_parameter()
{
    std::ifstream is;

    is.open("/home/robit/catkin_ws/src/order_ALEX/save/save");

    is >> kinematics.correct_init[0];
    is >> kinematics.correct_init[1];
    is >> kinematics.correct_init[2];
    is >> kinematics.correct_init[3];
    is >> kinematics.correct_init[4];
    is >> kinematics.correct_init[5];
    is >> kinematics.correct_init[6];

    is >> ALEX.X_tool[0];
    is >> ALEX.X_tool[1];
    is >> ALEX.X_tool[2];
    is >> ALEX.X_tool[3];

    is >> ALEX.Y_tool[0];
    is >> ALEX.Y_tool[1];
    is >> ALEX.Y_tool[2];
    is >> ALEX.Y_tool[3];

    is >> ALEX.Z_tool[0];
    is >> ALEX.Z_tool[1];
    is >> ALEX.Z_tool[2];
    is >> ALEX.Z_tool[3];

    for(int i = 0; i < 4; i++)
    {
        cout<<endl<<"ALEX.X_TOOL["<<i<<"] === "<<ALEX.X_tool[i]<<endl;
        cout<<"ALEX.Y_TOOL["<<i<<"] === "<<ALEX.Y_tool[i]<<endl;
        cout<<"ALEX.Z_TOOL["<<i<<"] === "<<ALEX.Z_tool[i]<<endl;
    }

    is.close();
}

void run::run_robot(int t)
{
    if(t == 0.0)
    {
        vector<spline::interpolation>().swap(X_pattern.pattern);
        vector<spline::interpolation>().swap(Y_pattern.pattern);
        vector<spline::interpolation>().swap(Z_pattern.pattern);
        vector<spline::interpolation>().swap(rX_pattern.pattern);
        vector<spline::interpolation>().swap(rY_pattern.pattern);
        vector<spline::interpolation>().swap(rZ_pattern.pattern);
        vector<spline::interpolation>().swap(ang6_pattern.pattern);

        //..get from point...............................................................................
        kinematics.fk_solve(&pos_from.pX, &pos_from.pY, &pos_from.pZ, &rot_from.rX, &rot_from.rY, &rot_from.rZ);

        moveinfo.pX = pos_from.pX;
        moveinfo.pY = pos_from.pY;
        moveinfo.pZ = pos_from.pZ;
        moveinfo.rX = rot_from.rX;
        moveinfo.rY = rot_from.rY;
        moveinfo.rZ = rot_from.rZ;
        moveinfo.end_flag = false;

        move2order_pub.publish(moveinfo);

        //..make via point to avoid origin................................................................
        if((roundf(pos_from.pX) == roundf(pos_to.pX)) && (roundf(pos_from.pY) == roundf(pos_to.pY)) || ((pos_to.pX*pos_from.pX >= 0) && (pos_to.pY*pos_from.pY >= 0)))
        {
            via_point = false;
        }
        else
        {
            double y = (((pos_to.pY - pos_from.pY)/(pos_to.pX - pos_from.pX)) * (-pos_from.pX)) + pos_from.pY;
            double x = pos_from.pX - pos_from.pY * ((pos_to.pX - pos_from.pX)/(pos_to.pY - pos_from.pY));

            if((y > -50.0 && y < 50.0) || (x > -50.0 && x < 50.0))
            {
                via_point = true;

                double th = atan2(pos_to.pY, pos_to.pX);
                double alpha;
                double length;

                Matrix3d Rz;
                Vector3d P_to, P_from, P_via, P_to_1, P_from_1, P_via_1;

                Rz << cos(-th), -sin(-th), 0,
                        sin(-th), cos(-th), 0,
                        0, 0, 1;

                cout<<Rz<<endl;

                P_to << pos_to.pX,
                        pos_to.pY,
                        0;

                P_from << pos_from.pX,
                        pos_from.pY,
                        0;

                P_to_1 = Rz*P_to;
                P_from_1 = Rz*P_from;

                cout<<P_from_1<<endl;

                alpha = atan2(P_from_1(1), P_from_1(0))/2;

                length = (sqrt(pow(pos_to.pX, 2) + pow(pos_to.pY, 2)) + sqrt(pow(pos_from.pX, 2) + pow(pos_from.pY, 2))) / 2;

                P_via_1 << length*cos(alpha),
                        length*sin(alpha),
                        0;

                Rz << cos(th), -sin(th), 0,
                        sin(th), cos(th), 0,
                        0, 0, 1;

                P_via = Rz*P_via_1;

                pos_via.pX = P_via(0);
                pos_via.pY = P_via(1);
                pos_via.pZ = (pos_to.pZ + pos_from.pZ)/2;
            }
            else
            {
                via_point = false;
            }
        }

        switch(motion_type)
        {
        case PICK:{
            X_pattern.put_point(0.0, pos_from.pX, 0.0, 0.0, Linear);
            if(via_point) X_pattern.put_point(param.ent*0.5, pos_via.pX, 0.0, 0.0, Linear);
            X_pattern.put_point(param.ent, pos_to.pX, 0.0, 0.0, Linear);

            Y_pattern.put_point(0.0, pos_from.pY, 0.0, 0.0, Linear);
            if(via_point) Y_pattern.put_point(param.ent*0.5, pos_via.pY, 0.0, 0.0, Linear);
            Y_pattern.put_point(param.ent, pos_to.pY, 0.0, 0.0, Linear);

            Z_pattern.put_point(0.0, pos_from.pZ, 0.0, 0.0, Linear);
            if(via_point) Z_pattern.put_point(param.ent*0.5, pos_via.pZ, 0.0, 0.0, Linear);
            Z_pattern.put_point(param.ent*0.75, pos_to.pZ + 70, 0.0, 0.0, Fifth);
            Z_pattern.put_point(param.ent, pos_to.pZ, 0.0, 0.0, Linear);

            rX_pattern.put_point(0.0, rot_from.rX, 0.0, 0.0, Linear);
            rX_pattern.put_point(param.ent, rot_to.rX, 0.0, 0.0, Linear);

            rY_pattern.put_point(0.0, rot_from.rY, 0.0, 0.0, Linear);
            rY_pattern.put_point(param.ent, rot_to.rY, 0.0, 0.0, Linear);

            rZ_pattern.put_point(0.0, rot_from.rZ, 0.0, 0.0, Linear);
            rZ_pattern.put_point(param.ent, rot_to.rZ, 0.0, 0.0, Linear);
            break;}

        case MOVE:{
            X_pattern.put_point(0.0, pos_from.pX, 0.0, 0.0, Linear);
            if(via_point) X_pattern.put_point(param.ent*0.5, pos_via.pX, 0.0, 0.0, Linear);
            X_pattern.put_point(param.ent, pos_to.pX, 0.0, 0.0, Linear);

            Y_pattern.put_point(0.0, pos_from.pY, 0.0, 0.0, Linear);
            if(via_point) Y_pattern.put_point(param.ent*0.5, pos_via.pY, 0.0, 0.0, Linear);
            Y_pattern.put_point(param.ent, pos_to.pY, 0.0, 0.0, Linear);

            Z_pattern.put_point(0.0, pos_from.pZ, 0.0, 0.0, Linear);
            if(via_point) Z_pattern.put_point(param.ent*0.5, pos_via.pZ, 0.0, 0.0, Linear);
            Z_pattern.put_point(param.ent, pos_to.pZ, 0.0, 0.0, Linear);

            rX_pattern.put_point(0.0, rot_from.rX, 0.0, 0.0, Linear);
            rX_pattern.put_point(param.ent, rot_to.rX, 0.0, 0.0, Linear);

            rY_pattern.put_point(0.0, rot_from.rY, 0.0, 0.0, Linear);
            rY_pattern.put_point(param.ent, rot_to.rY, 0.0, 0.0, Linear);

            rZ_pattern.put_point(0.0, rot_from.rZ, 0.0, 0.0, Linear);
            rZ_pattern.put_point(param.ent, rot_to.rZ, 0.0, 0.0, Linear);
            break;}

        case TOOL_CHANGE:{

            if((num_t < 2 && num_t_p > 1) || (num_t_p < 2 && num_t > 1))
                via_point = true;
            else
                via_point = false;

            //..x.....................................................................................................
            //   1. now
            X_pattern.put_point(param.ent*0.00, pos_from.pX, 0.0, 0.0, Linear); // 1. now           xyz
            Y_pattern.put_point(param.ent*0.00, pos_from.pY, 0.0, 0.0, Linear); // 1. now           xyz
            Z_pattern.put_point(param.ent*0.00, pos_from.pZ, 0.0, 0.0, Linear); // 1. now           xyz
            //   2. Y Init
            X_pattern.put_point(param.ent*0.05, 0.0        , 0.0, 0.0, Linear); // 2. Y Init        xyz 0.05
            Y_pattern.put_point(param.ent*0.05, ((num_t_p > 1) ? -param.default_X : param.default_X), 0.0, 0.0, Linear); // 2. Y Init        xyz
            Z_pattern.put_point(param.ent*0.05, param.default_Z, 0.0, 0.0, Linear); // 2. Y Init        xyz
            //   3. empty+ax+az   xyz
            X_pattern.put_point(param.ent*0.10, X_tool[num_t_p] - 100, 0.0, 0.0, Linear); // 3. empty+ax+az   xyz
            Y_pattern.put_point(param.ent*0.10, Y_tool[num_t_p], 0.0, 0.0, Linear); // 3. empty+ax+az   xyz
            Z_pattern.put_point(param.ent*0.10, Z_tool[num_t_p] + 100, 0.0, 0.0, Linear); // 3. empty+ax+az   xyz
            //   4. empty+ax      z
            X_pattern.put_point(param.ent*0.15, X_tool[num_t_p] - 100, 0.0, 0.0, Linear); // 4. empty+ax      z
            Y_pattern.put_point(param.ent*0.15, Y_tool[num_t_p], 0.0, 0.0, Linear); // 4. empty+ax      z
            Z_pattern.put_point(param.ent*0.15, Z_tool[num_t_p]    , 0.0, 0.0, Linear); // 4. empty+ax      z
            //   5.
            X_pattern.put_point(param.ent*0.20, X_tool[num_t_p]     , 0.0, 0.0, Linear); // 5. empty         x
            Y_pattern.put_point(param.ent*0.20, Y_tool[num_t_p], 0.0, 0.0, Linear); // 5. empty         x
            Z_pattern.put_point(param.ent*0.20, Z_tool[num_t_p]     , 0.0, 0.0, Linear); // 5. empty         x
            //   6. Byee
            //   7. empty+az      z
            X_pattern.put_point(param.ent*0.35, X_tool[num_t_p]     , 0.0, 0.0, Linear); // 7. empty+az      z
            Y_pattern.put_point(param.ent*0.35, Y_tool[num_t_p], 0.0, 0.0, Linear); // 7. empty+az      z
            Z_pattern.put_point(param.ent*0.30, Z_tool[num_t_p]     , 0.0, 0.0, Linear); // 7. empty+az      z
            Z_pattern.put_point(param.ent*0.35, Z_tool[num_t_p] + 100, 0.0, 0.0, Linear); // 7. empty+az      z
            if(via_point)
            {
                X_pattern.put_point(param.ent*0.40, -X_tool[num_t_p], 0.5, 0.0, Fifth); // 7. empty+az      z
                X_pattern.put_point(param.ent*0.50, -X_tool[num_t_p] + 100, 0.5, 0.0, Fifth); // 7. empty+az      z
                X_pattern.put_point(param.ent*0.60, -X_tool[num_t_p], 0.0, 0.0, Fifth); // 7. empty+az      z

                Y_pattern.put_point(param.ent*0.40, (num_t_p > 1 ? -(-X_tool[num_t_p] + 50) : -X_tool[num_t_p] + 50), 0.5, 0.0, Fifth); // 7. empty+az      z
                Y_pattern.put_point(param.ent*0.50, 0.0, 0.5, 0.0, Fifth); // 7. empty+az      z
                Y_pattern.put_point(param.ent*0.60, (num_t_p > 1 ? (-X_tool[num_t_p] + 50) : -(-X_tool[num_t_p] + 50)), 0.5, 0.0, Fifth); // 7. empty+az      z

                Z_pattern.put_point(param.ent*0.40, param.default_Z, 0.0, 0.0, Fifth); // 7. empty+az      z
                Z_pattern.put_point(param.ent*0.50, param.default_Z, 0.0, 0.0, Fifth); // 7. empty+az      z
                Z_pattern.put_point(param.ent*0.60, param.default_Z, 0.0, 0.0, Fifth); // 7. empty+az      z
            }
            //   8. Tool + az  y
            X_pattern.put_point(param.ent*0.65, X_tool[num_t]     , 0.0, 0.0, Linear); // 8. Tool+az       y
            Y_pattern.put_point(param.ent*0.65, Y_tool[num_t], 0.0, 0.0, Linear); // 8. Tool+az       y
            Z_pattern.put_point(param.ent*0.65, Z_tool[num_t] + 100, 0.0, 0.0, Linear); // 8. Tool+az       y
            //   9. Tool  z
            X_pattern.put_point(param.ent*0.70, X_tool[num_t]     , 0.0, 0.0, Linear); // 9. Tool          z
            Y_pattern.put_point(param.ent*0.70, Y_tool[num_t], 0.0, 0.0, Linear); // 9. Tool          z
            Z_pattern.put_point(param.ent*0.70, Z_tool[num_t]     , 0.0, 0.0, Linear); // 9. Tool          z
            //   10. Hello
            //   11. Tool + az
            X_pattern.put_point(param.ent*0.80, X_tool[num_t]     , 0.0, 0.0, Linear); // 11. Tool+ax      x
            X_pattern.put_point(param.ent*0.85, X_tool[num_t] - 100, 0.0, 0.0, Linear); // 11. Tool+ax      x
            Y_pattern.put_point(param.ent*0.85, Y_tool[num_t], 0.0, 0.0, Linear); // 11. Tool+ax      x
            Z_pattern.put_point(param.ent*0.85, Z_tool[num_t]     , 0.0, 0.0, Linear); // 11. Tool+ax      x
            //   12. Tool + ax + az
            X_pattern.put_point(param.ent*0.90, X_tool[num_t] - 100, 0.0, 0.0, Linear); // 12. Tool+ax+az   z
            Y_pattern.put_point(param.ent*0.90, Y_tool[num_t], 0.0, 0.0, Linear); // 12. Tool+ax+az   z
            Z_pattern.put_point(param.ent*0.90, Z_tool[num_t] + 100, 0.0, 0.0, Linear); // 12. Tool+ax+az   z
            //   13. Y Init
            X_pattern.put_point(param.ent*0.95, 0.0        , 0.0, 0.0, Linear); // 13. Y Init       xyz
            Y_pattern.put_point(param.ent*0.95, ((num_t > 1) ? -param.default_X : param.default_X), 0.0, 0.0, Linear); // 13. Y Init       xyz
            Z_pattern.put_point(param.ent*0.95, param.default_Z, 0.0, 0.0, Linear); // 13. Y Init       xyz
            //   14.  to   xyz
            X_pattern.put_point(param.ent*1.00, pos_to.pX  , 0.0, 0.0, Linear); // 14. to           xyz
            Y_pattern.put_point(param.ent*1.00, pos_to.pY  , 0.0, 0.0, Linear); // 14. to           xyz
            Z_pattern.put_point(param.ent*1.00, pos_to.pZ  , 0.0, 0.0, Linear); // 14. to           xyz


            ang6_pattern.put_point(param.ent*0.00, 0.0, 0.0, 0.0, Fifth);
            ang6_pattern.put_point(param.ent*0.25, 0.0, 0.0, 0.0, Fifth);
            ang6_pattern.put_point(param.ent*0.30, OPEN, 0.0, 0.0, Fifth);
            ang6_pattern.put_point(param.ent*0.75, OPEN, 0.0, 0.0, Fifth);
            ang6_pattern.put_point(param.ent*0.80, CLOSE, 0.0, 0.0, Fifth);
            ang6_pattern.put_point(param.ent*1.00, CLOSE, 0.0, 0.0, Fifth);

            rX_pattern.put_point(0.0, rot_from.rX, 0.0, 0.0, Linear);
            rX_pattern.put_point(param.ent, rot_to.rX, 0.0, 0.0, Linear);

            rY_pattern.put_point(0.0, rot_from.rY, 0.0, 0.0, Linear);
            rY_pattern.put_point(param.ent, rot_to.rY, 0.0, 0.0, Linear);

            rZ_pattern.put_point(0.0, rot_from.rZ, 0.0, 0.0, Linear);
            rZ_pattern.put_point(param.ent, rot_to.rZ, 0.0, 0.0, Linear);

            break;}

        case TOOL:{
            break;}


        default:
            cout<<"what mode??"<<endl;

        }
    }

    pos_now.pX = X_pattern.result(t);
    pos_now.pY = Y_pattern.result(t);
    pos_now.pZ = Z_pattern.result(t);

    if(motion_type == TOOL_CHANGE)
        kinematics.ang6 = ang6_pattern.result(t);
    else
        kinematics.ang6 = CLOSE;

    rot_now.rX = rX_pattern.result(t);
    rot_now.rY = rY_pattern.result(t);
    rot_now.rZ = rZ_pattern.result(t);

    if(kinematics.ik_solve(pos_now.pX, pos_now.pY, pos_now.pZ, rot_now.rX, rot_now.rY, rot_now.rZ) == 1)
    {
        t = 0.0;
        param.onoff = false;
        moveinfo.end_flag = true;
        move2order_pub.publish(moveinfo);
    }
    else
    {
        kinematics.update_motorangle();
        kinematics.motor_packet(400);
    }

    move_ALEX::coortest coor;
    coor.x = pos_now.pX;
    coor.y = pos_now.pY;
    coor.z = pos_now.pZ;

    coor_pub.publish(coor);
}
