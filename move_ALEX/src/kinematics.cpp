#include "../include/move_ALEX/kinematics.hpp"

double solve::ang2pos(double angle)
{
    return (double)((angle * 4096.0) / 360.0);
}
double solve::pos2ang(double pos)
{
    return (double)((pos * 360.0) / 4096.0);
}
double solve::to360(double deg)
{
    if(deg < 0) deg += 360;
    else if(deg > 360) deg -= 360;
}

void solve::init_save()
{
    ifstream is;

    is.open("/home/robit/catkin_ws/src/move_ALEX/init/init");

    for(int i=0; i<23; i++)
        is >> position[i];

    is.close();

    for(int DXL_ID = 0; DXL_ID < 7; DXL_ID++)
    {

        g_DXL_ID_position[DXL_ID] = position[DXL_ID]*4;
        g_DXL_ID_Save_position[DXL_ID] = position[DXL_ID]*4;

        cout<<"g_DXL_ID_Save_position["<<DXL_ID<<"] = "<<g_DXL_ID_Save_position[DXL_ID]<<endl;
    }
}


void solve::fk_solve(double *x, double *y, double *z, double *rx, double *ry, double *rz)
{
    double th1 = pos2ang(g_DXL_ID_position[0]-(g_DXL_ID_Save_position[0]+correct_init[0]))*deg2rad;
    double th2 = pos2ang(-g_DXL_ID_position[1]+(g_DXL_ID_Save_position[1]+correct_init[1]))*deg2rad;
    double th3 = (pos2ang(g_DXL_ID_position[2]-(g_DXL_ID_Save_position[2]+correct_init[2])) + 90);
    if(th3 >= 180) th3 -= 360;
    th3 *= deg2rad;
    double th4 = pos2ang(g_DXL_ID_position[3]-(g_DXL_ID_Save_position[3]+correct_init[3]))*deg2rad;
    double th5 = pos2ang(-g_DXL_ID_position[4]+(g_DXL_ID_Save_position[4]+correct_init[4]))*deg2rad;
    double th6 = pos2ang(g_DXL_ID_position[5]-(g_DXL_ID_Save_position[5]+correct_init[5]))*deg2rad;

//    cout<<endl<<"th1   ====   "<<th1*rad2deg<<endl;
//    cout<<"th2   ====   "<<th2*rad2deg<<endl;
//    cout<<"th3   ====   "<<th3*rad2deg<<endl;
//    cout<<"th4   ====   "<<th4*rad2deg<<endl;
//    cout<<"th5   ====   "<<th5*rad2deg<<endl;
//    cout<<"th6   ====   "<<th6*rad2deg<<endl<<endl;


    *x = L4*(sin(th5)*(sin(th1)*sin(th4)+cos(th4)*cos(th1)*cos(th2+th3))+cos(th5)*(cos(th1)*sin(th2+th3)))+L3*cos(th1)*sin(th2+th3)+L2*cos(th1)*cos(th2);
    *y = L3*(sin(th2+th3)*sin(th1))-L4*(sin(th5)*(sin(th4)*cos(th1)-cos(th4)*sin(th1)*cos(th2+th3))-cos(th5)*sin(th1)*sin(th2+th3))+L2*sin(th1)*cos(th2);
    *z = L1-L3*cos(th2+th3)+L2*sin(th2)-L4*(cos(th5)*cos(th2+th3)-cos(th4)*sin(th5)*sin(th2+th3));

    double R11 = sin(th6)*(cos(th4)*sin(th1)-cos(th1)*cos(th2+th3)*sin(th4))+cos(th6)*(cos(th5)*(sin(th1)*sin(th4)+cos(th1)*cos(th2+th3)*cos(th4))-sin(th5)*cos(th1)*sin(th2+th3));
    double R12 = cos(th6)*(cos(th4)*sin(th1)-cos(th1)*cos(th2+th3)*sin(th4))-sin(th6)*(cos(th5)*(sin(th1)*sin(th4)+cos(th1)*cos(th2+th3)*cos(th4))-cos(th1)*sin(th2+th3)*sin(th5));
    double R13 = sin(th5)*(sin(th1)*sin(th4)+cos(th1)*cos(th2+th3)*cos(th4))+cos(th1)*sin(th2+th3)*cos(th5);
    double R23 = sin(th1)*sin(th2+th3)*cos(th5)-sin(th5)*(cos(th1)*sin(th4)-sin(th1)*cos(th2+th3)*cos(th4));
    double R31 = cos(th6)*(sin(th5)*cos(th2+th3)+cos(th4)*cos(th5)*sin(th2+th3))-sin(th4)*sin(th6)*sin(th2+th3);
    double R32 = -sin(th6)*(sin(th5)*sin(th2+th3)+cos(th4)*cos(th5)*sin(th2+th3))-cos(th6)*sin(th4)*sin(th2+th3);
    double R33 = sin(th2+th3)*cos(th4)*sin(th5)-cos(th2+th3)*cos(th5);


    double beta = atan2(R13, sqrt(pow(R11,2)+pow(R12,2)))*rad2deg;
    double alpha = atan2(-R23/cos(beta), R33/cos(beta))*rad2deg;
    double gamma = atan2(-R12/cos(beta), R11/cos(beta))*rad2deg;

    if((int)beta == -90)
    {
        alpha = 0.0;
        gamma = atan2(-R32, R31)*rad2deg;
    }
    else if((int)beta == 90)
    {
        alpha = 0.0;
        gamma = atan2(R32, -R31)*rad2deg;
    }

    if(roundf(alpha) <= -180) alpha = 180.0;
    if(roundf(beta) <= -180) beta = 180.0;
    if(roundf(gamma) <= -180) gamma = 180.0;

    *rx = alpha;
    *ry = beta;
    *rz = gamma;

}

// R : Euler angle x, y, z
int solve::ik_solve(double pX, double pY, double pZ, double rX, double rY, double rZ)
{
    Matrix3d Rx, Ry, Rz;
    Matrix3d R0_1, R1_2, R2_3;
    Matrix3d R0_6, R0_3, R3_6;

    double pX_4, pY_4, pZ_4;
    double D;

    int RETURN = 0;

    rX *= deg2rad;
    rY *= deg2rad;
    rZ *= deg2rad;
    
    Rx << 1, 0, 0,
            0, cos(rX), -sin(rX),
            0, sin(rX), cos(rX);

    Ry << cos(rY), 0, sin(rY),
            0, 1, 0,
            -sin(rY), 0, cos(rY);

    Rz << cos(rZ), -sin(rZ), 0,
            sin(rZ), cos(rZ), 0,
            0, 0, 1;

    R0_6 = (Rx*Ry)*Rz;

    pX_4 = pX - L4*R0_6(0,2);
    pY_4 = pY - L4*R0_6(1,2);
    pZ_4 = pZ - L4*R0_6(2,2);

    th[0] = atan2(pY_4, pX_4);

    D = (pow(pX_4, 2)+pow(pY_4,2)+pow((pZ_4-L1),2)-pow(L2,2)-pow(L3,2))/(2*L2*L3);
    th[2] = atan2(D, sqrt(1-pow(D,2)));
    th[1] = atan2(pZ_4 - L1, sqrt(pow(pX_4,2) + pow(pY_4,2))) - atan2(-L3*cos(th[2]), L2 + L3*sin(th[2]));

    R0_1 << cos(th[0]), 0, sin(th[0]),
            sin(th[0]), 0, -cos(th[0]),
            0, 1, 0;

    R1_2 << cos(th[1]), -sin(th[1]), 0,
            sin(th[1]), cos(th[1]), 0,
            0, 0, 1;

    R2_3 << cos(th[2]), 0, sin(th[2]),
            sin(th[2]), 0, -cos(th[2]),
            0, 1, 0;

    R0_3 = (R0_1*R1_2)*R2_3;

    R3_6 = R0_3.inverse() * R0_6;

    th[3] = atan2(-R3_6(1,2),-R3_6(0,2));
    th[4] = atan2(-sqrt(1-pow(R3_6(2,2),2)),R3_6(2,2));
    th[5] = atan2(-R3_6(2,1), (R3_6(2,0)));


    //..radian to degree............................
    for(int i = 0; i < 6; i++)
    {
        th[i] = th[i]*rad2deg;
    }

    //..Limit........................................................................................
    if(isnan(th[0]) || isnan(th[1]) || isnan(th[2]) || isnan(th[3]) || isnan(th[4]) || isnan(th[5]))
    {
        RETURN = 1;
    }
    else if((th[1] > Ang_Lm_max[1] && th[1] < Ang_Lm_min[1]) ||
            (th[2] > Ang_Lm_max[2] && th[2] < Ang_Lm_min[2]) ||
            !(th[4] > Ang_Lm_min[4] && th[4] < Ang_Lm_max[4]))
    {
        RETURN = 2;

        for(int i = 0; i < 6; i++)
        {
            if(th[i] > Ang_Lm_max[i])
                th[i] == Ang_Lm_max[i];
            if(th[i] < Ang_Lm_min[i])
                th[i] == Ang_Lm_min[i];
        }
    }
    else
    {
        RETURN = 0;
    }

    return RETURN;
}

void solve::update_motorangle()
{
    for(int i = 0; i < 6; i++)
    {
        _th[i] = th[i];
    }
}

void solve::motor_packet(int speed)
{
    for(int i = 0; i < 7; i++)
    {
        g_DXL_ID_position[i] = g_DXL_ID_Save_position[i] + correct_init[i];
    }

    g_DXL_ID_position[0] += ang2pos(_th[0]);
    g_DXL_ID_position[1] -= ang2pos(_th[1]);
    g_DXL_ID_position[2] += ang2pos(_th[2] - 90);
    g_DXL_ID_position[3] += ang2pos(_th[3]);
    g_DXL_ID_position[4] -= ang2pos(_th[4]);
    g_DXL_ID_position[5] += ang2pos(_th[5]);
    g_DXL_ID_position[6] += ang6;

    for(int i = 0; i < 7; i++)
    {
        if(g_DXL_ID_position[i] >= 4096.0) g_DXL_ID_position[i] -= 4096.0;
        else if(g_DXL_ID_position[i] < 0.0) g_DXL_ID_position[i] += 4096.0;
    }

    for(int i = 0; i < 7; i++)
    {
        static int a = 0;
        if(g_DXL_ID_position[i] > 4096.0 || g_DXL_ID_position[i] < 0.0)
        {
            cout<<"WATCH OUT!!!!!!!!!!!!!!!!!!!!!!"<<endl<<endl;
            a++;
            cout<<"a == "<<a<<endl;
        }
        //        cout<<"a == "<<a<<endl;
    }

    //    for(int i = 0; i < 6; i++)
    //    {
    //        cout<<"g_DXL_ID_position["<<i<<"]  ===  "<<g_DXL_ID_position[i]<<endl;
    //    }


    alex_test::angle simulation;
    simulation.ang0 = _th[0];
    simulation.ang1 = _th[1];
    simulation.ang2 = _th[2]-90;
    if(simulation.ang2 < -180.0) simulation.ang2 += 360;
    simulation.ang3 = _th[3];
    simulation.ang4 = _th[4];
    simulation.ang5 = _th[5];

    angle_pub.publish(simulation);

    //    cout<<"....................."<<endl;
    //    cout<<"0 == "<<simulation.ang0<<endl;
    //    cout<<"1 == "<<simulation.ang1<<endl;
    //    cout<<"2 == "<<simulation.ang2<<endl;
    //    cout<<"3 == "<<simulation.ang3<<endl;
    //    cout<<"4 == "<<simulation.ang4<<endl;
    //    cout<<"5 == "<<simulation.ang5<<endl;

    move_ALEX::dxltest dxltestmsg;
    dxltestmsg.dxl_0 = g_DXL_ID_position[0];
    dxltestmsg.dxl_1 = g_DXL_ID_position[1];
    dxltestmsg.dxl_2 = g_DXL_ID_position[2];
    dxltestmsg.dxl_3 = g_DXL_ID_position[3];
    dxltestmsg.dxl_4 = g_DXL_ID_position[4];
    dxltestmsg.dxl_5 = g_DXL_ID_position[5];

    dxl_pub.publish(dxltestmsg);



    msg_generate::motor_msg dxMsg;

    for(int i = 0; i < 7; i++)
    {
        dxMsg.id.push_back(i);
        int pos = g_DXL_ID_position[i];
        dxMsg.position.push_back(pos);

        dxMsg.speed.push_back(speed);
    }
    dxMsg.length = dxMsg.id.size();
    dxMsg.mode = 3;

    motor_pub.publish(dxMsg);
}

