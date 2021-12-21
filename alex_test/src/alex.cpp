#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "../include/alex_test/angle.h"

using namespace std;

namespace gazebo
{
 class Alex : public ModelPlugin
 {
 private:

     physics::JointController * JC;
     physics::ModelPtr model;

     physics::JointPtr shoulder_pan;
     physics::JointPtr shoulder_lift;
     physics::JointPtr elbow;
     physics::JointPtr wrist_yaw;
     physics::JointPtr wrist_pitch;
     physics::JointPtr wrist_yaw2;

     event::ConnectionPtr updateConnection;

     ros::NodeHandle n;
     ros::Subscriber sub;

     double ang[6] = {0.0, };
     int time;

 public:

     Alex(){}
     ~Alex(){this->n.shutdown();}

     //running once with roslaunch
     void Load(physics::ModelPtr _parent, sdf::ElementPtr)
     {
         int argc = 0;
         char** argv = NULL;

         ros::init(argc, argv, "alex");
         sub = n.subscribe("angle", 100, &Alex::msgcallback, this);

         model = _parent;

         JC = new physics::JointController(model);

         shoulder_pan = model->GetJoint("shoulder_pan");
         shoulder_lift = model->GetJoint("shoulder_lift");
         elbow = model->GetJoint("elbow");
         wrist_yaw = model->GetJoint("wrist_yaw");
         wrist_pitch = model->GetJoint("wrist_pitch");
         wrist_yaw2 = model->GetJoint("wrist_yaw2");

         updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Alex::OnUpdate, this, _1));

         time = 0;
     }

     void msgcallback(const alex_test::angle::ConstPtr &msg)
     {
         ang[0] = ((msg->ang0)*3.141592)/180.0;
         ang[1] = ((msg->ang1)*3.141592)/180.0;
         ang[2] = ((msg->ang2)*3.141592)/180.0;
         ang[3] = ((msg->ang3)*3.141592)/180.0;
         ang[4] = ((msg->ang4)*3.141592)/180.0;
         ang[5] = ((msg->ang5)*3.141592)/180.0;
     }

     void OnUpdate(const common::UpdateInfo &)
     {
         time += 1;
         if(time >= 100){
          time = 0;

//          for(int i = 0; i < 5; i++)
//            cout<<"ang["<<i<<"]   ===   "<<ang[i]<<endl;
         }

         JC->SetJointPosition(shoulder_pan, ang[0]);
         JC->SetJointPosition(shoulder_lift, ang[1]);
         JC->SetJointPosition(elbow, ang[2]);
         JC->SetJointPosition(wrist_yaw, ang[3]);
         JC->SetJointPosition(wrist_pitch, ang[4]);
         JC->SetJointPosition(wrist_yaw2, ang[5]);
     }
 };

  GZ_REGISTER_MODEL_PLUGIN(Alex); //set plugin name
 }
