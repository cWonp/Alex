#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/order_ALEX/qnode.hpp"

namespace order_ALEX {

ros::Publisher order_pub;
ros::Subscriber move2order_sub;

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::move2order_Callback(const move_ALEX::move2order_<std::allocator<void> >::ConstPtr &msg)
{
    moveinfo.pX = msg->pX;
    moveinfo.pY = msg->pY;
    moveinfo.pZ = msg->pZ;
    moveinfo.rX = msg->rX;
    moveinfo.rY = msg->rY;
    moveinfo.rZ = msg->rZ;

    moveinfo.end_flag = msg->end_flag;

    Q_EMIT m2o_callback();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"order_ALEX");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

        //..msg..................................
        order_pub = n.advertise<order_ALEX::order>("order", 100);
        move2order_sub = n.subscribe("moveinfo", 100, &QNode::move2order_Callback, this);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);

        while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

}  // namespace order_ALEX
