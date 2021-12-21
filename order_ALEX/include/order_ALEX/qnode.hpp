/**
 * @file /include/order_ALEX/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef order_ALEX_QNODE_HPP_
#define order_ALEX_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <iostream>

//..msg...........................
#include "order.h"
#include "move2order.h"

using namespace std;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace order_ALEX {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
        bool init();
	void run();

        void move2order_Callback(const move_ALEX::move2order::ConstPtr& msg);
        move_ALEX::move2order moveinfo;

Q_SIGNALS:
    void rosShutdown();
    void m2o_callback();

private:
	int init_argc;
        char** init_argv;
};

}  // namespace order_ALEX

#endif /* order_ALEX_QNODE_HPP_ */
