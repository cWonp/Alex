/**
 * @file /include/order_ALEX/main_window.hpp
 *
 * @brief Qt based gui for order_ALEX.
 *
 * @date November 2010
 **/
#ifndef order_ALEX_MAIN_WINDOW_H
#define order_ALEX_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <iostream>
#include <QString>
#include <fstream>

//..msg.............................
#include "order.h"
#include "move2order.h"

using namespace std;

//..motion type.....................
#define PICK_PLACE  0
#define MOVE        1
#define TOOL_CHANGE 2
#define TOOL        3


namespace order_ALEX {

extern ros::Publisher order_pub;

class MainWindow : public QMainWindow {
Q_OBJECT
public:

	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

        order_ALEX::order ordermsg;
        move_ALEX::move2order moveinfo;

private Q_SLOTS:

        void move2order_callback();

        void on_flag_clicked();
        void on_b_save_clicked();
        void on_b_open_clicked();

        void on_init_clicked();

        void on_b_pick_clicked();
        void on_b_move_clicked();
        void on_b_tool_change_clicked();
        void on_b_tool_clicked();

        void on_cb_tooltype_currentIndexChanged(int index);

        //..pos............................
        void on_s_px_valueChanged(int value);
        void on_s_py_valueChanged(int value);
        void on_s_pz_valueChanged(int value);

        void on_e_px_textChanged(const QString &arg1);
        void on_e_py_textChanged(const QString &arg1);
        void on_e_pz_textChanged(const QString &arg1);

        //..rot.............................
        void on_s_rx_valueChanged(int value);
        void on_s_ry_valueChanged(int value);
        void on_s_rz_valueChanged(int value);

        void on_e_rx_textChanged(const QString &arg1);
        void on_e_ry_textChanged(const QString &arg1);
        void on_e_rz_textChanged(const QString &arg1);

        //ent...............................
        void on_s_ent_valueChanged(int value);
        void on_e_ent_textChanged(const QString &arg1);

        //correct.....................................
        void on_s_motor0_valueChanged(int value);
        void on_s_motor1_valueChanged(int value);
        void on_s_motor2_valueChanged(int value);
        void on_s_motor3_valueChanged(int value);
        void on_s_motor4_valueChanged(int value);
        void on_s_motor5_valueChanged(int value);
        void on_s_motor6_valueChanged(int value);

        void on_e_motor0_textChanged(const QString &arg1);
        void on_e_motor1_textChanged(const QString &arg1);
        void on_e_motor2_textChanged(const QString &arg1);
        void on_e_motor3_textChanged(const QString &arg1);
        void on_e_motor4_textChanged(const QString &arg1);
        void on_e_motor5_textChanged(const QString &arg1);
        void on_e_motor6_textChanged(const QString &arg1);

        void on_b_save_0_clicked();
        void on_b_save_1_clicked();
        void on_b_save_2_clicked();
        void on_b_save_3_clicked();

        void on_b_load_0_clicked();
        void on_b_load_1_clicked();
        void on_b_load_2_clicked();
        void on_b_load_3_clicked();




private:
	Ui::MainWindowDesign ui;
	QNode qnode;

        double L1 = 77.595;
        double L2 = 300.0;
        double L3 = 220.0;
        double L4 = 126.0;

        int x_tool = -154;
        int y_tool0 = 123;
        int z_tool =  165 - 98;

        bool button_flag = true;
        int motion_type = 0;
        int tool_number = 0;
};

}  // namespace order_ALEX

#endif // order_ALEX_MAIN_WINDOW_H
