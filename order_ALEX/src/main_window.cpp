/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/order_ALEX/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace order_ALEX {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(m2o_callback()), this, SLOT(move2order_callback()));

    qnode.init();

    ordermsg.run_flag = false;

    ordermsg.ent = 100;
    ordermsg.pX = L3;
    ordermsg.pY = 0.0;
    ordermsg.pZ = L1+L2-L4;
    ordermsg.rX = 180;
    ordermsg.rY = 0.0;
    ordermsg.rZ = 180;

    ordermsg.correct_init_0 = 0;
    ordermsg.correct_init_1 = 0;
    ordermsg.correct_init_2 = 0;
    ordermsg.correct_init_3 = 0;
    ordermsg.correct_init_4 = 0;
    ordermsg.correct_init_5 = 0;

    order_pub.publish(ordermsg);

    ui.s_px->setValue(ordermsg.pX);
    ui.s_py->setValue(ordermsg.pY);
    ui.s_pz->setValue(ordermsg.pZ);
    ui.s_rx->setValue(ordermsg.rX);
    ui.s_ry->setValue(ordermsg.rY);
    ui.s_rz->setValue(ordermsg.rZ);

    ui.s_ent->setValue(ordermsg.ent);

    ui.tooltype_label->setVisible(false);
    ui.cb_tooltype->setVisible(false);
}

MainWindow::~MainWindow() {}

void MainWindow::move2order_callback()
{
    moveinfo.pX = qnode.moveinfo.pX;
    moveinfo.pY = qnode.moveinfo.pY;
    moveinfo.pZ = qnode.moveinfo.pZ;

    moveinfo.rX = qnode.moveinfo.rX;
    moveinfo.rY = qnode.moveinfo.rY;
    moveinfo.rZ = qnode.moveinfo.rZ;

    moveinfo.end_flag = qnode.moveinfo.end_flag;

    ui.e_frompx->setText(QString::number(roundf(moveinfo.pX)));
    ui.e_frompy->setText(QString::number(roundf(moveinfo.pY)));
    ui.e_frompz->setText(QString::number(roundf(moveinfo.pZ)));

    ui.e_fromrx->setText(QString::number(roundf(moveinfo.rX)));
    ui.e_fromry->setText(QString::number(roundf(moveinfo.rY)));
    ui.e_fromrz->setText(QString::number(roundf(moveinfo.rZ)));

    if(moveinfo.end_flag)
    {
        ui.flag->setText("move");
        ordermsg.run_flag = 0;
        button_flag = true;
    }
}
void MainWindow::on_init_clicked()
{
    ui.s_px->setValue(L3);
    ui.s_py->setValue(0.0);
    ui.s_pz->setValue(L1+L2-L4);
    ui.s_rx->setValue(180.0);
    ui.s_ry->setValue(0.0);
    ui.s_rz->setValue(180.0);
}

void MainWindow::on_b_save_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save file"), "/home/robit/catkin_ws/src/order_ALEX/save");

    if(fileName.isEmpty() == true)
        qDebug() << "Save Cancel";

    else
    {
        QFile *file = new QFile;
        file->setFileName(fileName);
        file->open(QIODevice::WriteOnly);
        QTextStream out(file);

        out << ordermsg.correct_init_0 << endl
            << ordermsg.correct_init_1 << endl
            << ordermsg.correct_init_2 << endl
            << ordermsg.correct_init_3 << endl
            << ordermsg.correct_init_4 << endl
            << ordermsg.correct_init_5 << endl
            << ordermsg.correct_init_6 <<endl

            << ordermsg.X_Tool_0 << endl
            << ordermsg.X_Tool_1 << endl
            << ordermsg.X_Tool_2 << endl
            << ordermsg.X_Tool_3 << endl

            << ordermsg.Y_Tool_0 << endl
            << ordermsg.Y_Tool_1 << endl
            << ordermsg.Y_Tool_2 << endl
            << ordermsg.Y_Tool_3 << endl

            << ordermsg.Z_Tool_0 << endl
            << ordermsg.Z_Tool_1 << endl
            << ordermsg.Z_Tool_2 << endl
            << ordermsg.Z_Tool_3 << endl

            << ",";

        file->close();

    }
}

void MainWindow::on_b_open_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open file"), "/home/robit/catkin_ws/src/order_ALEX/save/");

    if(fileName.isEmpty() == true)
    {
        qDebug() << "Load Cancel";
    }

    else
    {
        std::ifstream is;

        //is.open(fileName.toStdString());
        is.open(fileName.toStdString().c_str());

        is >> ordermsg.correct_init_0;
        is >> ordermsg.correct_init_1;
        is >> ordermsg.correct_init_2;
        is >> ordermsg.correct_init_3;
        is >> ordermsg.correct_init_4;
        is >> ordermsg.correct_init_5;
        is >> ordermsg.correct_init_6;

        is >> ordermsg.X_Tool_0;
        is >> ordermsg.X_Tool_1;
        is >> ordermsg.X_Tool_2;
        is >> ordermsg.X_Tool_3;

        is >> ordermsg.Y_Tool_0;
        is >> ordermsg.Y_Tool_1;
        is >> ordermsg.Y_Tool_2;
        is >> ordermsg.Y_Tool_3;

        is >> ordermsg.Z_Tool_0;
        is >> ordermsg.Z_Tool_1;
        is >> ordermsg.Z_Tool_2;
        is >> ordermsg.Z_Tool_3;

        order_pub.publish(ordermsg);


        is.close();

        ui.s_motor0->setValue(ordermsg.correct_init_0);
        ui.s_motor1->setValue(ordermsg.correct_init_1);
        ui.s_motor2->setValue(ordermsg.correct_init_2);
        ui.s_motor3->setValue(ordermsg.correct_init_3);
        ui.s_motor4->setValue(ordermsg.correct_init_4);
        ui.s_motor5->setValue(ordermsg.correct_init_5);

        ui.e_toolx_0->setText(QString::number(ordermsg.X_Tool_0));
        ui.e_toolx_1->setText(QString::number(ordermsg.X_Tool_1));
        ui.e_toolx_2->setText(QString::number(ordermsg.X_Tool_2));
        ui.e_toolx_3->setText(QString::number(ordermsg.X_Tool_3));

        ui.e_tooly_0->setText(QString::number(ordermsg.Y_Tool_0));
        ui.e_tooly_1->setText(QString::number(ordermsg.Y_Tool_1));
        ui.e_tooly_2->setText(QString::number(ordermsg.Y_Tool_2));
        ui.e_tooly_3->setText(QString::number(ordermsg.Y_Tool_3));

        ui.e_toolz_0->setText(QString::number(ordermsg.Z_Tool_0));
        ui.e_toolz_1->setText(QString::number(ordermsg.Z_Tool_1));
        ui.e_toolz_2->setText(QString::number(ordermsg.Z_Tool_2));
        ui.e_toolz_3->setText(QString::number(ordermsg.Z_Tool_3));
    }
}

void MainWindow::on_flag_clicked()
{
    if(button_flag)
    {
        ui.e_topx->setText(QString::number(ordermsg.pX));
        ui.e_topy->setText(QString::number(ordermsg.pY));
        ui.e_topz->setText(QString::number(ordermsg.pZ));
        ui.e_torx->setText(QString::number(ordermsg.rX));
        ui.e_tory->setText(QString::number(ordermsg.rY));
        ui.e_torz->setText(QString::number(ordermsg.rZ));

        ui.flag->setText("stop");
        ordermsg.run_flag = 1;
        button_flag = false;
    }
    else if(!button_flag)
    {
        ui.flag->setText("move");
        ordermsg.run_flag = 0;
        button_flag = true;
    }

    ordermsg.type = motion_type;
    ordermsg.toolnumber = tool_number;
    order_pub.publish(ordermsg);
}

void MainWindow::on_b_pick_clicked()
{
    motion_type = PICK_PLACE;
    ui.b_pick->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:white;"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );
    ui.b_move->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );
    ui.b_tool_change->setStyleSheet(    "border: 0px solid gray;"
                                        "border-radius: 10px;"
                                        "padding: 0 8px;"
                                        "background:rgb(221, 221, 240);"
                                        "selection-background-color: rgb(216, 216, 216);"
                                        );
    ui.b_tool->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );

    ui.tooltype_label->setVisible(false);
    ui.cb_tooltype->setVisible(false);
}
void MainWindow::on_b_move_clicked()
{
    motion_type = MOVE;
    ui.b_pick->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );
    ui.b_move->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:white;"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );
    ui.b_tool_change->setStyleSheet(    "border: 0px solid gray;"
                                        "border-radius: 10px;"
                                        "padding: 0 8px;"
                                        "background:rgb(221, 221, 240);"
                                        "selection-background-color: rgb(216, 216, 216);"
                                        );
    ui.b_tool->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );

    ui.tooltype_label->setVisible(false);
    ui.cb_tooltype->setVisible(false);
}
void MainWindow::on_b_tool_change_clicked()
{
    motion_type = TOOL_CHANGE;
    ui.b_pick->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );
    ui.b_move->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );
    ui.b_tool_change->setStyleSheet(    "border: 0px solid gray;"
                                        "border-radius: 10px;"
                                        "padding: 0 8px;"
                                        "background:white;"
                                        "selection-background-color: rgb(216, 216, 216);"
                                        );
    ui.b_tool->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );

    ui.tooltype_label->setVisible(true);
    ui.cb_tooltype->setVisible(true);
}
void MainWindow::on_b_tool_clicked()
{
    motion_type = TOOL;
    ui.b_pick->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );
    ui.b_move->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:rgb(221, 221, 240);"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );
    ui.b_tool_change->setStyleSheet(    "border: 0px solid gray;"
                                        "border-radius: 10px;"
                                        "padding: 0 8px;"
                                        "background:rgb(221, 221, 240);"
                                        "selection-background-color: rgb(216, 216, 216);"
                                        );
    ui.b_tool->setStyleSheet(    "border: 0px solid gray;"
                                 "border-radius: 10px;"
                                 "padding: 0 8px;"
                                 "background:white;"
                                 "selection-background-color: rgb(216, 216, 216);"
                                 );

    ui.tooltype_label->setVisible(true);
    ui.cb_tooltype->setVisible(true);
}

void MainWindow::on_cb_tooltype_currentIndexChanged(int index)
{
    tool_number = index;
}
//target====================================
//..pos............................
void MainWindow::on_s_px_valueChanged(int value)
{
    ordermsg.pX = value;

    QString position = QString::number(value);
    ui.e_px->setText(position);
}

void MainWindow::on_s_py_valueChanged(int value)
{
    ordermsg.pY = value;

    QString position = QString::number(value);
    ui.e_py->setText(position);
}

void MainWindow::on_s_pz_valueChanged(int value)
{
    ordermsg.pZ = value;

    QString position = QString::number(value);
    ui.e_pz->setText(position);
}

void MainWindow::on_e_px_textChanged(const QString &arg1)
{
    ui.s_px->setValue(arg1.toInt());
}

void MainWindow::on_e_py_textChanged(const QString &arg1)
{
    ui.s_py->setValue(arg1.toInt());
}

void MainWindow::on_e_pz_textChanged(const QString &arg1)
{
    ui.s_pz->setValue(arg1.toInt());
}

//..rot.............................
void MainWindow::on_s_rx_valueChanged(int value)
{
    ordermsg.rX = value;

    QString position = QString::number(value);
    ui.e_rx->setText(position);
}

void MainWindow::on_s_ry_valueChanged(int value)
{
    ordermsg.rY = value;

    QString position = QString::number(value);
    ui.e_ry->setText(position);
}

void MainWindow::on_s_rz_valueChanged(int value)
{
    ordermsg.rZ = value;

    QString position = QString::number(value);
    ui.e_rz->setText(position);
}

void MainWindow::on_e_rx_textChanged(const QString &arg1)
{
    ui.s_rx->setValue(arg1.toInt());
}
void MainWindow::on_e_ry_textChanged(const QString &arg1)
{
    ui.s_ry->setValue(arg1.toInt());
}
void MainWindow::on_e_rz_textChanged(const QString &arg1)
{
    ui.s_rz->setValue(arg1.toInt());
}

//ent...............................
void MainWindow::on_s_ent_valueChanged(int value)
{
    ordermsg.ent = value;

    QString position = QString::number(value);
    ui.e_ent->setText(position);
}

void MainWindow::on_e_ent_textChanged(const QString &arg1)
{
    ui.s_ent->setValue(arg1.toInt());
}


//correct.........................................................
void MainWindow::on_s_motor0_valueChanged(int value)
{
    ordermsg.correct_init_0 = value;

    QString position = QString::number(value);
    ui.e_motor0->setText(position);

    order_pub.publish(ordermsg);
}
void MainWindow::on_s_motor1_valueChanged(int value)
{
    ordermsg.correct_init_1 = value;

    QString position = QString::number(value);
    ui.e_motor1->setText(position);

    order_pub.publish(ordermsg);
}
void MainWindow::on_s_motor2_valueChanged(int value)
{
    ordermsg.correct_init_2 = value;

    QString position = QString::number(value);
    ui.e_motor2->setText(position);

    order_pub.publish(ordermsg);
}
void MainWindow::on_s_motor3_valueChanged(int value)
{
    ordermsg.correct_init_3 = value;

    QString position = QString::number(value);
    ui.e_motor3->setText(position);

    order_pub.publish(ordermsg);
}
void MainWindow::on_s_motor4_valueChanged(int value)
{
    ordermsg.correct_init_4 = value;

    QString position = QString::number(value);
    ui.e_motor4->setText(position);

    order_pub.publish(ordermsg);
}
void MainWindow::on_s_motor5_valueChanged(int value)
{
    ordermsg.correct_init_5 = value;

    QString position = QString::number(value);
    ui.e_motor5->setText(position);

    order_pub.publish(ordermsg);
}
void MainWindow::on_s_motor6_valueChanged(int value)
{
    ordermsg.correct_init_6 = value;

    QString position = QString::number(value);
    ui.e_motor6->setText(position);

    order_pub.publish(ordermsg);
}

void MainWindow::on_e_motor0_textChanged(const QString &arg1)
{
    ui.s_motor0->setValue(arg1.toInt());
}
void MainWindow::on_e_motor1_textChanged(const QString &arg1)
{
    ui.s_motor1->setValue(arg1.toInt());
}
void MainWindow::on_e_motor2_textChanged(const QString &arg1)
{
    ui.s_motor2->setValue(arg1.toInt());
}
void MainWindow::on_e_motor3_textChanged(const QString &arg1)
{
    ui.s_motor3->setValue(arg1.toInt());
}
void MainWindow::on_e_motor4_textChanged(const QString &arg1)
{
    ui.s_motor4->setValue(arg1.toInt());
}
void MainWindow::on_e_motor5_textChanged(const QString &arg1)
{
    ui.s_motor5->setValue(arg1.toInt());
}
void MainWindow::on_e_motor6_textChanged(const QString &arg1)
{
    ui.s_motor6->setValue(arg1.toInt());
}



//..save tool coor.............................................
void MainWindow::on_b_save_0_clicked()
{
    ui.e_toolx_0->setText(QString::number(ordermsg.pX));
    ui.e_tooly_0->setText(QString::number(ordermsg.pY));
    ui.e_toolz_0->setText(QString::number(ordermsg.pZ));

    ordermsg.X_Tool_0 = ordermsg.pX;
    ordermsg.Y_Tool_0 = ordermsg.pY;
    ordermsg.Z_Tool_0 = ordermsg.pZ;
    ordermsg.run_flag = false;
    order_pub.publish(ordermsg);
}
void MainWindow::on_b_save_1_clicked()
{
    ui.e_toolx_1->setText(QString::number(ordermsg.pX));
    ui.e_tooly_1->setText(QString::number(ordermsg.pY));
    ui.e_toolz_1->setText(QString::number(ordermsg.pZ));

    ordermsg.X_Tool_1 = ordermsg.pX;
    ordermsg.Y_Tool_1 = ordermsg.pY;
    ordermsg.Z_Tool_1 = ordermsg.pZ;
    ordermsg.run_flag = false;
    order_pub.publish(ordermsg);
}
void MainWindow::on_b_save_2_clicked()
{
    ui.e_toolx_2->setText(QString::number(ordermsg.pX));
    ui.e_tooly_2->setText(QString::number(ordermsg.pY));
    ui.e_toolz_2->setText(QString::number(ordermsg.pZ));

    ordermsg.X_Tool_2 = ordermsg.pX;
    ordermsg.Y_Tool_2 = ordermsg.pY;
    ordermsg.Z_Tool_2 = ordermsg.pZ;
    ordermsg.run_flag = false;
    order_pub.publish(ordermsg);
}
void MainWindow::on_b_save_3_clicked()
{
    ui.e_toolx_3->setText(QString::number(ordermsg.pX));
    ui.e_tooly_3->setText(QString::number(ordermsg.pY));
    ui.e_toolz_3->setText(QString::number(ordermsg.pZ));

    ordermsg.X_Tool_3 = ordermsg.pX;
    ordermsg.Y_Tool_3 = ordermsg.pY;
    ordermsg.Z_Tool_3 = ordermsg.pZ;
    ordermsg.run_flag = false;
    order_pub.publish(ordermsg);
}

void MainWindow::on_b_load_0_clicked()
{
    ui.s_px->setValue(ordermsg.X_Tool_0);
    ui.s_py->setValue(ordermsg.Y_Tool_0);
    ui.s_pz->setValue(ordermsg.Z_Tool_0);
}
void MainWindow::on_b_load_1_clicked()
{
    ui.s_px->setValue(ordermsg.X_Tool_1);
    ui.s_py->setValue(ordermsg.Y_Tool_1);
    ui.s_pz->setValue(ordermsg.Z_Tool_1);
}
void MainWindow::on_b_load_2_clicked()
{
    ui.s_px->setValue(ordermsg.X_Tool_2);
    ui.s_py->setValue(ordermsg.Y_Tool_2);
    ui.s_pz->setValue(ordermsg.Z_Tool_2);
}
void MainWindow::on_b_load_3_clicked()
{
    ui.s_px->setValue(ordermsg.X_Tool_3);
    ui.s_py->setValue(ordermsg.Y_Tool_3);
    ui.s_pz->setValue(ordermsg.Z_Tool_3);
}
}  // namespace order_ALEX


