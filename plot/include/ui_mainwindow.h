/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QCustomPlot *customPlot;
    QPushButton *btn_exit;
    QPushButton *btn_freeze;
    QPushButton *btn_clear;
    QLabel *x_label;
    QLabel *y_label;
    QLabel *z_label;
    QLabel *yaw_label;
    QLabel *pitch_label;
    QLabel *max_lable;
    QLabel *min_lable;
    QLabel *mean_lable;
    QLabel *var_lable;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1302, 718);
        QFont font;
        font.setPointSize(16);
        MainWindow->setFont(font);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        customPlot = new QCustomPlot(centralWidget);
        customPlot->setObjectName(QStringLiteral("customPlot"));
        customPlot->setGeometry(QRect(80, 60, 911, 451));
        btn_exit = new QPushButton(centralWidget);
        btn_exit->setObjectName(QStringLiteral("btn_exit"));
        btn_exit->setGeometry(QRect(1040, 460, 101, 41));
        QFont font1;
        font1.setPointSize(15);
        btn_exit->setFont(font1);
        btn_freeze = new QPushButton(centralWidget);
        btn_freeze->setObjectName(QStringLiteral("btn_freeze"));
        btn_freeze->setGeometry(QRect(1040, 340, 101, 41));
        btn_freeze->setFont(font1);
        btn_clear = new QPushButton(centralWidget);
        btn_clear->setObjectName(QStringLiteral("btn_clear"));
        btn_clear->setGeometry(QRect(1040, 400, 101, 41));
        btn_clear->setFont(font1);
        x_label = new QLabel(centralWidget);
        x_label->setObjectName(QStringLiteral("x_label"));
        x_label->setGeometry(QRect(80, 560, 80, 41));
        x_label->setFont(font);
        y_label = new QLabel(centralWidget);
        y_label->setObjectName(QStringLiteral("y_label"));
        y_label->setGeometry(QRect(270, 560, 80, 41));
        y_label->setFont(font);
        z_label = new QLabel(centralWidget);
        z_label->setObjectName(QStringLiteral("z_label"));
        z_label->setGeometry(QRect(470, 560, 80, 41));
        z_label->setFont(font);
        yaw_label = new QLabel(centralWidget);
        yaw_label->setObjectName(QStringLiteral("yaw_label"));
        yaw_label->setGeometry(QRect(655, 560, 100, 41));
        yaw_label->setFont(font);
        pitch_label = new QLabel(centralWidget);
        pitch_label->setObjectName(QStringLiteral("pitch_label"));
        pitch_label->setGeometry(QRect(865, 560, 120, 41));
        pitch_label->setFont(font);

        max_lable = new QLabel(centralWidget);
        max_lable->setObjectName(QStringLiteral("max_label"));
        max_lable->setGeometry(QRect(1060, 60, 161, 41));
        max_lable->setFont(font);
        min_lable = new QLabel(centralWidget);
        min_lable->setObjectName(QStringLiteral("min_label"));
        min_lable->setGeometry(QRect(1060, 130, 161, 41));
        min_lable->setFont(font);
        mean_lable = new QLabel(centralWidget);
        mean_lable->setObjectName(QStringLiteral("mean_label"));
        mean_lable->setGeometry(QRect(1060, 200, 161, 41));
        mean_lable->setFont(font);
        var_lable = new QLabel(centralWidget);
        var_lable->setObjectName(QStringLiteral("var_label"));
        var_lable->setGeometry(QRect(1060, 270, 161, 41));
        var_lable->setFont(font);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1302, 37));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);
        QObject::connect(btn_exit, SIGNAL(clicked()), MainWindow, SLOT(close()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        btn_exit->setText(QApplication::translate("MainWindow", "Exit", Q_NULLPTR));
        btn_freeze->setText(QApplication::translate("MainWindow", "Freeze", Q_NULLPTR));
        btn_clear->setText(QApplication::translate("MainWindow", "Clear", Q_NULLPTR));
        x_label->setText(QApplication::translate("MainWindow", "x", Q_NULLPTR));
        y_label->setText(QApplication::translate("MainWindow", "y", Q_NULLPTR));
        z_label->setText(QApplication::translate("MainWindow", "z", Q_NULLPTR));
        yaw_label->setText(QApplication::translate("MainWindow", "yaw", Q_NULLPTR));
        pitch_label->setText(QApplication::translate("MainWindow", "pitch", Q_NULLPTR));
        max_lable->setText(QApplication::translate("MainWindow", "max:", Q_NULLPTR));
        min_lable->setText(QApplication::translate("MainWindow", "min:", Q_NULLPTR));
        mean_lable->setText(QApplication::translate("MainWindow", "mean:", Q_NULLPTR));
        var_lable->setText(QApplication::translate("MainWindow", "variance:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
