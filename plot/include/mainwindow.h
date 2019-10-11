#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMutex>
#include <QDebug>
#include <QVector>

#include "qcustomplot.h"
#include "ui_mainwindow.h"
#include "include/hero_plot.h"

//#define SHOW_DETAIL

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

private:
    Ui::MainWindow *ui;

    // main thread
    HeroPlot *hero_plot;

    // control variables
    int graph_num_ = 0;
    bool plot_freeze_flag_ = false;
    int max_vec_size_ = 17000;
    double time_key = 0;
    double last_time_key = 0;

    // locker
    QMutex mutex_;

    // plot vectors
    QVector<QVector<double> > value_vecs_;
    QVector<double> time_vec;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void run();

private:
    void setupPlot();
    void newGraph(QString name, QPen pen);
    void plotAxes();
    void showFPS();
    void showDetails(HeroPlot::PLOT_TYPE graph_index);

    template<typename T>
    inline void calMax(const QVector<QCPGraphData> *m_data, T &max);
    template<typename T>
    inline void calMin(const QVector<QCPGraphData> *m_data, T &min);
    // calculate mean and variance
    template<typename T>
    inline void calMV(const QVector<QCPGraphData> *m_data, T &variance, T &mean);

private slots:
    void addPoint(int);
    void plot();
    void contextMenuRequest(QPoint pos);
    void hideSelectedGraph();
    void hideAllGraphs();
    void refreshGraphs();
    void moveLegend();
    void legendDoubleClicked(QCPLegend *legend, QCPAbstractLegendItem *item);
    void clearBtnClicked();
    void freezeBtnClicked();

    void testPlot();

};

#endif // MAINWINDOW_H
