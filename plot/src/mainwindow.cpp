#include "include/mainwindow.h"

MainWindow::MainWindow(const char* serial_name, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    hero_plot = new HeroPlot(serial_name);
    setupPlot();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupPlot()
{
    ui->customPlot->legend->setVisible(true);
    ui->customPlot->legend->setFont(QFont("Helvetica",9));
    ui->customPlot->legend->setSelectableParts(QCPLegend::spItems);

    newGraph("x", QPen(QColor(0, 0, 255)));
    newGraph("y", QPen(QColor(255, 0, 0)));
    newGraph("z", QPen(QColor(0, 255, 0)));
    newGraph("yaw", QPen(QColor(255, 0, 255)));
    newGraph("pitch", QPen(QColor(255, 120, 120)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");
    ui->customPlot->xAxis->setTicker(timeTicker);
    ui->customPlot->axisRect()->setupFullAxesBox();
//    ui->customPlot->yAxis->setRange(-92, 92);

    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                    QCP::iSelectLegend | QCP::iSelectPlottables);
    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));

    connect(hero_plot, SIGNAL(addPointSignal(int)), this, SLOT(addPoint(int)) );
    // setup a timer that repeatedly calls MainWindow::plot
    connect(&hero_plot->timer, SIGNAL(timeout()), this, SLOT(plot()));

    // setup policy and connect slot for context menu popup:
    ui->customPlot->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->customPlot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

    connect(ui->customPlot, SIGNAL(legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*,QMouseEvent*)), this,
            SLOT(legendDoubleClicked(QCPLegend*,QCPAbstractLegendItem*)));
    connect(ui->btn_freeze, SIGNAL(clicked(bool)), SLOT(freezeBtnClicked()));
    connect(ui->btn_clear, SIGNAL(clicked(bool)), SLOT(clearBtnClicked()));
    connect(ui->btn_detail, SIGNAL(clicked(bool)), SLOT(detailBtnClicked()));

    hero_plot->timer.start(0);
    ui->customPlot->replot();
}

void MainWindow::run()
{
    hero_plot->start();
}

void MainWindow::newGraph(QString name, QPen pen)
{
    ui->customPlot->addGraph();
    ui->customPlot->graph(graph_num_)->setName(name);
    ui->customPlot->graph(graph_num_)->setPen(pen);
    graph_num_++;
}

void MainWindow::addPoint(int type)
{
    QVector<QCPGraphData> *m_data = ui->customPlot->graph(type)->data()->coreData();

    QCPGraphData new_point;

    new_point.key = hero_plot->pub_time_key;
    new_point.value = hero_plot->pub_value_vec[type];
    m_data->append(new_point);

    if(m_data->size() > max_vec_size_)
    {
        m_data->pop_front();
    }

}

void MainWindow::plot()
{
    time_key = hero_plot->pub_time_key;
    if(time_key - last_time_key > 0.0002)
    {
        if(!plot_freeze_flag_)
        {
            plotAxes();
            for(int i = 0; i < graph_num_; ++i)
            {
                if(i == 0)
                {
//                    ui->customPlot->graph(i)->rescaleValueAxis();
                }
                else
                {
//                    ui->customPlot->graph(i)->rescaleValueAxis(true);
                }
            }
            ui->customPlot->replot();
        }
    }
    // show fps
    showFPS();

    last_time_key = time_key;
}

void MainWindow::plotAxes()
{
    for(int i = 0; i < graph_num_; ++i)
    {
        if(!ui->customPlot->graph(i)->data()->coreData()->isEmpty())
        {
            if(show_detail_flag_)
                showDetails();

            switch(i)
            {
                case 0:
                    ui->x_label->setText(QString("x: %1").arg(ui->customPlot->graph(i)->data()->coreData()->back().value, 0, 'f', 2));
                    break;
                case 1:
                    ui->y_label->setText(QString("y: %1").arg(ui->customPlot->graph(i)->data()->coreData()->back().value, 0, 'f', 2));
                    break;
                case 2:
                    ui->z_label->setText(QString("z: %1").arg(ui->customPlot->graph(i)->data()->coreData()->back().value, 0, 'f', 2));
                    break;
                case 3:
                    ui->yaw_label->setText(QString("yaw: %1").arg(ui->customPlot->graph(i)->data()->coreData()->back().value, 0, 'f', 2));
                    break;
                case 4:
                    ui->pitch_label->setText(QString("pitch: %1").arg(ui->customPlot->graph(i)->data()->coreData()->back().value, 0, 'f', 2));
                    break;
            }
        }
    }
//    qDebug() << "time: " << x_time_vec.back() << "value: " << x_vec.back();
//    qDebug() << "time size: " << x_time_vec.size() << "value size: " << x_vec.size();
    ui->customPlot->xAxis->setRange(hero_plot->pub_time_key, 1, Qt::AlignRight);
}

void MainWindow::showFPS()
{
    static double fps_last_time_key = 0;
    static int frameCount;
    ++frameCount;
    if (time_key - fps_last_time_key > 1) // average fps over 1 seconds
    {
      ui->statusBar->showMessage(
            QString("FPS : %1\n detail : %2")
            .arg(frameCount/(time_key-fps_last_time_key), 0, 'f', 0)
            .arg(show_detail_flag_), 0);
      fps_last_time_key = time_key;
      frameCount = 0;
    }
}

void MainWindow::showDetails()
{
    static double min[5]{9999.9, 9999.9, 9999.9, 9999.9, 9999.9};
    static double max[5]{-9999.9, -9999.9, -9999.9, -9999.9, -9999.9};
    static double mean[5]{0,0,0,0,0};
    static double variance[5]{0,0,0,0,0};
    static double detail_last_time_key = 0;

    if(time_key - detail_last_time_key > 1)
    {
        if(ui->customPlot->selectedGraphs().size() > 0)
        {
            if(ui->customPlot->selectedGraphs().first() == ui->customPlot->graph(0))
            {
                QVector<QCPGraphData> *m_data = ui->customPlot->selectedGraphs().first()->data()->coreData();
                calMax(m_data, max[0]);
                calMin(m_data, min[0]);
                calMV(m_data, variance[0], mean[0]);

                ui->max_lable->setText(QString("max: %1").arg(max[0], 0, 'f', 2));
                ui->min_lable->setText(QString("min: %1").arg(min[0], 0, 'f', 2));
                ui->mean_lable->setText(QString("mean: %1").arg(mean[0], 0, 'f', 2));
                ui->var_lable->setText(QString("variance: %1").arg(variance[0], 0, 'f', 2));
            }
            else if(ui->customPlot->selectedGraphs().first() == ui->customPlot->graph(1))
            {
                QVector<QCPGraphData> *m_data = ui->customPlot->selectedGraphs().first()->data()->coreData();
                calMax(m_data, max[1]);
                calMin(m_data, min[1]);
                calMV(m_data, variance[1], mean[1]);

                ui->max_lable->setText(QString("max: %1").arg(max[1], 0, 'f', 2));
                ui->min_lable->setText(QString("min: %1").arg(min[1], 0, 'f', 2));
                ui->mean_lable->setText(QString("mean: %1").arg(mean[1], 0, 'f', 2));
                ui->var_lable->setText(QString("variance: %1").arg(variance[1], 0, 'f', 2));
            }
            else if(ui->customPlot->selectedGraphs().first() == ui->customPlot->graph(2))
            {
                QVector<QCPGraphData> *m_data = ui->customPlot->selectedGraphs().first()->data()->coreData();
                calMax(m_data, max[2]);
                calMin(m_data, min[2]);
                calMV(m_data, variance[2], mean[2]);

                ui->max_lable->setText(QString("max: %1").arg(max[2], 0, 'f', 2));
                ui->min_lable->setText(QString("min: %1").arg(min[2], 0, 'f', 2));
                ui->mean_lable->setText(QString("mean: %1").arg(mean[2], 0, 'f', 2));
                ui->var_lable->setText(QString("variance: %1").arg(variance[2], 0, 'f', 2));
            }
            else if(ui->customPlot->selectedGraphs().first() == ui->customPlot->graph(3))
            {
                QVector<QCPGraphData> *m_data = ui->customPlot->selectedGraphs().first()->data()->coreData();
                calMax(m_data, max[3]);
                calMin(m_data, min[3]);
                calMV(m_data, variance[3], mean[3]);

                ui->max_lable->setText(QString("max: %1").arg(max[3], 0, 'f', 2));
                ui->min_lable->setText(QString("min: %1").arg(min[3], 0, 'f', 2));
                ui->mean_lable->setText(QString("mean: %1").arg(mean[3], 0, 'f', 2));
                ui->var_lable->setText(QString("variance: %1").arg(variance[3], 0, 'f', 2));
            }
            else if(ui->customPlot->selectedGraphs().first() == ui->customPlot->graph(4))
            {
                QVector<QCPGraphData> *m_data = ui->customPlot->selectedGraphs().first()->data()->coreData();
                calMax(m_data, max[4]);
                calMin(m_data, min[4]);
                calMV(m_data, variance[4], mean[4]);

                ui->max_lable->setText(QString("max: %1").arg(max[4], 0, 'f', 2));
                ui->min_lable->setText(QString("min: %1").arg(min[4], 0, 'f', 2));
                ui->mean_lable->setText(QString("mean: %1").arg(mean[4], 0, 'f', 2));
                ui->var_lable->setText(QString("variance: %1").arg(variance[4], 0, 'f', 2));
            }
        }
    detail_last_time_key = time_key;
    }
}

void MainWindow::contextMenuRequest(QPoint pos)
{
    QMenu *menu = new QMenu(this);
    menu->setAttribute(Qt::WA_DeleteOnClose);

    if (ui->customPlot->legend->selectTest(pos, false) >= 0) // context menu on legend requested
    {
        menu->addAction("Move to top left", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignLeft));
        menu->addAction("Move to top center", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignHCenter));
        menu->addAction("Move to top right", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignRight));
        menu->addAction("Move to bottom right", this, SLOT(moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignRight));
        menu->addAction("Move to bottom left", this, SLOT(moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignLeft));
    }
    else  // general context menu on graphs requested
    {
        if (ui->customPlot->selectedGraphs().size() > 0)
        {
            menu->addAction("Hide selected graph", this, SLOT(hideSelectedGraph()));
        }
        if (ui->customPlot->graphCount() > 0)
        {
            menu->addAction("Hide all graphs", this, SLOT(hideAllGraphs()));
            menu->addAction("Refresh graphs", this, SLOT(refreshGraphs()));
        }
    }

    menu->popup(ui->customPlot->mapToGlobal(pos));
}

void MainWindow::hideSelectedGraph()
{
    ui->customPlot->selectedGraphs().first()->setVisible(false);
    ui->customPlot->replot();
}

void MainWindow::hideAllGraphs()
{
    for(int i = 0; i < graph_num_; ++i)
    {
        ui->customPlot->graph(i)->setVisible(false);
    }
    ui->customPlot->replot();
}

void MainWindow::refreshGraphs()
{
    for(int i = 0; i < graph_num_; ++i)
    {
        ui->customPlot->graph(i)->setVisible(true);
    }
    ui->customPlot->replot();
}

void MainWindow::moveLegend()
{
    // make sure this slot is really called by a context menu action, so it carries the data we need
    if (QAction* contextAction = qobject_cast<QAction*>(sender()))
    {
      bool ok;
      int dataInt = contextAction->data().toInt(&ok);
      if (ok)
      {
          ui->customPlot->axisRect()->insetLayout()->setInsetAlignment(0, (Qt::Alignment)dataInt);
          ui->customPlot->replot();
      }
    }
}

void MainWindow::legendDoubleClicked(QCPLegend *legend, QCPAbstractLegendItem *item)
{
  // Rename a graph by double clicking on its legend item
  Q_UNUSED(legend)
  if (item) // only react if item was clicked (user could have clicked on border padding of legend where there is no item, then item is 0)
  {
    QCPPlottableLegendItem *plItem = qobject_cast<QCPPlottableLegendItem*>(item);
    bool ok;
    QString newName = QInputDialog::getText(this, "QCustomPlot example", "New graph name:", QLineEdit::Normal, plItem->plottable()->name(), &ok);
    if (ok)
    {
      plItem->plottable()->setName(newName);
      ui->customPlot->replot();
    }
  }
}

void MainWindow::clearBtnClicked()
{
    for(int i = 0; i < graph_num_; ++i)
    {
        ui->customPlot->graph(i)->data()->coreData()->clear();
    }
    ui->customPlot->replot();
}

void MainWindow::freezeBtnClicked()
{
    plot_freeze_flag_ = !plot_freeze_flag_;
}

void MainWindow::detailBtnClicked()
{
    show_detail_flag_ = !show_detail_flag_;
}

template<typename T>
inline void MainWindow::calMax(const QVector<QCPGraphData> *m_data, T &max)
{
    for(auto i = m_data->begin(); i != m_data->end(); ++i)
    {
        if(i->value > max)
        {
            max = i->value;
        }
    }
}

template<typename T>
inline void MainWindow::calMin(const QVector<QCPGraphData> *m_data, T &min)
{
    for(auto i = m_data->begin(); i != m_data->end(); ++i)
    {
        if(i->value < min)
        {
            min = i->value;
        }
    }
}

template<typename T>
inline void MainWindow::calMV(const QVector<QCPGraphData> *m_data, T &variance, T &mean)
{
    mean = 0; variance = 0;
    T sum = 0;
    for(auto i = m_data->begin(); i != m_data->end(); ++i)
    {
        sum += i->value;
    }

    mean = sum / m_data->size();
    for(auto i = m_data->begin(); i != m_data->end(); ++i)
    {
        variance += pow( i->value-mean, 2);
    }
}

void MainWindow::testPlot()
{
//    if(hero_plot->pub_value_vecs[0].size() > 1)
//    {
////        mutex_.lock();
//        ui->customPlot->graph(0)->addData(hero_plot->pub_time_vecs[0].back(), hero_plot->pub_value_vecs[0].back());
////        ui->customPlot->graph(0)->setData(hero_plot->pub_time_vecs[0], hero_plot->pub_value_vecs[0]);
//        ui->customPlot->replot();
////        mutex_.unlock();
//    }
}





