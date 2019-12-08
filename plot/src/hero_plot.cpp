#include "include/hero_plot.h"

HeroPlot::HeroPlot(const char* serial_name)
{
    std::string dev_name(serial_name);
    serialport.open(dev_name);
    setupParam();
}

void HeroPlot::run()
{
    while(true)
    {
        usleep(1000);
        serialport.readData(read_pack_);
        if(read_pack_.plot_type >= 0 && read_pack_.plot_type <= 4)
        {
            if(read_pack_.curve_num <=3)
            {
                for(int i = 0; i < read_pack_.curve_num; ++i)
                {

                    addPoint(read_pack_.plot_value[i], (i+read_pack_.plot_type) % 5);
                }
            }
            else
            {
                std::cout << "curve_num error: " << read_pack_.curve_num << std::endl;
            }
        }
        else
        {
            std::cout << "type error: " << read_pack_.plot_type << std::endl;
        }
    }
}

void HeroPlot::setupParam()
{
    for(int i = 0; i < 5; ++i)
    {
        pub_value_vec.append(0);
    }
    read_pack_.curve_num = 0;
    read_pack_.plot_type = -1;
}

void HeroPlot::addPoint(double value, int type)
{
    static QTime time(QTime::currentTime());
    double time_from_start = time.elapsed()/1000.0; // time elapsed since start, in seconds

    pub_time_key = time_from_start;
    pub_value_vec[type] = value;

    emit addPointSignal(type);
}
