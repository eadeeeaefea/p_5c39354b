/******************************************************************************
 Copyright© HITwh HERO-Robomaster2020 Group

 Author: Wang Xiaoyan on 2019.9.20

 Detail:
 *****************************************************************************/

#include "anglesolver.h"

using namespace std;


AngleSolver::AngleSolver() {

}

AngleSolver::~AngleSolver() {

}

void AngleSolver::init() {
    yaw_offset_ = 0.6;
    pitch_offset_ = 1.2;//经测试得到的参数
}

void AngleSolver::newrun(double x, double y, double z, double v,
                               double& yaw, double& pitch, double ptz_pitch) {
    Timer timer;
    timer.start();
 
/*
  positive direction of axis Y in PNP
  and this part are converse
*/
    y = -y;
 
 double y1,z1,ptz;
    y1=y;
    z1=z;
    ptz=(ptz_pitch/180)*PI;
    y=y1*cos(ptz)+z1*sin(ptz);
    z=z1*cos(ptz)-y1*sin(ptz);//将坐标旋转回标准坐标系(ｙ轴竖直向下）

    yaw = (atan(x / z) / PI) * 180 + yaw_offset;//计算ｙａｗ轴

    /*caculate pitch_max and pitch_min*/
    double pitch_min, pitch_max, mid;

    pitch_min = atan2(y, sqrt(x * x + z * z))-0.05;
    pitch_max = pitch_min+0.6;
    //cout << "pitch_min=" << pitch_min << endl;
    //cout << "pitch_max=" << pitch_max << endl;//ｄｅｂｕｇ备用

    double t1, t2, delta_y1, delta_y2, differ1, differ2;//计算上限和下限角度对应落点与目标差值（正常应一正一负）
    t1 = sqrt(x * x + z * z) / (v * cos(pitch_min));
    delta_y1 = v * sin(pitch_min) * t1 - 0.5 * g * t1 * t1;
    differ1 = delta_y1 - y;

    t2 = sqrt(x * x + z * z) / (v * cos(pitch_max));
    delta_y2 = v * sin(pitch_max) * t2 - 0.5 * g * t2 * t2;
    differ2 = delta_y2 - y;

    //cout << "differ1=" << differ1 << endl;
    //cout << "differ2=" << differ2 << endl;//debug备用

    if (fabs(differ1) <= 0.001) { pitch = pitch_min; }
    else if (fabs(differ2) <= 0.001) { pitch = pitch_max; }
    else if (differ1 * differ2 > 0) { return; }//提前排除不需二分的情况

/*dichotomy,loop for several times until differ is small enough*/
    if (differ1 * differ2 < 0) {
        int i = 0;
        double delta_y = 0, t = 0, differ = 0;
        for (i = 0; i < 20; i++) {
            mid = (pitch_max + pitch_min) / 2;
            t = sqrt(x * x + z * z) / (v * cos(mid));
            delta_y = v * sin(mid) * t - 0.5 * g * t * t;
            differ = delta_y - y;
            if (fabs(differ) <= 0.001 && (i == 19)) {
                pitch = mid;
                break;
            } else if (differ * differ1 > 0) {
                pitch_min = mid;
            } else {
                pitch_max = mid;
            }
        }
    }


    pitch=(pitch/PI)*180+temp_pitch_offset-ptz_pitch;//计算得ｐｉｔｃｈ轴角度需减去云台绝对角度

    cout << "Anglesolver newrun:  yaw=" << yaw << "  pitch=" << pitch << endl;
    cout << "Anglesolver newrun: " << timer.getTime() << "ms" << endl;
    timer.stop();
}


double AngleSolver::get_yaw_offset() {
    return yaw_offset_;
}

double AngleSolver::get_pitch_offset() {
    return pitch_offset_;
}

void AngleSolver::set_yaw_offset(double yaw_offset) {
    yaw_offset_ = yaw_offset;
}

void AngleSolver::set_pitch_offset(double pitch_offset) {
    pitch_offset_ = pitch_offset;
}

// 未加坐标旋转变换，稍微慢一些的两个算法，速度矢量三角计算快于二分法。
//二分法算法原理同上
  void AngleSolver::newdichotomy(double x, double y, double z, double v,
                      double& yaw, double& pitch, double ptz_pitch) {
      Timer timer;
      timer.start();

      double temp_pitch_offset = pitch_offset;

/*
  positive direction of axis Y in PNP
  and this part are converse
*/
      y = -y;
      yaw = (atan(x / z) / PI) * 180 + yaw_offset;

      /*caculate pitch_max and pitch_min*/
      double pitch_min, pitch_max, mid;

      pitch_min = atan2(y, sqrt(x * x + z * z))-0.05;
      pitch_max = pitch_min+0.6;
      //cout << "pitch_min=" << pitch_min << endl;
      //cout << "pitch_max=" << pitch_max << endl;

      double t1, t2, delta_y1, delta_y2, differ1, differ2;
      t1 = sqrt(x * x + z * z) / (v * cos(pitch_min));
      delta_y1 = v * sin(pitch_min) * t1 - 0.5 * g * t1 * t1;
      differ1 = delta_y1 - y;

      t2 = sqrt(x * x + z * z) / (v * cos(pitch_max));
      delta_y2 = v * sin(pitch_max) * t2 - 0.5 * g * t2 * t2;
      differ2 = delta_y2 - y;

      //cout << "differ1=" << differ1 << endl;
      //cout << "differ2=" << differ2 << endl;

      if (fabs(differ1) <= 0.001) { pitch = pitch_min; }
      else if (fabs(differ2) <= 0.001) { pitch = pitch_max; }
      else if (differ1 * differ2 > 0) { return; }

/*dichotomy,loop for several times until differ is small enough*/
      if (differ1 * differ2 < 0) {
          int i = 0;
          double delta_y = 0, t = 0, differ = 0;
          for (i = 0; i < 20; i++) {
              mid = (pitch_max + pitch_min) / 2;
              t = sqrt(x * x + z * z) / (v * cos(mid));
              delta_y = v * sin(mid) * t - 0.5 * g * t * t;
              differ = delta_y - y;
              if (fabs(differ) <= 0.001 && (i == 19)) {
                  pitch = mid;
                  break;
              } else if (differ * differ1 > 0) {
                  pitch_min = mid;
              } else {
                  pitch_max = mid;
              }
          }
          }


          pitch = (pitch / PI) * 180 + temp_pitch_offset;
          //pitch=(pitch/PI)*180+temp_pitch_offset-ptz_pitch;

          cout << "NewDichotomy:  yaw=" << yaw << "  pitch=" << pitch << endl;
          cout << "NewDichotomy time: " << timer.getTime() << "ms" << endl;
          timer.stop();
      }


    void AngleSolver::speedtriangle(double x, double y, double z, double v,
                                   double& yaw, double& pitch, double ptz_pitch) {
        Timer timer;
        timer.start();

        double temp_pitch_offset = pitch_offset;

/*
        positive direction of axis Y in PNP
                                        and this part don't need to converse
        */



        yaw = (atan(x / z) / PI )* 180 + yaw_offset;

　　　　//由速度矢量三角形推导出的公式实现
        double speed_angle,sin_speed_angle,pitch_tan_y,pitch_tan_x,mid_value;
        sin_speed_angle=g*sqrt(x*x+z*z)/(v*sqrt(v*v+2*g*y));
        speed_angle=asin(sin_speed_angle);
        mid_value=v-cos(speed_angle)*sqrt(v*v+2*g*y);
        pitch_tan_y=v*fabs(mid_value);
        pitch_tan_x=g*sqrt(x*x+z*z);
        pitch=atan2(pitch_tan_y,pitch_tan_x);

        if(mid_value<0)
        {pitch=-pitch;}


            pitch = (pitch/PI)*180 + temp_pitch_offset;
            cout<<"SpeedTriangle:  yaw="<<yaw<<"  pitch="<<pitch<<endl;
            cout << "SpeedTriangle time: " << timer.getTime() << "ms" << endl;
            timer.stop();
        }

// 上届二分法
void AngleSolver::run(double x, double y, double z, double v,
                      double &yaw, double &pitch) {
#ifdef RUNNING_TIME
    Timer timer;
    timer.start();
#endif

    if (parabolaSolve(sqrt(x*x+z*z), y, v, pitch)) {
        pitch = -pitch + pitch_offset_;
        yaw = atan(x / z) / PI * 180 + yaw_offset_;
    } else {
        pitch = 0.0;
        yaw = 0.0;
    }

#ifdef RUNNING_TIME
    cout << "solve angle time: " << timer.getTime() << "ms" << endl;
    timer.stop();
#endif
}


bool AngleSolver::parabolaSolve(double x, double y, double v, double &theta) {
    double min_theta = atan(y / x) - 0.1;
    double max_theta = min_theta + 0.5;

    double flag1 = parabolaDeltaY(x, y, v, min_theta) * parabolaDeltaY(x, y, v, max_theta);
    double flag2 = parabolaDeltaY(x, y, v, min_theta) - parabolaDeltaY(x, y, v, max_theta);
    double mid;

    if (flag1 > 0) {
        return 0;
    } else if (parabolaDeltaY(x, y, v, max_theta) == 0) {
        theta = max_theta / PI * 180;
        return 1;
    } else {
        if (flag2 < 0) {
            while (max_theta - min_theta > 0.0001) {
                mid = (max_theta + min_theta) / 2;
                if (parabolaDeltaY(x, y, v, mid) < 0) {
                    min_theta = mid;
                } else if (parabolaDeltaY(x, y, v, mid) > 0) {
                    max_theta = mid;
                } else {
                    break;
                }
            }
            theta = max_theta / PI * 180;
            return 1;
        } else if (flag2 > 0) {
            while (max_theta - min_theta > 0.0001) {
                mid = (max_theta + min_theta) / 2;
                if (parabolaDeltaY(x, y, v, mid) < 0) {
                    max_theta = mid;
                } else if (parabolaDeltaY(x, y, v, mid) > 0) {
                    min_theta = mid;
                } else {
                    break;
                }
            }
            theta = max_theta / PI * 180;
            return 1;
        } else {
            theta = max_theta / PI * 180;
            return 1;
        }
    }
}

double AngleSolver::parabolaDeltaY(double x, double y, double v, double theta) {
    static const double g = 9.7988;

    double vx = v * cos(theta);
    double vy = v * sin(theta);
    double t = x / vx;
    double delta_y = vy * t + 0.5 * g * t * t - y;  // 云台坐标系y轴向下，g向下，故g的符号为正
    // double a = ((-k) / m) * pow(m*g/k, 0.5);
    // double b = atan(vy / pow(m*g/k, 0.5));
    // double t = (m / k / vx) * exp(k*x/m - 1);
    // double delta_y = pow(m*g/k, 0.5) * log(1/cos(a*t+b)) / a - log(1/cos(b)) / a - y;
    return delta_y;
}
